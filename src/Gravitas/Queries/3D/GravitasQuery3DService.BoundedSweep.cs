//=======================================================================
// GravitasQuery3DService.BoundedSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Queries;

/// <content>Queries bounded 3D sphere sweeps using a ready partition view.</content>
public sealed partial class GravitasQuery3DService
{
    // The rounded-box kernel examines three axes with two breakpoint faces each,
    // sorts at most six breakpoints (15 comparisons), consumes six breakpoints,
    // and solves at most seven intervals (three axis terms and one fixed-width
    // quadratic per interval). Reserve one worker bounds check and one contact reduction.
    private const ulong CuboidSweepGeometry = 6 + 15 + 6 + 7 * (3 + 1) + 2;
    // Each includes the worker bounds test, entry roots, containment and contact.
    private const ulong SphereSweepGeometry = 5;
    private const ulong CapsuleSweepGeometry = 10;
    // Rigid triangles visit the initial witness, two faces, three edges and
    // three vertices, with eight hit reductions. The affine fallback can
    // inspect three vertices twice, seven features with three roots each,
    // and three edge constraints per face root.
    private const ulong MeshTriangleSweepGeometry =
        (3 + 1 + 2 + 3 * CapsuleSweepGeometry + 3 * SphereSweepGeometry + 8)
        + (6 + 7 * (1 + 3) + 3 * 3);

    /// <summary>Finds the closest swept-sphere hit within caller-owned work and storage limits.</summary>
    /// <remarks>
    /// Run on the simulation thread between completed pure 3D fixed steps.
    /// Only <see cref="PhysicsQueryExecutionStatus.Completed"/> certifies a result.
    /// A null hit on completion means the sweep missed.
    /// </remarks>
    public PhysicsQueryExecutionStatus SweepSphere(
        Vector3d origin, Fixed64 radius, Vector3d direction, Fixed64 maxDistance,
        PhysicsLayerMask layerMask, LSCollider? excludedCollider,
        in PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
        out Physics3DHit? hit, out PhysicsQueryWork work)
    {
        ChronicleGuard.ThrowIfNull(scratch, nameof(scratch));
        ChronicleGuard.ThrowIfArgument(radius > ColliderGeometryPolicy.MaximumGeometryReach,
            nameof(radius), "Effective geometry reach must not exceed 4,096 world units.");
        hit = null;
        work = default;

        int voxelCount = 0;
        var meter = new PhysicsQueryMeter(budget);
        scratch.Borrow();
        try
        {
            if (_context.Settings.RuntimeMode != PhysicsRuntimeMode.ThreeD)
                return PhysicsQueryExecutionStatus.UnsupportedRuntimeMode;
            if (!_context.BoundedQueryPartitionsReady)
                return PhysicsQueryExecutionStatus.WorldNotReady;
            if (radius <= Fixed64.Zero || direction == Vector3d.Zero || maxDistance <= Fixed64.Zero)
                return PhysicsQueryExecutionStatus.Completed;
            if (!TryGetSweepEndpoint(origin, direction, maxDistance, out Vector3d end)
                || !TryGetSweepSegment(origin, end, out Vector3d segment, out Fixed64 length))
                return PhysicsQueryExecutionStatus.Unrepresentable;
            if (length <= Fixed64.Epsilon)
                return PhysicsQueryExecutionStatus.Completed;
            Vector3d extents = Vector3d.One * radius;
            if (!Vector3d.TrySubtract(Vector3d.Min(origin, end), extents, out Vector3d min)
                || !Vector3d.TryAdd(Vector3d.Max(origin, end), extents, out Vector3d max))
                return PhysicsQueryExecutionStatus.Unrepresentable;

            PhysicsQueryExecutionStatus status = scratch.Discover(_context, min, max, false,
                ref meter, out voxelCount, out ulong changeSequence);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            _sweepWorker.Prepare(origin, end, radius);
            status = ScanBoundedSweepVoxels(voxelCount, segment.Normalized, layerMask,
                excludedCollider, scratch, ref meter, out Physics3DHit? closest);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;

            status = PhysicsQueryScratch.CheckView(_context, changeSequence, ref meter);
            if (status == PhysicsQueryExecutionStatus.Completed)
                hit = closest;
            return status;
        }
        finally
        {
            work = meter.Work;
            scratch.Release(voxelCount);
        }
    }

    private PhysicsQueryExecutionStatus ScanBoundedSweepVoxels(
        int count, Vector3d direction, PhysicsLayerMask layerMask, LSCollider? excludedCollider,
        PhysicsQueryScratch scratch, ref PhysicsQueryMeter meter, out Physics3DHit? closest)
    {
        closest = null;
        var voxels = scratch.Voxels;
        for (int i = 0; i < count; i++)
        {
            var status = PhysicsQueryScratch.GetNativePartition<PhysicsPartition, PhysicsPartition2D>(
                voxels[i]!, ref meter, out var partition);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (partition == null)
                continue;

            status = ScanBoundedSweepMemberships(partition.ContainedDynamicObjects,
                direction, layerMask, excludedCollider, scratch, ref meter, ref closest);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            status = ScanBoundedSweepMemberships(partition.ContainedKinematicObjects,
                direction, layerMask, excludedCollider, scratch, ref meter, ref closest);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            status = ScanBoundedSweepMemberships(partition.ContainedStaticObjects,
                direction, layerMask, excludedCollider, scratch, ref meter, ref closest);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
        }

        return PhysicsQueryExecutionStatus.Completed;
    }

    private PhysicsQueryExecutionStatus ScanBoundedSweepMemberships(
        SwiftSortedList<int>? memberships, Vector3d direction, PhysicsLayerMask layerMask,
        LSCollider? excludedCollider, PhysicsQueryScratch scratch, ref PhysicsQueryMeter meter,
        ref Physics3DHit? closest)
    {
        if (memberships == null || memberships.Count == 0)
            return PhysicsQueryExecutionStatus.Completed;
        if (!meter.ChargeCandidates((ulong)memberships.Count))
            return PhysicsQueryExecutionStatus.WorkBudgetExceeded;

        foreach (int id in memberships.AsReadOnlySpan())
        {
            if (!_context.Physics.TryGetColliderById(id, out LSCollider? collider)
                || ReferenceEquals(collider, excludedCollider) || !layerMask.Includes(collider!.Layer))
                continue;
            var status = scratch.RememberCandidate(id, ref meter, out bool isNew);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (!isNew)
                continue;
            if (excludedCollider != null)
            {
                var constraints = _context.Constraints3D;
                if (!meter.ChargeCandidates(constraints.LinkedCollisionProbeLimit))
                    return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
                if (constraints.ShouldExcludeLinkedCollision(excludedCollider, collider!))
                    continue;
            }
            if (!meter.ChargeGeometry(1))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (!_sweepWorker.BoundsOverlap(collider!))
                continue;
            status = ChargeSweepGeometry(collider!, scratch.Mesh, ref meter);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (TryBuildPreparedSweepHit(collider, direction, out Physics3DHit candidate, scratch.Mesh)
                && PhysicsHitSelectionPolicy.ShouldReplace(candidate, closest.HasValue, closest.GetValueOrDefault()))
                closest = candidate;
        }

        return PhysicsQueryExecutionStatus.Completed;
    }

    private static PhysicsQueryExecutionStatus ChargeSweepGeometry(
        LSCollider collider, MeshQueryScratch? meshScratch, ref PhysicsQueryMeter meter)
    {
        if (collider is LSCompoundCollider compound)
        {
            // Prepay the admission, sweep and witness passes before accessing any part.
            if (!meter.ChargeGeometry(3 * (ulong)compound.PartCount + 1))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            for (int i = 0; i < compound.PartCount; i++)
            {
                var status = ChargeSweepLeaf(compound.GetPartCollider(i), meshScratch, ref meter);
                if (status != PhysicsQueryExecutionStatus.Completed)
                    return status;
            }
            return PhysicsQueryExecutionStatus.Completed;
        }

        return ChargeSweepLeaf(collider, meshScratch, ref meter);
    }

    private static PhysicsQueryExecutionStatus ChargeSweepLeaf(
        LSCollider collider, MeshQueryScratch? meshScratch, ref PhysicsQueryMeter meter)
    {
        ulong geometry;
        switch (collider)
        {
            case LSCuboidCollider:
                geometry = CuboidSweepGeometry;
                break;
            case LSSphereCollider:
                geometry = SphereSweepGeometry;
                break;
            case LSCapsuleCollider:
                geometry = CapsuleSweepGeometry;
                break;
            case LSCylinderCollider or LSConeCollider:
                geometry = ConvexSweepQueryWorker.PrimitiveSweepWork;
                break;
            default:
                // Compounds contain only native leaves. Mesh is the remaining native leaf.
                var mesh = (LSMeshCollider)collider;
                ulong triangles = (ulong)mesh.Mesh.TriangleCount;
                if (meshScratch == null || (ulong)meshScratch.Capacity < triangles)
                    return PhysicsQueryExecutionStatus.ScratchCapacityExceeded;
                // Two traversals, each with a push, pop and bounds test per node
                // and at most one output per leaf. A patch fan has at most
                // three authored corners per triangle and the patch check scans
                // the triangle owners. Each witness visit fetches three vertices,
                // checks a face and three edges, and reduces the nearest point.
                ulong traversal = 2 * (3 * (2 * triangles - 1) + triangles);
                geometry = traversal + triangles + 3 * triangles * MeshTriangleSweepGeometry
                    + (triangles + 1) * (3 + 1 + 3 + 2) + 2;
                break;
        }

        return meter.ChargeGeometry(geometry)
            ? PhysicsQueryExecutionStatus.Completed : PhysicsQueryExecutionStatus.WorkBudgetExceeded;
    }
}
