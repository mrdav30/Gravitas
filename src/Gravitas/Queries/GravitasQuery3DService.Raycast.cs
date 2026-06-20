using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge;
using GridForge.Grids;
using GridForge.Utility;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns 3D raycast, swept-sphere, and X/Z circle query buffers for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery3DService
{
    private readonly GravitasWorldContext _context;
    private readonly RaycastSegmentWorker _worker = new();
    private readonly SweptSphereQueryWorker _sweepWorker = new();
    private SwiftList<Vector3d> _bufferIntersectionPoints = new();
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();
    private readonly SwiftHashSet<int> _redundantVoxelCheck = new();
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly GridTraceScratch _traceScratch = new();

    private PhysicsLayerMask _currentLayerMask;
    private LSCollider? _currentExcludedCollider;
    private bool _currentStaticSweepTargetsOnly;
    private bool _currentIncludeTriggers;

    /// <summary>
    /// Initializes a new raycast service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasQuery3DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the context-local raycast/sweep query version.
    /// </summary>
    public uint RaycastVersion { get; private set; }

    /// <summary>
    /// Gets the context-local 3D X/Z circle query version.
    /// </summary>
    public uint CircleVersion { get; private set; }

    internal int LastQueryCandidateCount { get; private set; }

    /// <summary>
    /// Resets context-local raycast query buffers.
    /// </summary>
    public void Reset()
    {
        RaycastVersion = 0;
        CircleVersion = 0;
        LastQueryCandidateCount = 0;
        _bufferIntersectionPoints.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        _coveredVoxels.FastClear();
        _traceScratch.Clear();
    }

    /// <summary>
    /// Performs a raycast from an origin in a direction up to a maximum distance.
    /// </summary>
    public bool Raycast(
        Vector3d origin,
        Vector3d direction,
        Fixed64 maxDistance,
        out Physics3DHit raycastHit,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        if (direction.MagnitudeSquared == Fixed64.Zero || maxDistance <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            raycastHit = default;
            return false;
        }

        Vector3d rayDirection = direction.Normalized;
        Vector3d end = origin + rayDirection * maxDistance;

        BeginRaycastTrace(origin, end);
        bool hit = TryFindClosestHit(origin, end, rayDirection, out raycastHit);
        _context.Diagnostics.EmitRayQuery(
            origin,
            end,
            Fixed64.Zero,
            layerMask.Bits,
            hit,
            hit ? 1 : 0,
            raycastHit);
        return hit;
    }

    /// <summary>
    /// Executes a raycast between two points and writes hits from closest to farthest into caller-owned storage.
    /// </summary>
    public int RaycastAll(
        Vector3d start3d,
        Vector3d end3d,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        _currentLayerMask = layerMask;
        results.FastClear();

        Vector3d segment = end3d - start3d;
        if (segment.MagnitudeSquared == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        BeginRaycastTrace(start3d, end3d);
        AddAllHits(start3d, end3d, segment.Normalized, results);
        Physics3DHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitRayQuery(
            start3d,
            end3d,
            Fixed64.Zero,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    /// <summary>
    /// Sweeps a sphere from an origin in a direction up to a maximum distance and returns the closest hit.
    /// </summary>
    public bool SweepSphere(
        Vector3d origin,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistance,
        out Physics3DHit sweepHit,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider = null)
    {
        if (direction.MagnitudeSquared == Fixed64.Zero || maxDistance <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            sweepHit = default;
            return false;
        }

        Vector3d sweepDirection = direction.Normalized;
        Vector3d end = origin + sweepDirection * maxDistance;
        bool hit = SweepSphere(origin, end, radius, layerMask, excludedCollider, out sweepHit);
        _context.Diagnostics.EmitRayQuery(
            origin,
            end,
            radius,
            layerMask.Bits,
            hit,
            hit ? 1 : 0,
            sweepHit);
        return hit;
    }

    /// <summary>
    /// Sweeps a sphere between two points and writes hits from closest to farthest into caller-owned storage.
    /// </summary>
    public int SweepSphereAll(
        Vector3d start3d,
        Vector3d end3d,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null)
    {
        return SweepSphereAllCore(
            start3d,
            end3d,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers: true,
            staticTargetsOnly: false);
    }

    internal int SweepSphereAgainstStaticAll(
        Vector3d start3d,
        Vector3d end3d,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepSphereAllCore(
            start3d,
            end3d,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true);
    }

    private int SweepSphereAllCore(
        Vector3d start3d,
        Vector3d end3d,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        results.FastClear();
        Vector3d segment = end3d - start3d;
        if (segment.MagnitudeSquared == Fixed64.Zero || radius <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        BeginSweepTrace(start3d, end3d, radius, layerMask, excludedCollider, includeTriggers, staticTargetsOnly);
        AddAllSweepHits(start3d, end3d, segment.Normalized, radius, results);
        Physics3DHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitRayQuery(
            start3d,
            end3d,
            radius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    private void BeginRaycastTrace(Vector3d start, Vector3d end)
    {
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        _bufferIntersectionPoints.FastClear();
        _currentExcludedCollider = null;
        _currentStaticSweepTargetsOnly = false;
        _currentIncludeTriggers = true;
        LastQueryCandidateCount = 0;
        RaycastVersion++;
        _worker.PrepareSegmentCheck(start, end);
    }

    private void BeginSweepTrace(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        bool includeTriggers = true,
        bool staticTargetsOnly = false)
    {
        _currentLayerMask = layerMask;
        _currentExcludedCollider = excludedCollider;
        _currentIncludeTriggers = includeTriggers;
        _currentStaticSweepTargetsOnly = staticTargetsOnly;
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        LastQueryCandidateCount = 0;
        RaycastVersion++;
        _sweepWorker.Prepare(start, end, radius);
    }

    private bool SweepSphere(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        out Physics3DHit sweepHit)
    {
        sweepHit = default;
        if (radius <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return false;
        }

        Vector3d segment = end - start;
        if (segment.MagnitudeSquared == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return false;
        }

        Vector3d direction = segment.Normalized;
        BeginSweepTrace(start, end, radius, layerMask, excludedCollider);
        return TryFindClosestSweepHit(start, end, radius, direction, out sweepHit);
    }

    private bool TryFindClosestSweepHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        out Physics3DHit sweepHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        Physics3DHit closestHit = default;

        TraceSweepForClosestHit(start, end, radius, direction, ref found, ref closestDistance, ref closestHit);

        sweepHit = closestHit;
        return found;
    }

    private bool TryFindClosestHit(Vector3d start, Vector3d end, Vector3d direction, out Physics3DHit raycastHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        Physics3DHit closestHit = default;

        TraceLineForClosestHit(start, end, direction, ref found, ref closestDistance, ref closestHit);

        raycastHit = closestHit;
        return found;
    }

    private void AddAllHits(Vector3d start, Vector3d end, Vector3d direction, SwiftList<Physics3DHit> results)
    {
        TraceLineForAllHits(start, end, direction, results);
    }

    private void AddAllSweepHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 radius,
        SwiftList<Physics3DHit> results)
    {
        TraceSweepForAllHits(start, end, radius, direction, results);
    }

    private void TraceLineForClosestHit(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        foreach (GridVoxelSet covered in GridTracer.TraceLine(_context.World, start, end))
        {
            foreach (Voxel voxel in covered.Voxels)
                ProcessTraceVoxelForClosestHit(
                    voxel,
                    start,
                    direction,
                    ref found,
                    ref closestDistance,
                    ref closestHit);
        }

        ProcessTraceEndVoxelForClosestHit(end, start, direction, ref found, ref closestDistance, ref closestHit);
    }

    private void TraceLineForAllHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        foreach (GridVoxelSet covered in GridTracer.TraceLine(_context.World, start, end))
        {
            foreach (Voxel voxel in covered.Voxels)
                ProcessTraceVoxelForAllHits(voxel, start, direction, results);
        }

        ProcessTraceEndVoxelForAllHits(end, start, direction, results);
    }

    private void TraceSweepForClosestHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        PrepareSweepBounds(start, end, radius, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessSweepVoxelForClosestHit(
                _coveredVoxels[i],
                start,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
    }

    private void TraceSweepForAllHits(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        PrepareSweepBounds(start, end, radius, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessSweepVoxelForAllHits(_coveredVoxels[i], start, direction, results);
    }

    private void PrepareSweepBounds(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        out Vector3d coverageMin,
        out Vector3d coverageMax)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        coverageMin = Vector3d.Min(start, end) - radiusExtents;
        coverageMax = Vector3d.Max(start, end) + radiusExtents;
    }

    private void ProcessTraceEndVoxelForClosestHit(
        Vector3d end,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (_context.World.TryGetVoxel(end, out Voxel? voxel))
            ProcessTraceVoxelForClosestHit(voxel!, origin, direction, ref found, ref closestDistance, ref closestHit);
    }

    private void ProcessTraceEndVoxelForAllHits(
        Vector3d end,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (_context.World.TryGetVoxel(end, out Voxel? voxel))
            ProcessTraceVoxelForAllHits(voxel!, origin, direction, results);
    }

    private void ProcessTraceVoxelForClosestHit(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForClosestHit(
            partition!,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessTraceVoxelForAllHits(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForAllHits(partition!, origin, direction, results);
    }

    private void ProcessSweepVoxelForClosestHit(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForClosestSweepHit(
            partition!,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessSweepVoxelForAllHits(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForAllSweepHits(partition!, origin, direction, results);
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedKinematicObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedStaticObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit)
                || hit.Distance >= closestDistance)
            {
                continue;
            }

            found = true;
            closestDistance = hit.Distance;
            closestHit = hit;
        }
    }

    private void ProcessPartitionForAllHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        ProcessColliderListForAllHits(partition.ContainedDynamicObjects, origin, direction, results);
        ProcessColliderListForAllHits(partition.ContainedKinematicObjects, origin, direction, results);
        ProcessColliderListForAllHits(partition.ContainedStaticObjects, origin, direction, results);
    }

    private void ProcessColliderListForAllHits(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private void ProcessPartitionForClosestSweepHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!_currentStaticSweepTargetsOnly)
        {
            ProcessColliderListForClosestSweepHit(
                partition.ContainedDynamicObjects,
                origin,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
        }

        ProcessColliderListForClosestSweepHit(
            partition.ContainedKinematicObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestSweepHit(
            partition.ContainedStaticObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessColliderListForClosestSweepHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit)
                || !ShouldReplaceClosestSweepHit(hit, found, closestDistance, closestHit))
            {
                continue;
            }

            found = true;
            closestDistance = hit.Distance;
            closestHit = hit;
        }
    }

    private static bool ShouldReplaceClosestSweepHit(
        Physics3DHit hit,
        bool found,
        Fixed64 closestDistance,
        Physics3DHit closestHit)
    {
        if (!found)
            return true;

        int distanceCompare = hit.Distance.CompareTo(closestDistance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        int hitId = hit.Collider?.Id ?? -1;
        int closestId = closestHit.Collider?.Id ?? -1;
        return hitId < closestId;
    }

    private void ProcessPartitionForAllSweepHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (!_currentStaticSweepTargetsOnly)
            ProcessColliderListForAllSweepHits(partition.ContainedDynamicObjects, origin, direction, results);

        ProcessColliderListForAllSweepHits(partition.ContainedKinematicObjects, origin, direction, results);
        ProcessColliderListForAllSweepHits(partition.ContainedStaticObjects, origin, direction, results);
    }

    private void ProcessColliderListForAllSweepHits(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private bool TryBuildHitForCollider(
        int colliderId,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit hit)
    {
        hit = default;
        return _context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
            && DoesCurrentColliderIntersectRay(current)
            && TryBuildHit(current!, origin, direction, out hit);
    }

    private bool TryBuildSweepHitForCollider(
        int colliderId,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit hit)
    {
        hit = default;
        return _context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
            && IsSweepCandidate(current)
            && TryBuildSweepHit(current!, origin, direction, out hit);
    }

    private bool TryBuildHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit raycastHit)
    {
        Fixed64 closestDistance = Fixed64.MaxValue;
        Vector3d closestIntersection = Vector3d.Zero;

        for (int i = _bufferIntersectionPoints.Count - 1; i >= 0; i--)
        {
            Fixed64 dist = Vector3d.Distance(_bufferIntersectionPoints[i], origin);
            if (dist < closestDistance)
            {
                closestDistance = dist;
                closestIntersection = _bufferIntersectionPoints[i];
            }
        }

        if (closestDistance == Fixed64.MaxValue)
        {
            raycastHit = default;
            return false;
        }

        Vector3d normal = collider.GetNormalAtPoint(closestIntersection);
        raycastHit = new Physics3DHit(collider, closestIntersection, normal, closestDistance, direction);
        return true;
    }

    private bool TryBuildSweepHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit sweepHit)
    {
        sweepHit = default;
        if (!_sweepWorker.TrySweep(collider, out Vector3d sweepCenter, out Fixed64 distance))
            return false;

        Vector3d point = GetSweepSurfacePoint(collider, sweepCenter, direction);
        Vector3d normal = ResolveSweepNormal(collider, point, sweepCenter, direction);
        sweepHit = new Physics3DHit(collider, point, normal, distance, direction);
        return true;
    }

    private bool DoesCurrentColliderIntersectRay(LSCollider? current)
    {
        if (current == null)
            return false;

        if (!_currentLayerMask.Includes(current.Layer)
            || current.RaycastVersion == RaycastVersion
            || !_redundantColliderCheck.Add(current.Id))
        {
            return false;
        }

        current.RaycastVersion = RaycastVersion;
        _bufferIntersectionPoints.FastClear();
        return current.ColliderOverlapsRay(_worker, ref _bufferIntersectionPoints);
    }

    private bool IsSweepCandidate(LSCollider? current)
    {
        if (current == null)
            return false;

        if (ReferenceEquals(current, _currentExcludedCollider)
            || !_currentLayerMask.Includes(current.Layer)
            || current.RaycastVersion == RaycastVersion
            || !_redundantColliderCheck.Add(current.Id))
        {
            return false;
        }

        if (!_currentIncludeTriggers && current.IsTrigger)
            return false;

        if (_currentStaticSweepTargetsOnly && !IsStaticStyleSweepTarget(current))
            return false;

        current.RaycastVersion = RaycastVersion;
        LastQueryCandidateCount++;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsStaticStyleSweepTarget(LSCollider collider)
    {
        StiffBody? body = collider.Body;
        return body == null || body.Immovable || body.IsKinematic;
    }

    private static Vector3d GetSweepSurfacePoint(LSCollider collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return collider.Center - direction * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(sweepCenter);
    }

    private static Vector3d ResolveSweepNormal(
        LSCollider collider,
        Vector3d point,
        Vector3d sweepCenter,
        Vector3d direction)
    {
        Vector3d fromPointToSweepCenter = sweepCenter - point;
        if ((collider is LSCuboidCollider || collider is LSCylinderCollider)
            && fromPointToSweepCenter.MagnitudeSquared > Fixed64.Epsilon)
        {
            return fromPointToSweepCenter.Normalized;
        }

        Vector3d normal = collider.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normal.Normalized;
            if (collider is LSMeshCollider && Vector3d.Dot(normal, direction) > Fixed64.Zero)
                return -normal;

            return normal;
        }

        if (fromPointToSweepCenter.MagnitudeSquared > Fixed64.Epsilon)
            return -fromPointToSweepCenter.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Zero;
    }

}
