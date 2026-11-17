//=======================================================================
// GravitasQuery2DService.BoundedSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Grids;
using SwiftCollections;

namespace Gravitas.Queries;

/// <content>Queries planar support through bounded world and candidate traversal.</content>
public sealed partial class GravitasQuery2DService
{
    /// <summary>Finds the nearest eligible support within caller-owned work and scratch limits.</summary>
    /// <remarks>
    /// Requires pure 2D and a stable between-step host phase.
    /// Completed with null support is a proven miss. Other outcomes clear support.
    /// Serialize this call with world and collider mutation.
    /// </remarks>
    public PhysicsQueryExecutionStatus QuerySupport(
        in Physics2DSupportQuery request, in PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
        out Physics2DSupportHit? support, out PhysicsQueryWork work)
    {
        support = null;
        work = default;
        ChronicleGuard.ThrowIfNull(scratch, nameof(scratch));
        ValidateSupportSource(request);
        var meter = new PhysicsQueryMeter(budget);
        int voxelCount = 0;
        scratch.Borrow();
        try
        {
            if (_context.Settings.RuntimeMode != PhysicsRuntimeMode.TwoD)
                return PhysicsQueryExecutionStatus.UnsupportedRuntimeMode;
            if (!_context.BoundedQueryPartitionsReady)
                return PhysicsQueryExecutionStatus.WorldNotReady;
            if (!TryGetSupportBounds(request, out var end, out var min, out var max))
                return PhysicsQueryExecutionStatus.Unrepresentable;

            var status = scratch.Discover(_context, new Vector3d(min.X, Fixed64.Zero, min.Y),
                new Vector3d(max.X, Fixed64.Zero, max.Y), planar: true, ref meter, out voxelCount, out ulong sequence);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            Physics2DHit? closest = null;
            for (int i = 0; i < voxelCount; i++)
            {
                var partitionStatus = scratch.Voxels[i]!.TryGetPartitionBounded<PhysicsPartition2D>(
                    meter.DiscoveryRemaining, out ulong partitionWork, out var partition);
                meter.AddDiscovery(partitionWork);
                if (partitionStatus != GridBoundedVoxelStatus.Completed)
                    return PhysicsQueryScratch.MapDiscoveryFailure(partitionStatus);
                if (partition == null)
                    continue;

                status = VisitBoundedSupportPartition(partition, request, end, min, max, scratch, ref meter, ref closest);
                if (status != PhysicsQueryExecutionStatus.Completed)
                    return status;
            }

            status = PhysicsQueryScratch.CheckView(_context, sequence, ref meter);
            if (status == PhysicsQueryExecutionStatus.Completed && closest.HasValue)
                support = new Physics2DSupportHit(closest.Value, _context.FrameCount);
            return status;
        }
        finally
        {
            scratch.Release(voxelCount);
            work = meter.Work;
        }
    }

    private PhysicsQueryExecutionStatus VisitBoundedSupportPartition(
        PhysicsPartition2D partition, in Physics2DSupportQuery request, Vector2d end, Vector2d min, Vector2d max,
        PhysicsQueryScratch scratch, ref PhysicsQueryMeter meter, ref Physics2DHit? closest)
    {
        var status = VisitBoundedSupportMembership(partition.ContainedDynamicObjects, request, end, min, max,
            scratch, ref meter, ref closest);
        if (status == PhysicsQueryExecutionStatus.Completed)
            status = VisitBoundedSupportMembership(partition.ContainedKinematicObjects, request, end, min, max,
                scratch, ref meter, ref closest);
        if (status == PhysicsQueryExecutionStatus.Completed)
            status = VisitBoundedSupportMembership(partition.ContainedStaticObjects, request, end, min, max,
                scratch, ref meter, ref closest);
        return status;
    }

    private PhysicsQueryExecutionStatus VisitBoundedSupportMembership(
        SwiftSortedList<int>? membership, in Physics2DSupportQuery request, Vector2d end, Vector2d min, Vector2d max,
        PhysicsQueryScratch scratch, ref PhysicsQueryMeter meter, ref Physics2DHit? closest)
    {
        if (membership == null || membership.Count == 0)
            return PhysicsQueryExecutionStatus.Completed;
        if (!meter.ChargeCandidates((ulong)membership.Count))
            return PhysicsQueryExecutionStatus.WorkBudgetExceeded;

        foreach (int id in membership.AsReadOnlySpan())
        {
            if (!_context.Physics2D.TryGetColliderById(id, out var collider) || !collider!.IsActive
                || !request.Layers.Includes(collider.Layer) || !IsSupportCandidate(request, collider)
                || collider.MaxX < min.X || collider.MinX > max.X
                || collider.MaxY < min.Y || collider.MinY > max.Y)
                continue;

            var status = scratch.RememberCandidate(id, ref meter, out bool isNew);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (!isNew)
                continue;

            status = VisitBoundedSupportCandidate(collider, request, end, ref meter, ref closest);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
        }
        return PhysicsQueryExecutionStatus.Completed;
    }

    private PhysicsQueryExecutionStatus VisitBoundedSupportCandidate(
        LSCollider2D collider, in Physics2DSupportQuery request, Vector2d end,
        ref PhysicsQueryMeter meter, ref Physics2DHit? closest)
    {
        if (!meter.ChargeCandidates(_context.Constraints2D.LinkedCollisionProbeLimit))
            return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
        if (!_context.Physics2D.RequireCollisionPair(request.Source, collider))
            return PhysicsQueryExecutionStatus.Completed;
        var status = QueryDetection2D.ReserveSupportGeometry(collider, ref meter);
        if (status != PhysicsQueryExecutionStatus.Completed)
            return status;
        if (QueryDetection2D.TryFindSupport(request, end, collider, out var hit)
            && PhysicsHitSelectionPolicy.ShouldReplace(hit, closest.HasValue, closest.GetValueOrDefault()))
            closest = hit;
        return PhysicsQueryExecutionStatus.Completed;
    }
}
