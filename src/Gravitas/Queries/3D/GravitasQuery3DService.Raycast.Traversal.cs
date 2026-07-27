//=======================================================================
// GravitasQuery3DService.Raycast.Traversal.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;

namespace Gravitas.Queries;

public sealed partial class GravitasQuery3DService
{
    private void TraceLineForClosestHit(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        GridTracer.TraceLineInto(_context.World, start, end, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessTraceVoxelForClosestHit(
                _coveredVoxels[i],
                start,
                direction,
                ref found,
                ref closestHit);
    }

    private void TraceLineForAllHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        GridTracer.TraceLineInto(_context.World, start, end, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessTraceVoxelForAllHits(_coveredVoxels[i], start, direction, results);
    }

    private void TraceSweepForClosestHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        ref bool found,
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

    private void TraceConvexSweepForClosestHit(
        LSCollider source,
        Vector3d displacement,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        PrepareConvexSweepBounds(source, displacement, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessConvexSweepVoxelForClosestHit(
                _coveredVoxels[i],
                ref found,
                ref closestHit);
    }

    private void TraceConvexSweepForAllHits(
        LSCollider source,
        Vector3d displacement,
        SwiftList<Physics3DHit> results)
    {
        PrepareConvexSweepBounds(source, displacement, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessConvexSweepVoxelForAllHits(_coveredVoxels[i], results);
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

    private static void PrepareConvexSweepBounds(
        LSCollider source,
        Vector3d displacement,
        out Vector3d coverageMin,
        out Vector3d coverageMax)
    {
        coverageMin = Vector3d.Min(source.BoundsMin, source.BoundsMin + displacement);
        coverageMax = Vector3d.Max(source.BoundsMax, source.BoundsMax + displacement);
    }

    private void ProcessTraceVoxelForClosestHit(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
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

    private void ProcessConvexSweepVoxelForClosestHit(
        Voxel voxel,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForClosestConvexSweepHit(
            partition!,
            ref found,
            ref closestHit);
    }

    private void ProcessConvexSweepVoxelForAllHits(
        Voxel voxel,
        SwiftList<Physics3DHit> results)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForAllConvexSweepHits(partition!, results);
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
            origin,
            direction,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedKinematicObjects,
            origin,
            direction,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedStaticObjects,
            origin,
            direction,
            ref found,
            ref closestHit);
    }

    private void ProcessColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit)
                || !PhysicsHitSelectionPolicy.ShouldReplace(hit, found, closestHit))
            {
                continue;
            }

            found = true;
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
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestSweepHit(
            partition.ContainedDynamicObjects,
            origin,
            direction,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestSweepHit(
            partition.ContainedKinematicObjects,
            origin,
            direction,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestSweepHit(
            partition.ContainedStaticObjects,
            origin,
            direction,
            ref found,
            ref closestHit);
    }

    private void ProcessColliderListForClosestSweepHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit)
                || !PhysicsHitSelectionPolicy.ShouldReplace(hit, found, closestHit))
            {
                continue;
            }

            found = true;
            closestHit = hit;
        }
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

    private void ProcessPartitionForClosestConvexSweepHit(
        PhysicsPartition partition,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestConvexSweepHit(
            partition.ContainedDynamicObjects,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestConvexSweepHit(
            partition.ContainedKinematicObjects,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestConvexSweepHit(
            partition.ContainedStaticObjects,
            ref found,
            ref closestHit);
    }

    private void ProcessPartitionForAllConvexSweepHits(
        PhysicsPartition partition,
        SwiftList<Physics3DHit> results)
    {
        if (!_currentStaticSweepTargetsOnly)
            ProcessColliderListForAllConvexSweepHits(partition.ContainedDynamicObjects, results);

        ProcessColliderListForAllConvexSweepHits(partition.ContainedKinematicObjects, results);
        ProcessColliderListForAllConvexSweepHits(partition.ContainedStaticObjects, results);
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

    private void ProcessColliderListForClosestConvexSweepHit(
        SwiftSparseSet? colliderIds,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildConvexSweepHitForCollider(colliderIds.DenseKeys[i], out Physics3DHit hit)
                || !PhysicsHitSelectionPolicy.ShouldReplace(hit, found, closestHit))
            {
                continue;
            }

            found = true;
            closestHit = hit;
        }
    }

    private void ProcessColliderListForAllConvexSweepHits(
        SwiftSparseSet? colliderIds,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildConvexSweepHitForCollider(colliderIds.DenseKeys[i], out Physics3DHit hit))
                results.Add(hit);
        }
    }

}
