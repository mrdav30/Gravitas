//=======================================================================
// GravitasQuery3DService.Circle.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Utility;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>
/// Owns 3D X/Z circle overlap query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery3DService
{
    /// <summary>
    /// Finds the closest collider whose surface is inside the supplied X/Z circle radius.
    /// </summary>
    public bool OverlapCircle(
        Vector3d position,
        Fixed64 radius,
        out Physics3DHit raycastHit,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        Physics3DHit closestHit = default;
        Fixed64 closestDist = Fixed64.MaxValue;
        bool found = false;

        TraceCircleForClosestHit(position, radius, ref found, ref closestHit, ref closestDist);

        raycastHit = closestHit;
        _context.Diagnostics.EmitCircleQuery(
            position,
            radius,
            Vector3d.Zero,
            Fixed64.Zero,
            layerMask.Bits,
            found,
            found ? 1 : 0,
            closestHit);

        return found;
    }

    /// <summary>
    /// Finds the closest circle-overlap hit whose hit point lies in the supplied direction and distance.
    /// </summary>
    public bool OverlapCircleInDirection(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        out Physics3DHit raycastHit,
        Fixed64 maxDistance,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        Vector3d normalizedDirection = direction.MagnitudeSquared == Fixed64.Zero ? Vector3d.Zero : direction.Normalized;
        Fixed64 maxDistanceSqr = maxDistance * maxDistance;
        Physics3DHit closestHit = default;
        Fixed64 closestDist = Fixed64.MaxValue;
        bool found = false;

        TraceCircleForDirectionalHit(
            position,
            radius,
            normalizedDirection,
            maxDistanceSqr,
            ref found,
            ref closestHit,
            ref closestDist);

        raycastHit = closestHit;
        _context.Diagnostics.EmitCircleQuery(
            position,
            radius,
            normalizedDirection,
            maxDistance,
            layerMask.Bits,
            found,
            found ? 1 : 0,
            closestHit);

        return found;
    }

    /// <summary>
    /// Finds all colliders whose surfaces are inside the supplied X/Z circle radius.
    /// </summary>
    public int OverlapCircleAll(
        Vector3d position,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        _currentLayerMask = layerMask;
        NextCircleVersion();
        ResetLastQueryCounters();

        results.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        TraceCircleForAllHits(position, radius, results);

        Physics3DHitSorter.SortByDistance(results);
        Physics3DHit firstHit = results.Count > 0 ? results[0] : default;
        _context.Diagnostics.EmitCircleQuery(
            position,
            radius,
            Vector3d.Zero,
            Fixed64.Zero,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            firstHit);

        return results.Count;
    }

    internal int OverlapSphereAgainstStaticAll(
        Vector3d position,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        results.FastClear();
        if (radius <= Fixed64.Zero)
        {
            ResetLastQueryCounters();
            return 0;
        }

        _currentLayerMask = layerMask;
        _currentExcludedCollider = excludedCollider;
        _currentIncludeTriggers = includeTriggers;
        _currentStaticSweepTargetsOnly = true;
        NextCircleVersion();
        ResetLastQueryCounters();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        Vector3d min = new(position.X - radius, position.Y - radius, position.Z - radius);
        Vector3d max = new(position.X + radius, position.Y + radius, position.Z + radius);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
                continue;

            ProcessPartitionForAllSphereHits(partition!, position, radius, results);
        }

        Physics3DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private void ProcessPartitionForAllSphereHits(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        SwiftList<Physics3DHit> results)
    {
        if (!_currentStaticSweepTargetsOnly)
            ProcessColliderListForAllSphereHits(partition.ContainedDynamicObjects, position, radius, results);

        ProcessColliderListForAllSphereHits(partition.ContainedKinematicObjects, position, radius, results);
        ProcessColliderListForAllSphereHits(partition.ContainedStaticObjects, position, radius, results);
    }

    private void ProcessColliderListForAllSphereHits(
        SwiftSparseSet? colliderIds,
        Vector3d position,
        Fixed64 radius,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
            if (TryBuildOverlapSphereHit(colliderIds.DenseKeys[i], position, radius, out Physics3DHit hitInfo))
                results.Add(hitInfo);
    }

    private bool TryBuildOverlapSphereHit(int colliderId, Vector3d position, Fixed64 radius, out Physics3DHit hit)
    {
        hit = default;
        if (!_context.Physics.TryGetColliderById(colliderId, out LSCollider? current))
            return false;

        LSCollider collider = current!;
        if (ReferenceEquals(collider, _currentExcludedCollider)
            || (_currentExcludedCollider != null
                && _context.Constraints3D.ShouldExcludeLinkedCollision(_currentExcludedCollider, collider))
            || !_currentLayerMask.Includes(collider.Layer)
            || collider.CircleQueryVersion == CircleVersion
            || !_redundantColliderCheck.Add(collider.Id)
            || (!_currentIncludeTriggers && collider.IsTrigger)
            || (_currentStaticSweepTargetsOnly && !IsStaticStyleSweepTarget(collider)))
        {
            return false;
        }

        collider.CircleQueryVersion = CircleVersion;
        LastQueryCandidateCount++;

        Fixed64 broadDistance = collider.ScaledRadius + radius;
        if ((collider.Center - position).MagnitudeSquared > broadDistance * broadDistance)
            return false;

        Vector3d point = GetClosestSurfacePoint(collider, position);
        Vector3d toPoint = point - position;
        Fixed64 distance = toPoint.Magnitude;
        if (distance > radius)
            return false;

        Vector3d normal = collider.GetNormalAtPoint(point);
        hit = new Physics3DHit(collider, point, normal, distance, toPoint);
        return true;
    }

    private void TraceCircleForClosestHit(
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDist)
    {
        Vector2d min = new(position.X - radius, position.Z - radius);
        Vector2d max = new(position.X + radius, position.Z + radius);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch, layerY: position.Y);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForClosestHit(partition!, position, radius, ref found, ref closestHit, ref closestDist);
        }
    }

    private void TraceCircleForDirectionalHit(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistanceSqr,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDist)
    {
        Vector2d min = new(position.X - radius, position.Z - radius);
        Vector2d max = new(position.X + radius, position.Z + radius);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch, layerY: position.Y);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForDirectionalHit(
                partition!,
                position,
                radius,
                direction,
                maxDistanceSqr,
                ref found,
                ref closestHit,
                ref closestDist);
        }
    }

    private void TraceCircleForAllHits(Vector3d position, Fixed64 radius, SwiftList<Physics3DHit> results)
    {
        Vector2d min = new(position.X - radius, position.Z - radius);
        Vector2d max = new(position.X + radius, position.Z + radius);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch, layerY: position.Y);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForAllHits(partition!, position, radius, results);
        }
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDist)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
            position,
            radius,
            ref found,
            ref closestHit,
            ref closestDist);

        ProcessColliderListForClosestHit(
            partition.ContainedKinematicObjects,
            position,
            radius,
            ref found,
            ref closestHit,
            ref closestDist);

        ProcessColliderListForClosestHit(
            partition.ContainedStaticObjects,
            position,
            radius,
            ref found,
            ref closestHit,
            ref closestDist);
    }

    private void ProcessColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDist)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out Physics3DHit hitInfo)
                || (found && !Physics3DHitSorter.ComesBefore(hitInfo, closestHit)))
            {
                continue;
            }

            found = true;
            closestHit = hitInfo;
            closestDist = hitInfo.Distance;
        }
    }

    private void ProcessPartitionForDirectionalHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistanceSqr,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDist)
    {
        ProcessColliderListForDirectionalHit(partition.ContainedDynamicObjects, position, radius, direction, maxDistanceSqr, ref found, ref closestHit, ref closestDist);
        ProcessColliderListForDirectionalHit(partition.ContainedKinematicObjects, position, radius, direction, maxDistanceSqr, ref found, ref closestHit, ref closestDist);
        ProcessColliderListForDirectionalHit(partition.ContainedStaticObjects, position, radius, direction, maxDistanceSqr, ref found, ref closestHit, ref closestDist);
    }

    private void ProcessColliderListForDirectionalHit(
        SwiftSparseSet? colliderIds,
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistanceSqr,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDist)
    {
        if (colliderIds == null || direction.MagnitudeSquared == Fixed64.Zero)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out Physics3DHit hitInfo))
                continue;

            Vector3d toHit = hitInfo.Point - position;
            if (toHit.MagnitudeSquared > maxDistanceSqr
                || Vector3d.Dot(toHit.Normalized, direction) <= Fixed64.Zero
                || (found && !Physics3DHitSorter.ComesBefore(hitInfo, closestHit)))
            {
                continue;
            }

            found = true;
            closestHit = hitInfo;
            closestDist = hitInfo.Distance;
        }
    }

    private void ProcessPartitionForAllHits(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        SwiftList<Physics3DHit> results)
    {
        ProcessColliderListForAllHits(partition.ContainedDynamicObjects, position, radius, results);
        ProcessColliderListForAllHits(partition.ContainedKinematicObjects, position, radius, results);
        ProcessColliderListForAllHits(partition.ContainedStaticObjects, position, radius, results);
    }

    private void ProcessColliderListForAllHits(
        SwiftSparseSet? colliderIds,
        Vector3d position,
        Fixed64 radius,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out Physics3DHit hitInfo))
                results.Add(hitInfo);
        }
    }

    private bool TryBuildOverlapHit(int colliderId, Vector3d position, Fixed64 radius, out Physics3DHit raycastHit)
    {
        raycastHit = default;
        if (!_context.Physics.TryGetColliderById(colliderId, out LSCollider? current))
            return false;

        LSCollider collider = current!;
        if (!_currentLayerMask.Includes(collider.Layer)
            || collider.CircleQueryVersion == CircleVersion
            || !_redundantColliderCheck.Add(collider.Id))
        {
            return false;
        }

        collider.CircleQueryVersion = CircleVersion;
        LastQueryCandidateCount++;
        Fixed64 broadDistance = collider.ScaledRadius + radius;
        if ((collider.Center - position).MagnitudeSquared > broadDistance * broadDistance)
            return false;

        Vector3d point = GetClosestSurfacePoint(collider, position);
        Vector3d toPoint = point - position;
        Fixed64 distance = toPoint.Magnitude;
        if (distance > radius)
            return false;

        Vector3d normal = collider.GetNormalAtPoint(point);
        raycastHit = new Physics3DHit(collider, point, normal, distance, toPoint);
        return true;
    }

    private static Vector3d GetClosestSurfacePoint(LSCollider collider, Vector3d position)
    {
        if ((position - collider.Center).MagnitudeSquared == Fixed64.Zero)
            return collider.Center + Vector3d.Right * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(position);
    }
}
