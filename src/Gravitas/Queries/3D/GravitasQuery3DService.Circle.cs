//=======================================================================
// GravitasQuery3DService.Circle.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
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
        bool found = false;

        TraceCircleForClosestHit(position, radius, ref found, ref closestHit);

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
    /// Finds the closest circle-overlap hit whose projected surface offset lies in the supplied X/Z
    /// direction and within the supplied distance.
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

        var planarDirection = new Vector2d(direction.X, direction.Z);
        Vector2d normalizedPlanarDirection = Vector2d.GetNormalized(planarDirection);
        var normalizedDirection = new Vector3d(
            normalizedPlanarDirection.X,
            Fixed64.Zero,
            normalizedPlanarDirection.Y);
        Physics3DHit closestHit = default;
        bool found = false;

        TraceCircleForDirectionalHit(
            position,
            radius,
            planarDirection,
            maxDistance,
            ref found,
            ref closestHit);

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

        return TryBuildSurfaceOverlapHit(
            collider,
            position,
            radius,
            out hit);
    }

    private void TraceCircleForClosestHit(
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        GetCoveredCircleVoxels(position, radius);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForClosestHit(partition!, position, radius, ref found, ref closestHit);
        }
    }

    private void TraceCircleForDirectionalHit(
        Vector3d position,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maxDistance,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (direction == Vector2d.Zero)
            return;

        GetCoveredCircleVoxels(position, radius);
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
                maxDistance,
                ref found,
                ref closestHit);
        }
    }

    private void TraceCircleForAllHits(Vector3d position, Fixed64 radius, SwiftList<Physics3DHit> results)
    {
        GetCoveredCircleVoxels(position, radius);
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

    private void GetCoveredCircleVoxels(Vector3d position, Fixed64 radius)
    {
        var min = new Vector3d(
            position.X - radius,
            Fixed64.MinValue,
            position.Z - radius);
        var max = new Vector3d(
            position.X + radius,
            Fixed64.MaxValue,
            position.Z + radius);
        GridTracer.GetCoveredVoxelsInto(
            _context.World,
            min,
            max,
            _coveredVoxels,
            _traceScratch);
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
            position,
            radius,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedKinematicObjects,
            position,
            radius,
            ref found,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedStaticObjects,
            position,
            radius,
            ref found,
            ref closestHit);
    }

    private void ProcessColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out Physics3DHit hitInfo)
                || !PhysicsHitSelectionPolicy.ShouldReplace(hitInfo, found, closestHit))
            {
                continue;
            }

            found = true;
            closestHit = hitInfo;
        }
    }

    private void ProcessPartitionForDirectionalHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maxDistance,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForDirectionalHit(partition.ContainedDynamicObjects, position, radius, direction, maxDistance, ref found, ref closestHit);
        ProcessColliderListForDirectionalHit(partition.ContainedKinematicObjects, position, radius, direction, maxDistance, ref found, ref closestHit);
        ProcessColliderListForDirectionalHit(partition.ContainedStaticObjects, position, radius, direction, maxDistance, ref found, ref closestHit);
    }

    private void ProcessColliderListForDirectionalHit(
        SwiftSparseSet? colliderIds,
        Vector3d position,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maxDistance,
        ref bool found,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out Physics3DHit hitInfo))
                continue;

            if (hitInfo.Distance > maxDistance
                || WideGeometry.GetDifferenceDotProduct2D(
                    hitInfo.Direction.X,
                    Fixed64.Zero,
                    hitInfo.Direction.Z,
                    Fixed64.Zero,
                    direction.X,
                    Fixed64.Zero,
                    direction.Y,
                    Fixed64.Zero).Sign <= 0
                || !PhysicsHitSelectionPolicy.ShouldReplace(hitInfo, found, closestHit))
            {
                continue;
            }

            found = true;
            closestHit = hitInfo;
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

        return TryBuildPlanarOverlapHit(
            collider,
            position,
            radius,
            out raycastHit);
    }

    private static bool TryBuildPlanarOverlapHit(
        LSCollider collider,
        Vector3d position,
        Fixed64 radius,
        out Physics3DHit hit)
    {
        if (!ColliderPlanarProjection.TryGetRelation(
                collider,
                new Vector2d(position.X, position.Z),
                radius,
                out ProjectedSurfaceRelation relation))
        {
            hit = default;
            return false;
        }

        hit = new Physics3DHit(
            collider,
            new ContactAnchor(relation.ContactAnchor),
            relation.OutwardNormal,
            relation.Distance,
            new Vector3d(
                relation.Offset.X,
                Fixed64.Zero,
                relation.Offset.Y));
        return true;
    }

    internal static bool TryBuildSurfaceOverlapHit(
        LSCollider collider,
        Vector3d position,
        Fixed64 radius,
        out Physics3DHit hit)
    {
        ContactAnchor anchor;
        Vector3d toPoint;
        Vector3d normal;
        if (collider is LSCuboidCollider cuboid)
        {
            anchor = new ContactAnchor(
                cuboid.OrientedBox.GetClosestPointAnchor(position));
            if (!anchor.TryGetOffsetFrom(position, out toPoint))
            {
                hit = default;
                return false;
            }
            normal = cuboid.OrientedBox.GetNearestFaceNormal(position);
        }
        else
        {
            Vector3d point = GetClosestSurfacePoint(collider, position);
            if (!Vector3d.TrySubtract(point, position, out toPoint))
            {
                hit = default;
                return false;
            }
            anchor = ContactAnchor.FromWorldPoint(point);
            normal = collider.GetNormalAtPoint(point);
        }

        if (!Vector3d.TryGetMagnitude(toPoint, out Fixed64 distance)
            || distance > radius)
        {
            hit = default;
            return false;
        }

        hit = new Physics3DHit(
            collider,
            anchor,
            normal,
            distance,
            toPoint);
        return true;
    }

    private static Vector3d GetClosestSurfacePoint(LSCollider collider, Vector3d position)
    {
        if (position == collider.Center && collider is not LSMeshCollider)
            return collider.Center + Vector3d.Right * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(position);
    }
}
