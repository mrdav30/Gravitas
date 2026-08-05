//=======================================================================
// GravitasQuery3DService.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
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
using SwiftCollections.Query;

namespace Gravitas.Queries;

/// <content>
/// Owns apex-origin 3D cone-volume overlap query behavior.
/// </content>
public sealed partial class GravitasQuery3DService
{
    /// <summary>
    /// Finds the closest collider whose surface intersects an apex-origin
    /// directional cone volume.
    /// </summary>
    public bool OverlapCone(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit,
        PhysicsLayerMask layerMask)
    {
        Vector3d normalizedDirection = ValidateConeQuery(origin, direction, length, endRadius, out Vector3d end);
        _currentLayerMask = layerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        bool found = false;
        Physics3DHit closestHit = default;
        Fixed64 closestDistance = Fixed64.MaxValue;
        TraceConeForClosestHit(
            origin,
            end,
            normalizedDirection,
            length,
            endRadius,
            ref found,
            ref closestHit,
            ref closestDistance);

        hit = closestHit;
        _context.Diagnostics.EmitRayQuery(
            origin,
            end,
            endRadius,
            layerMask.Bits,
            found,
            found ? 1 : 0,
            closestHit);
        return found;
    }

    /// <summary>
    /// Writes all colliders whose surfaces intersect an apex-origin
    /// directional cone volume into caller-owned storage.
    /// </summary>
    public int OverlapConeAll(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        Vector3d normalizedDirection = ValidateConeQuery(origin, direction, length, endRadius, out Vector3d end);
        _currentLayerMask = layerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        results.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        TraceConeForAllHits(origin, end, normalizedDirection, length, endRadius, results);
        Physics3DHitSorter.SortByDistance(results);

        _context.Diagnostics.EmitRayQuery(
            origin,
            end,
            endRadius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    private void TraceConeForClosestHit(
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDistance)
    {
        GetConeQueryBounds(origin, baseCenter, direction, endRadius, out Vector3d min, out Vector3d max);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
                continue;

            ProcessConePartitionForClosestHit(
                partition!,
                origin,
                baseCenter,
                direction,
                length,
                endRadius,
                ref found,
                ref closestHit,
                ref closestDistance);
        }
    }

    private void TraceConeForAllHits(
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        SwiftList<Physics3DHit> results)
    {
        GetConeQueryBounds(origin, baseCenter, direction, endRadius, out Vector3d min, out Vector3d max);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
                continue;

            ProcessConePartitionForAllHits(partition!, origin, baseCenter, direction, length, endRadius, results);
        }
    }

    private void ProcessConePartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDistance)
    {
        ProcessConeColliderListForClosestHit(partition.ContainedDynamicObjects, origin, baseCenter, direction, length, endRadius, ref found, ref closestHit, ref closestDistance);
        ProcessConeColliderListForClosestHit(partition.ContainedKinematicObjects, origin, baseCenter, direction, length, endRadius, ref found, ref closestHit, ref closestDistance);
        ProcessConeColliderListForClosestHit(partition.ContainedStaticObjects, origin, baseCenter, direction, length, endRadius, ref found, ref closestHit, ref closestDistance);
    }

    private void ProcessConePartitionForAllHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        SwiftList<Physics3DHit> results)
    {
        ProcessConeColliderListForAllHits(partition.ContainedDynamicObjects, origin, baseCenter, direction, length, endRadius, results);
        ProcessConeColliderListForAllHits(partition.ContainedKinematicObjects, origin, baseCenter, direction, length, endRadius, results);
        ProcessConeColliderListForAllHits(partition.ContainedStaticObjects, origin, baseCenter, direction, length, endRadius, results);
    }

    private void ProcessConeColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDistance)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapConeHit(colliderIds.DenseKeys[i], origin, baseCenter, direction, length, endRadius, out Physics3DHit hit)
                || !PhysicsHitSelectionPolicy.ShouldReplace(hit, found, closestHit))
            {
                continue;
            }

            found = true;
            closestHit = hit;
            closestDistance = hit.Distance;
        }
    }

    private void ProcessConeColliderListForAllHits(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildOverlapConeHit(colliderIds.DenseKeys[i], origin, baseCenter, direction, length, endRadius, out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private bool TryBuildOverlapConeHit(
        int colliderId,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        hit = default;
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
        return TryBuildConeHitForCollider(collider, origin, baseCenter, direction, length, endRadius, out hit);
    }

    private bool TryBuildConeHitForCollider(
        LSCollider collider,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        if (collider is LSCompoundCollider compound)
            return TryBuildConeHitForCompound(compound, origin, baseCenter, direction, length, endRadius, out hit);

        if (collider is LSMeshCollider { Mode: MeshColliderMode.Concave } concaveMesh)
            return TryBuildConeHitForConcaveMesh(concaveMesh, origin, baseCenter, direction, length, endRadius, out hit);

        bool supportedConvexTarget = ConvexColliderSupport.IsSupported(collider);
        if (supportedConvexTarget
            && !ConvexColliderSupport.IntersectsConeVolume(collider, origin, baseCenter, direction, endRadius))
        {
            hit = default;
            return false;
        }

        Vector3d axisPoint = GetClosestPointOnConeAxis(origin, direction, length, collider.Center);
        Vector3d point = GetClosestSurfacePoint(collider, axisPoint);
        if (!IsPointInsideConeVolume(origin, direction, length, endRadius, point, out Fixed64 axialDistance))
        {
            Vector3d supportTowardAxis = collider.Center == axisPoint
                ? GetClosestSurfacePoint(collider, origin)
                : GetClosestSurfacePoint(collider, axisPoint);
            if (!IsPointInsideConeVolume(origin, direction, length, endRadius, supportTowardAxis, out axialDistance))
            {
                if (!supportedConvexTarget)
                {
                    hit = default;
                    return false;
                }

                point = GetClosestSurfacePoint(collider, origin);
                axialDistance = FixedMath.Min(
                    Vector3d.ProjectNonNegativeDifferenceParameter(point, origin, direction),
                    length);
            }
            else
            {
                point = supportTowardAxis;
            }
        }

        Vector3d toPoint = point - origin;
        Vector3d normal = collider.GetNormalAtPoint(point);
        hit = new Physics3DHit(collider, point, normal, axialDistance, toPoint);
        return true;
    }

    private bool TryBuildConeHitForConcaveMesh(
        LSMeshCollider mesh,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        GetConeQueryBounds(origin, baseCenter, direction, endRadius, out Vector3d min, out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _meshTriangleCandidates);
        LastMeshTriangleCandidateCount += _meshTriangleCandidates.Count;

        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        int bestTriangleIndex = int.MaxValue;
        Physics3DHit best = default;
        var queryOriginAnchor = new FixedPointAnchor(
            origin,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        if (!queryOriginAnchor.TryGetLocalPointIn(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                out Vector3d localOrigin))
        {
            hit = default;
            return false;
        }
        Vector3d localDirection =
            mesh.Mesh.Rotation.Inverse().Rotate(direction);

        for (int i = 0; i < _meshTriangleCandidates.Count; i++)
        {
            int triangleIndex = _meshTriangleCandidates[i];
            mesh.Mesh.GetLocalTriangleVertices(
                triangleIndex,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            Vector3d normal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            if (!new FixedTriangle(first, second, third).TryGetFiniteConeIntersectionMinimumAxialPoint(
                    localOrigin,
                    localDirection,
                    length,
                    endRadius,
                    out Vector3d localPoint))
            {
                continue;
            }

            FixedPointAnchor pointAnchor =
                mesh.Mesh.CreatePointAnchor(localPoint);
            Fixed64 axialDistance = FixedMath.Min(
                pointAnchor.ProjectNonNegativeOffsetFrom(
                    queryOriginAnchor,
                    direction),
                length);

            if (found
                && (axialDistance > bestDistance
                    || (axialDistance == bestDistance && triangleIndex >= bestTriangleIndex)))
            {
                continue;
            }

            _ = pointAnchor.TryGetOffsetFrom(
                queryOriginAnchor,
                out Vector3d toPoint);
            best = new Physics3DHit(
                mesh,
                new ContactAnchor(pointAnchor),
                normal,
                axialDistance,
                toPoint);
            bestDistance = axialDistance;
            bestTriangleIndex = triangleIndex;
            found = true;
        }

        hit = best;
        return found;
    }

    private bool TryBuildConeHitForCompound(
        LSCompoundCollider compound,
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        bool found = false;
        Physics3DHit best = default;
        for (int i = 0; i < compound.PartCount; i++)
        {
            if (!TryBuildConeHitForCollider(compound.GetPartCollider(i), origin, baseCenter, direction, length, endRadius, out Physics3DHit partHit)
                || !PhysicsHitSelectionPolicy.ShouldReplace(partHit, found, best))
            {
                continue;
            }

            best = new Physics3DHit(compound, partHit.Anchor, partHit.Normal, partHit.Distance, partHit.Direction);
            found = true;
        }

        hit = found ? best : default;
        return found;
    }

    private static bool IsPointInsideConeVolume(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d point,
        out Fixed64 axialDistance)
    {
        if (!FixedSegment.ContainsPointInFiniteCone(
                point,
                origin,
                direction,
                length,
                endRadius))
        {
            axialDistance = default;
            return false;
        }

        axialDistance = Vector3d.ProjectNonNegativeDifferenceParameter(point, origin, direction);
        return true;
    }

    private static Vector3d GetClosestPointOnConeAxis(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Vector3d point)
    {
        Fixed64 axial = FixedMath.Min(
            Vector3d.ProjectNonNegativeDifferenceParameter(point, origin, direction),
            length);
        return new FixedRay(origin, direction).GetPoint(axial);
    }

    private static void GetConeQueryBounds(
        Vector3d origin,
        Vector3d baseCenter,
        Vector3d direction,
        Fixed64 endRadius,
        out Vector3d min,
        out Vector3d max)
    {
        ConeGeometry.CreateFiniteConeBounds(origin, baseCenter, direction, endRadius, out min, out max);
    }

    private static Vector3d ValidateConeQuery(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Vector3d end)
    {
        SwiftThrowHelper.ThrowIfArgument(
            direction == Vector3d.Zero,
            nameof(direction),
            "Cone query direction must be non-zero.");
        SwiftThrowHelper.ThrowIfArgument(
            length <= Fixed64.Zero,
            nameof(length),
            "Cone query length must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(
            endRadius <= Fixed64.Zero,
            nameof(endRadius),
            "Cone query end radius must be greater than zero.");

        Vector3d normalizedDirection = direction.Normalized;
        SwiftThrowHelper.ThrowIfArgument(
            !new FixedRay(origin, normalizedDirection).TryGetPoint(length, out end),
            nameof(length),
            "Cone query endpoint must be representable without fixed-point saturation.");
        return normalizedDirection;
    }
}
