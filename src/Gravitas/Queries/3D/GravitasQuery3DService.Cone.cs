//=======================================================================
// GravitasQuery3DService.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Utility;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Queries;

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
        Vector3d normalizedDirection = ValidateConeQuery(origin, direction, length, endRadius);
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
            normalizedDirection,
            length,
            endRadius,
            ref found,
            ref closestHit,
            ref closestDistance);

        hit = closestHit;
        _context.Diagnostics.EmitRayQuery(
            origin,
            origin + normalizedDirection * length,
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
        Vector3d normalizedDirection = ValidateConeQuery(origin, direction, length, endRadius);
        _currentLayerMask = layerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        results.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        TraceConeForAllHits(origin, normalizedDirection, length, endRadius, results);
        Physics3DHitSorter.SortByDistance(results);

        _context.Diagnostics.EmitRayQuery(
            origin,
            origin + normalizedDirection * length,
            endRadius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    private void TraceConeForClosestHit(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDistance)
    {
        GetConeQueryBounds(origin, direction, length, endRadius, out Vector3d min, out Vector3d max);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
                continue;

            ProcessConePartitionForClosestHit(
                partition!,
                origin,
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
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        SwiftList<Physics3DHit> results)
    {
        GetConeQueryBounds(origin, direction, length, endRadius, out Vector3d min, out Vector3d max);
        GridTracer.GetCoveredVoxelsInto(_context.World, min, max, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
                continue;

            ProcessConePartitionForAllHits(partition!, origin, direction, length, endRadius, results);
        }
    }

    private void ProcessConePartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        ref bool found,
        ref Physics3DHit closestHit,
        ref Fixed64 closestDistance)
    {
        ProcessConeColliderListForClosestHit(partition.ContainedDynamicObjects, origin, direction, length, endRadius, ref found, ref closestHit, ref closestDistance);
        ProcessConeColliderListForClosestHit(partition.ContainedKinematicObjects, origin, direction, length, endRadius, ref found, ref closestHit, ref closestDistance);
        ProcessConeColliderListForClosestHit(partition.ContainedStaticObjects, origin, direction, length, endRadius, ref found, ref closestHit, ref closestDistance);
    }

    private void ProcessConePartitionForAllHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        SwiftList<Physics3DHit> results)
    {
        ProcessConeColliderListForAllHits(partition.ContainedDynamicObjects, origin, direction, length, endRadius, results);
        ProcessConeColliderListForAllHits(partition.ContainedKinematicObjects, origin, direction, length, endRadius, results);
        ProcessConeColliderListForAllHits(partition.ContainedStaticObjects, origin, direction, length, endRadius, results);
    }

    private void ProcessConeColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
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
            if (!TryBuildOverlapConeHit(colliderIds.DenseKeys[i], origin, direction, length, endRadius, out Physics3DHit hit)
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
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildOverlapConeHit(colliderIds.DenseKeys[i], origin, direction, length, endRadius, out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private bool TryBuildOverlapConeHit(
        int colliderId,
        Vector3d origin,
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
        return TryBuildConeHitForCollider(collider, origin, direction, length, endRadius, out hit);
    }

    private bool TryBuildConeHitForCollider(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        if (collider is LSCompoundCollider compound)
            return TryBuildConeHitForCompound(compound, origin, direction, length, endRadius, out hit);

        if (collider is LSMeshCollider { Mode: MeshColliderMode.Concave } concaveMesh)
            return TryBuildConeHitForConcaveMesh(concaveMesh, origin, direction, length, endRadius, out hit);

        bool supportedConvexTarget = ConvexColliderSupport.IsSupported(collider);
        if (supportedConvexTarget
            && !ConvexColliderSupport.IntersectsConeVolume(collider, origin, direction, length, endRadius))
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
                axialDistance = FixedMath.Clamp(
                    Vector3d.Dot(point - origin, direction),
                    Fixed64.Zero,
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
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        GetConeQueryBounds(origin, direction, length, endRadius, out Vector3d min, out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _meshTriangleCandidates);
        LastMeshTriangleCandidateCount += _meshTriangleCandidates.Count;

        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        int bestTriangleIndex = int.MaxValue;
        Physics3DHit best = default;

        for (int i = 0; i < _meshTriangleCandidates.Count; i++)
        {
            int triangleIndex = _meshTriangleCandidates[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d normal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            if (!TryBuildConeTriangleHit(
                    origin,
                    direction,
                    length,
                    endRadius,
                    first,
                    second,
                    third,
                    normal,
                    out Vector3d point,
                    out Fixed64 axialDistance))
            {
                continue;
            }

            if (found
                && (axialDistance > bestDistance
                    || (axialDistance == bestDistance && triangleIndex >= bestTriangleIndex)))
            {
                continue;
            }

            best = new Physics3DHit(mesh, point, normal, axialDistance, point - origin);
            bestDistance = axialDistance;
            bestTriangleIndex = triangleIndex;
            found = true;
        }

        hit = best;
        return found;
    }

    private static bool TryBuildConeTriangleHit(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d normal,
        out Vector3d point,
        out Fixed64 axialDistance)
    {
        bool found = false;
        Vector3d bestPoint = default;
        Fixed64 bestAxialDistance = Fixed64.MaxValue;

        TryKeepConeTrianglePoint(origin, direction, length, endRadius, first, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeTrianglePoint(origin, direction, length, endRadius, second, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeTrianglePoint(origin, direction, length, endRadius, third, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeTriangleSegment(origin, direction, length, endRadius, first, second, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeTriangleSegment(origin, direction, length, endRadius, second, third, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeTriangleSegment(origin, direction, length, endRadius, third, first, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeAxisTriangleIntersection(origin, direction, length, endRadius, first, second, third, normal, ref found, ref bestPoint, ref bestAxialDistance);

        point = bestPoint;
        axialDistance = bestAxialDistance;
        return found;
    }

    private static void TryKeepConeTriangleSegment(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d start,
        Vector3d end,
        ref bool found,
        ref Vector3d bestPoint,
        ref Fixed64 bestAxialDistance)
    {
        TryKeepConeTrianglePoint(origin, direction, length, endRadius, start, ref found, ref bestPoint, ref bestAxialDistance);
        TryKeepConeTrianglePoint(origin, direction, length, endRadius, end, ref found, ref bestPoint, ref bestAxialDistance);

        Vector3d deltaStart = start - origin;
        Vector3d segment = end - start;
        Fixed64 axialStart = Vector3d.Dot(deltaStart, direction);
        Fixed64 axialSlope = Vector3d.Dot(segment, direction);
        Fixed64 radiusSlope = endRadius / length;
        Fixed64 coneSlopeSqr = Fixed64.One + radiusSlope * radiusSlope;

        Fixed64 a = segment.MagnitudeSquared - coneSlopeSqr * axialSlope * axialSlope;
        Fixed64 b = (Fixed64)2 * (Vector3d.Dot(deltaStart, segment) - coneSlopeSqr * axialStart * axialSlope);
        Fixed64 c = deltaStart.MagnitudeSquared - coneSlopeSqr * axialStart * axialStart;

        if (a.Abs() <= Fixed64.Epsilon)
        {
            if (b.Abs() > Fixed64.Epsilon)
                TryKeepConeTriangleSegmentParameter(origin, direction, length, endRadius, start, segment, -c / b, ref found, ref bestPoint, ref bestAxialDistance);
        }
        else
        {
            Fixed64 discriminant = b * b - (Fixed64)4 * a * c;
            if (discriminant >= Fixed64.Zero)
            {
                Fixed64 root = FixedMath.Sqrt(discriminant);
                Fixed64 denominator = (Fixed64)2 * a;
                TryKeepConeTriangleSegmentParameter(origin, direction, length, endRadius, start, segment, (-b - root) / denominator, ref found, ref bestPoint, ref bestAxialDistance);
                TryKeepConeTriangleSegmentParameter(origin, direction, length, endRadius, start, segment, (-b + root) / denominator, ref found, ref bestPoint, ref bestAxialDistance);
            }
        }

        if (axialSlope.Abs() > Fixed64.Epsilon)
        {
            TryKeepConeTriangleSegmentParameter(origin, direction, length, endRadius, start, segment, -axialStart / axialSlope, ref found, ref bestPoint, ref bestAxialDistance);
            TryKeepConeTriangleSegmentParameter(origin, direction, length, endRadius, start, segment, (length - axialStart) / axialSlope, ref found, ref bestPoint, ref bestAxialDistance);
        }
    }

    private static void TryKeepConeTriangleSegmentParameter(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d start,
        Vector3d segment,
        Fixed64 parameter,
        ref bool found,
        ref Vector3d bestPoint,
        ref Fixed64 bestAxialDistance)
    {
        if (parameter < -Fixed64.Epsilon || parameter > Fixed64.One + Fixed64.Epsilon)
            return;

        parameter = FixedMath.Clamp01(parameter);
        TryKeepConeTrianglePoint(origin, direction, length, endRadius, start + segment * parameter, ref found, ref bestPoint, ref bestAxialDistance);
    }

    private static void TryKeepConeAxisTriangleIntersection(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d normal,
        ref bool found,
        ref Vector3d bestPoint,
        ref Fixed64 bestAxialDistance)
    {
        Fixed64 denominator = Vector3d.Dot(normal, direction);
        if (denominator.Abs() <= Fixed64.Epsilon)
            return;

        Fixed64 axialDistance = Vector3d.Dot(normal, first - origin) / denominator;
        if (axialDistance < -Fixed64.Epsilon || axialDistance > length + Fixed64.Epsilon)
            return;

        axialDistance = FixedMath.Clamp(axialDistance, Fixed64.Zero, length);
        Vector3d point = origin + direction * axialDistance;
        if (!MeshUtils.IsPointInTrianglePlane(first, second, third, normal, point))
            return;

        TryKeepConeTrianglePoint(origin, direction, length, endRadius, point, ref found, ref bestPoint, ref bestAxialDistance);
    }

    private static void TryKeepConeTrianglePoint(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d point,
        ref bool found,
        ref Vector3d bestPoint,
        ref Fixed64 bestAxialDistance)
    {
        if (!IsPointInsideConeVolume(origin, direction, length, endRadius, point, out Fixed64 axialDistance))
            return;

        if (found && axialDistance >= bestAxialDistance)
            return;

        bestPoint = point;
        bestAxialDistance = axialDistance;
        found = true;
    }

    private bool TryBuildConeHitForCompound(
        LSCompoundCollider compound,
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Physics3DHit hit)
    {
        bool found = false;
        Physics3DHit best = default;
        for (int i = 0; i < compound.PartCount; i++)
        {
            if (!TryBuildConeHitForCollider(compound.GetPartCollider(i), origin, direction, length, endRadius, out Physics3DHit partHit)
                || !PhysicsHitSelectionPolicy.ShouldReplace(partHit, found, best))
            {
                continue;
            }

            best = new Physics3DHit(compound, partHit.Point, partHit.Normal, partHit.Distance, partHit.Direction);
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
        Vector3d delta = point - origin;
        axialDistance = Vector3d.Dot(delta, direction);
        if (axialDistance < Fixed64.Zero || axialDistance > length)
            return false;

        Fixed64 radialSqr = delta.MagnitudeSquared - axialDistance * axialDistance;
        Fixed64 coneRadius = endRadius * (axialDistance / length);
        return radialSqr <= coneRadius * coneRadius + Fixed64.Epsilon;
    }

    private static Vector3d GetClosestPointOnConeAxis(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Vector3d point)
    {
        Fixed64 axial = Vector3d.Dot(point - origin, direction);
        axial = FixedMath.Clamp(axial, Fixed64.Zero, length);
        return origin + direction * axial;
    }

    private static void GetConeQueryBounds(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        out Vector3d min,
        out Vector3d max)
    {
        Vector3d end = origin + direction * length;
        ConeGeometry.CreateFiniteConeBounds(origin, end, direction, endRadius, out min, out max);
    }

    private static Vector3d ValidateConeQuery(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius)
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
        Vector3d requestedDisplacement = normalizedDirection * length;
        SwiftThrowHelper.ThrowIfArgument(
            !FixedVectorDifference.TryTranslate(origin, requestedDisplacement, out _),
            nameof(length),
            "Cone query endpoint must be representable without fixed-point saturation.");
        return normalizedDirection;
    }
}
