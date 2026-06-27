//=======================================================================
// GravitasQueryMixedService.CircleAgainst3DReducers.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections.Query;
using System;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed swept-circle reducers against 3D collider targets.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private bool TrySweepCircleAgainst3DCollider(
        LSCollider collider,
        Vector2d start,
        Vector2d direction2D,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        if (collider is LSSphereCollider sphere)
        {
            return TrySweepCircleAgainstSphere(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                sphere,
                sourceCollider,
                out hit);
        }

        if (collider is LSCuboidCollider cuboid)
        {
            return TrySweepCircleAgainstCuboid(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cuboid,
                sourceCollider,
                out hit);
        }

        if (collider is LSCapsuleCollider capsule)
        {
            bool found = TrySweepCircleAgainstCapsule(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                capsule,
                sourceCollider,
                out hit,
                out bool handled);
            if (handled)
                return found;
        }

        if (collider is LSCylinderCollider cylinder)
        {
            bool found = TrySweepCircleAgainstCylinder(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cylinder,
                sourceCollider,
                out hit,
                out bool handled);
            if (handled)
                return found;
        }

        if (collider is LSMeshCollider mesh)
            return TrySweepCircleAgainstMesh(start, direction2D, length, radius, slabCenterY, halfThickness, direction3D, mesh, sourceCollider, out hit);

        if (collider is LSCompoundCollider compound)
            return TrySweepCircleAgainstCompound3D(start, direction2D, length, radius, slabCenterY, halfThickness, direction3D, compound, sourceCollider, out hit);

        hit = default;
        return false;
    }

    private static PhysicsQueryReducerKind ClassifySweepSphereAgainst2DReducer(LSCollider2D collider)
    {
        if (collider is LSCircleCollider2D
            || collider is LSCapsuleCollider2D
            || collider is LSAABBoxCollider2D
            || collider is LSPolygonCollider2D)
        {
            return PhysicsQueryReducerKind.Exact;
        }

        if (collider is LSCompoundCollider2D compound)
        {
            for (int i = 0; i < compound.PartCount; i++)
            {
                if (ClassifySweepSphereAgainst2DReducer(compound.GetPartCollider(i)) == PhysicsQueryReducerKind.ConservativeFallback)
                    return PhysicsQueryReducerKind.ConservativeFallback;
            }

            return PhysicsQueryReducerKind.Exact;
        }

        return PhysicsQueryReducerKind.ConservativeFallback;
    }

    private static PhysicsQueryReducerKind ClassifySweepCircleAgainst3DReducer(LSCollider collider)
    {
        if (collider is LSSphereCollider
            || collider is LSCuboidCollider
            || collider is LSCapsuleCollider
            || collider is LSCylinderCollider
            || collider is LSMeshCollider)
        {
            return PhysicsQueryReducerKind.Exact;
        }

        if (collider is LSCompoundCollider compound)
        {
            for (int i = 0; i < compound.PartCount; i++)
            {
                if (ClassifySweepCircleAgainst3DReducer(compound.GetPartCollider(i)) == PhysicsQueryReducerKind.ConservativeFallback)
                    return PhysicsQueryReducerKind.ConservativeFallback;
            }

            return PhysicsQueryReducerKind.Exact;
        }

        return PhysicsQueryReducerKind.ConservativeFallback;
    }

    private static bool TrySweepCircleAgainstCuboid(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCuboidCollider cuboid,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Span<Vector2d> projection = stackalloc Vector2d[32];
        if (!TryBuildCuboidSlabProjection(cuboid, slabCenterY, halfThickness, projection, out int projectionCount)
            || !TrySweepCircleAgainstConvexProjection(
                start,
                direction,
                length,
                radius,
                projection.Slice(0, projectionCount),
                out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            cuboid,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
        return true;
    }

    private static bool TrySweepCircleAgainstCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCapsuleCollider capsule,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit,
        out bool handled)
    {
        if (!TryGetVerticalSegmentInterval(capsule.LineSegmentStart, capsule.LineSegmentEnd, out Fixed64 segmentMinY, out Fixed64 segmentMaxY))
        {
            handled = true;
            return TrySweepCircleAgainstProjectedFiniteSlabTarget(
                start,
                direction,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                capsule,
                sourceCollider,
                out hit);
        }

        handled = true;
        Fixed64 verticalExcess = GetIntervalDistance(segmentMinY, segmentMaxY, slabCenterY - halfThickness, slabCenterY + halfThickness);
        Fixed64 capsuleRadius = capsule.ScaledRadius;
        if (verticalExcess > capsuleRadius)
        {
            hit = default;
            return false;
        }

        Fixed64 planarRadiusSqr = capsuleRadius * capsuleRadius - verticalExcess * verticalExcess;
        if (planarRadiusSqr < Fixed64.Zero)
            planarRadiusSqr = Fixed64.Zero;

        return TryBuildCircleAgainstPlanarCircleTargetHit(
            start,
            direction,
            length,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            capsule,
            new Vector2d(capsule.Center.X, capsule.Center.Z),
            FixedMath.Sqrt(planarRadiusSqr),
            sourceCollider,
            out hit);
    }

    private static bool TrySweepCircleAgainstCylinder(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCylinderCollider cylinder,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit,
        out bool handled)
    {
        if (!TryGetVerticalSegmentInterval(cylinder.LineSegmentStart, cylinder.LineSegmentEnd, out Fixed64 segmentMinY, out Fixed64 segmentMaxY))
        {
            handled = true;
            return TrySweepCircleAgainstProjectedFiniteSlabTarget(
                start,
                direction,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cylinder,
                sourceCollider,
                out hit);
        }

        handled = true;
        if (!IntervalsOverlap(segmentMinY, segmentMaxY, slabCenterY - halfThickness, slabCenterY + halfThickness))
        {
            hit = default;
            return false;
        }

        return TryBuildCircleAgainstPlanarCircleTargetHit(
            start,
            direction,
            length,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cylinder,
            new Vector2d(cylinder.Center.X, cylinder.Center.Z),
            cylinder.ScaledRadius,
            sourceCollider,
            out hit);
    }

    private static bool TrySweepCircleAgainstProjectedFiniteSlabTarget(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider collider,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        bool found;
        Fixed64 distance;
        if (collider is LSCapsuleCollider capsule)
        {
            found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
                start,
                direction,
                length,
                radius,
                slabMinY,
                slabMaxY,
                capsule,
                out distance);
        }
        else if (collider is LSCylinderCollider cylinder)
        {
            found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
                start,
                direction,
                length,
                radius,
                slabMinY,
                slabMaxY,
                cylinder,
                out distance);
        }
        else
        {
            distance = default;
            found = false;
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            collider,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
        return true;
    }

    private bool TrySweepCircleAgainstMesh(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSMeshCollider mesh,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        CreateCircleSlabSweepBounds(
            start,
            start + direction * length,
            radius,
            slabCenterY,
            halfThickness,
            out Vector3d min,
            out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _meshTriangleCandidates);
        LastMeshTriangleCandidateCount += _meshTriangleCandidates.Count;

        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        int bestTriangleIndex = int.MaxValue;
        PhysicsMixedHit best = default;

        for (int i = 0; i < _meshTriangleCandidates.Count; i++)
        {
            int triangleIndex = _meshTriangleCandidates[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            if (!TrySweepCircleAgainstTriangleProjection(
                start,
                direction,
                length,
                radius,
                slabMinY,
                slabMaxY,
                first,
                second,
                third,
                out Fixed64 distance,
                out Vector3d point3D))
            {
                continue;
            }

            if (found
                && (distance > bestDistance
                    || (distance == bestDistance && triangleIndex >= bestTriangleIndex)))
            {
                continue;
            }

            Vector2d center2D = start + direction * distance;
            Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
            best = BuildCircleAgainst3DHit(
                mesh,
                point3D,
                sweepCenter,
                direction3D,
                radius,
                slabCenterY,
                halfThickness,
                PhysicsQueryReducerKind.Exact,
                distance,
                sourceCollider);
            bestDistance = distance;
            bestTriangleIndex = triangleIndex;
            found = true;
        }

        hit = best;
        return found;
    }

    private bool TrySweepCircleAgainstCompound3D(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCompoundCollider compound,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        PhysicsMixedHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!TrySweepCircleAgainst3DCollider(
                part,
                start,
                direction,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                sourceCollider,
                out PhysicsMixedHit candidate))
            {
                continue;
            }

            if (found && candidate.Distance >= bestDistance)
                continue;

            best = candidate;
            bestDistance = candidate.Distance;
            found = true;
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new PhysicsMixedHit(
            compound,
            sourceCollider,
            best.Point3D,
            best.Point2D,
            best.Normal3DTo2D,
            best.ReducerKind,
            best.Distance,
            best.Direction3D);
        return true;
    }

    private static bool TryBuildCircleAgainstPlanarCircleTargetHit(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 sourceRadius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider collider,
        Vector2d targetCenter,
        Fixed64 targetPlanarRadius,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        if (!TrySweepPointInPlane(start, direction, length, targetCenter, sourceRadius + targetPlanarRadius, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            collider,
            sweepCenter,
            direction3D,
            sourceRadius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
        return true;
    }

}
