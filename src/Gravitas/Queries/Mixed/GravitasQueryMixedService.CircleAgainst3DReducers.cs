//=======================================================================
// GravitasQueryMixedService.CircleAgainst3DReducers.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
        Vector2d end,
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
                end,
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
            return TrySweepCircleAgainstCapsule(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                capsule,
                sourceCollider,
                out hit);

        if (collider is LSCylinderCollider cylinder)
            return TrySweepCircleAgainstCylinder(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cylinder,
                sourceCollider,
                out hit);

        if (collider is LSConeCollider cone)
            return TrySweepCircleAgainstCone(start, direction2D, length, radius, slabCenterY, halfThickness, direction3D, cone, sourceCollider, out hit);

        if (collider is LSMeshCollider mesh)
            return TrySweepCircleAgainstMesh(start, end, direction2D, length, radius, slabCenterY, halfThickness, direction3D, mesh, sourceCollider, out hit);

        if (collider is LSCompoundCollider compound)
            return TrySweepCircleAgainstCompound3D(start, end, direction2D, length, radius, slabCenterY, halfThickness, direction3D, compound, sourceCollider, out hit);

        hit = default;
        return false;
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
        if (!cuboid.OrientedBox.TryGetCircleSlabSweepDistance(
                new Vector3d(start.X, slabCenterY, start.Y),
                direction,
                length,
                halfThickness,
                radius,
                out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            direction,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cuboid,
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
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        if (!FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            start,
            direction,
            length,
            radius,
            slabMinY,
            slabMaxY,
            capsule,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            direction,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            capsule,
            sourceCollider);
        return true;
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
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        if (!FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            start,
            direction,
            length,
            radius,
            slabMinY,
            slabMaxY,
            cylinder,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            direction,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cylinder,
            sourceCollider);
        return true;
    }

    private static PhysicsMixedHit BuildCircleAgainstProjectedFiniteSlabHit(
        Vector2d start,
        Vector2d direction,
        Fixed64 distance,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider collider,
        LSCollider2D? sourceCollider)
    {
        var center2D = new Vector2d(
            Fixed64.MultiplyAdd(direction.X, distance, start.X),
            Fixed64.MultiplyAdd(direction.Y, distance, start.Y));
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        return BuildCircleAgainst3DHit(
            collider,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
    }

    private static bool TrySweepCircleAgainstCone(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSConeCollider cone,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        if (!FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            start,
            direction,
            length,
            radius,
            slabMinY,
            slabMaxY,
            cone,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            direction,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cone,
            sourceCollider);
        return true;
    }

    private bool TrySweepCircleAgainstMesh(
        Vector2d start,
        Vector2d end,
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
            end,
            radius,
            slabCenterY,
            halfThickness,
            out Vector3d min,
            out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _meshTriangleCandidates);
        LastMeshTriangleCandidateCount += _meshTriangleCandidates.Count;

        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        int bestTriangleIndex = int.MaxValue;
        PhysicsMixedHit best = default;

        for (int i = 0; i < _meshTriangleCandidates.Count; i++)
        {
            int triangleIndex = _meshTriangleCandidates[i];
            mesh.Mesh.GetLocalTriangleVertices(
                triangleIndex,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            var triangle = new FixedTriangle(
                first,
                second,
                third);
            if (!triangle.TryGetFiniteSlabProjectedCircleSweep(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                start,
                direction,
                maximumDistance: length,
                radius,
                slabCenterY,
                halfThickness,
                out Fixed64 distance,
                out FixedPointAnchor triangleContact))
            {
                continue;
            }

            if (found
                && (distance > bestDistance
                    || (distance == bestDistance && triangleIndex >= bestTriangleIndex)))
            {
                continue;
            }

            Vector2d center2D = distance == length ? end : start + direction * distance;
            Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
            best = BuildCircleAgainst3DHit(
                mesh,
                triangleContact,
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
        Vector2d end,
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
                end,
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

            if (!PhysicsHitSelectionPolicy.ShouldReplaceDistance(candidate.Distance, found, bestDistance))
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
            best.Anchor3D,
            best.Anchor2D,
            best.Normal3DTo2D,
            best.ReducerKind,
            best.Distance,
            best.Direction3D);
        return true;
    }

}
