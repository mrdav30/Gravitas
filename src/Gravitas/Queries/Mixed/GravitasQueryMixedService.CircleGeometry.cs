//=======================================================================
// GravitasQueryMixedService.CircleGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Queries;

/// <summary>
/// Owns shared projected-circle sweep geometry helpers for mixed query reducers.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    internal static bool TrySweepCircleAgainstSphere(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSSphereCollider sphere,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 sphereRadius = sphere.ScaledRadius;
        if (!FixedMath.TryGetSphereSlabCrossSectionRadius(
                sphere.Center.Y,
                sphereRadius,
                slabCenterY,
                halfThickness,
                out Fixed64 planarSphereRadius))
        {
            hit = default;
            return false;
        }

        Vector2d sphereCenter = new(sphere.Center.X, sphere.Center.Z);
        if (!TrySweepPointInPlane(
            start,
            end,
            direction,
            length,
            sphereCenter,
            planarSphereRadius,
            radius,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D =
            distance == length
                ? end
                : start + direction * distance;
        Vector3d sweepCenter = new(
            center2D.X,
            slabCenterY,
            center2D.Y);
        hit = BuildCircleAgainst3DHit(
            sphere,
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

    private static bool TrySweepPointInPlane(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Vector2d point,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 distance) =>
        RadialSweepAdmission.TryIntersect(
            start,
            direction,
            length,
            point,
            radius,
            radiusExpansion,
            end,
            point,
            out distance);
}
