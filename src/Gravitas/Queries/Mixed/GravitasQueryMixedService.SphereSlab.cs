//=======================================================================
// GravitasQueryMixedService.SphereSlab.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed swept-sphere slab and prism helper reducers.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private static bool TrySweepSphereAgainstCircleSlab(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCircleCollider2D circle,
        out PhysicsMixedHit hit)
    {
        Vector3d center = new(circle.Center.X, circle.MixedSlabCenterY, circle.Center.Y);
        Fixed64 combinedRadius = circle.ScaledRadius + radius;
        Fixed64 expandedHalfHeight = circle.MixedHalfThickness + radius;
        Vector3d localStart = start - center;

        if (IsInsideCircleSlab(localStart, combinedRadius, expandedHalfHeight))
        {
            hit = BuildSphereAgainst2DHit(
                circle,
                start,
                radius,
                PhysicsQueryReducerKind.Exact,
                Fixed64.Zero,
                direction);
            return true;
        }

        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        TryKeepEarlierSweep(
            TrySweepCircleSlabSide(localStart, direction, length, combinedRadius, expandedHalfHeight, out Fixed64 sideDistance),
            sideDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepCircleSlabCap(localStart, direction, length, combinedRadius, expandedHalfHeight, out Fixed64 topDistance),
            topDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepCircleSlabCap(localStart, direction, length, combinedRadius, -expandedHalfHeight, out Fixed64 bottomDistance),
            bottomDistance,
            ref found,
            ref bestDistance);

        if (!found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = start + direction * bestDistance;
        hit = BuildSphereAgainst2DHit(
            circle,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            bestDistance,
            direction);
        return true;
    }

    private static bool TrySweepSphereAgainstPrismBounds(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        FixedBoundBox bounds = collider.MixedBounds3D;
        Vector3d radiusExtents = Vector3d.One * radius;
        Vector3d min = bounds.Min - radiusExtents;
        Vector3d max = bounds.Max + radiusExtents;
        if (!TrySweepBox(start, direction, length, min, max, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = start + direction * distance;
        hit = BuildSphereAgainst2DHit(
            collider,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.ConservativeFallback,
            distance,
            direction);
        return true;
    }

}
