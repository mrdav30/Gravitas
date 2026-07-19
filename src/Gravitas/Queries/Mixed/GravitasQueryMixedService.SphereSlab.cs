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
    /// <summary>
    /// Reduces a prepared spatial-distance sphere sweep against one embedded circle slab.
    /// The direction must match the authored start/end segment and the length must use the same parameterization.
    /// </summary>
    internal static bool TrySweepSphereAgainstCircleSlab(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCircleCollider2D circle,
        out PhysicsMixedHit hit)
    {
        Vector3d center = new(circle.Center.X, circle.MixedSlabCenterY, circle.Center.Y);
        var radialBound = new FixedBoundCircle(circle.Center, circle.ScaledRadius);
        Vector2d planarStart = new(start.X, start.Z);
        Vector2d planarEnd = new(end.X, end.Z);
        Fixed64 expandedHalfHeight = circle.MixedHalfThickness + radius;
        Fixed64 localStartY = start.Y - center.Y;
        Fixed64 localEndY = end.Y - center.Y;

        if (IsInsideCircleSlab(
                localStartY,
                planarStart,
                radialBound,
                radius,
                expandedHalfHeight))
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
            TrySweepCircleSlabSide(
                localStartY,
                localEndY,
                planarStart,
                planarEnd,
                new Vector2d(direction.X, direction.Z),
                length,
                radialBound,
                radius,
                expandedHalfHeight,
                out Fixed64 sideDistance),
            sideDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepCircleSlabCap(
                localStartY,
                localEndY,
                direction.Y,
                planarStart,
                planarEnd,
                length,
                radialBound,
                radius,
                expandedHalfHeight,
                out Fixed64 topDistance),
            topDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepCircleSlabCap(
                localStartY,
                localEndY,
                direction.Y,
                planarStart,
                planarEnd,
                length,
                radialBound,
                radius,
                -expandedHalfHeight,
                out Fixed64 bottomDistance),
            bottomDistance,
            ref found,
            ref bestDistance);

        if (!found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = bestDistance == length ? end : start + direction * bestDistance;
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
        Vector3d end,
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
        Fixed64 distance = Fixed64.Zero;
        if (!SweepBoundsUtility.OverlapsInclusive(start, start, min, max)
            && !SweepBoundsUtility.TryClipSegment(start, direction, length, min, max, out distance, out _))
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = distance == length ? end : start + direction * distance;
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
