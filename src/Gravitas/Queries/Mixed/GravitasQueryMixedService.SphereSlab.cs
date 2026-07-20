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
        Vector2d planarCenter = circle.Center;
        var center = new Vector3d(planarCenter.X, circle.MixedSlabCenterY, planarCenter.Y);
        var segment = new FixedSegment(start, end);
        if (!segment.TryGetFiniteCylinderIntersectionDistanceInterval(
                center,
                Vector3d.Up,
                circle.MixedHalfThickness,
                circle.ScaledRadius,
                radius,
                radius,
                length,
                out Fixed64 distance,
                out _,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = segment.GetPointAtDistance(distance, length);
        hit = BuildSphereAgainst2DHit(
            circle,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            distance,
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
