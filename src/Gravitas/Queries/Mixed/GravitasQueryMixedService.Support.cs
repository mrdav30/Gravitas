//=======================================================================
// GravitasQueryMixedService.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed query hit construction, eligibility, bounds, and shared support helpers.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private static PhysicsMixedHit BuildSphereAgainst2DHit(
        LSCollider2D collider,
        Vector3d sweepCenter,
        Fixed64 radius,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        Vector3d direction)
    {
        Vector3d point2D = GetClosestEmbeddedPoint(collider, sweepCenter);
        Vector3d to2D = point2D - sweepCenter;
        Vector3d normal3DTo2D = to2D.MagnitudeSquared > Fixed64.Epsilon
            ? to2D.Normalized
            : Resolve3DTo2DFallback(collider, sweepCenter, direction);
        Vector3d point3D = sweepCenter + normal3DTo2D * radius;
        return new PhysicsMixedHit(
            null,
            collider,
            point3D,
            point2D,
            normal3DTo2D,
            reducerKind,
            distance,
            direction);
    }

    private static PhysicsMixedHit BuildCircleAgainst3DHit(
        LSCollider collider,
        Vector3d sweepCenter,
        Vector3d direction,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        LSCollider2D? sourceCollider)
    {
        Vector3d point3D = GetSweepSurfacePoint(collider, sweepCenter, direction);
        return BuildCircleAgainst3DHit(
            collider,
            point3D,
            sweepCenter,
            direction,
            radius,
            slabCenterY,
            halfThickness,
            reducerKind,
            distance,
            sourceCollider);
    }

    private static PhysicsMixedHit BuildCircleAgainst3DHit(
        LSCollider collider,
        Vector3d point3D,
        Vector3d sweepCenter,
        Vector3d direction,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        LSCollider2D? sourceCollider)
    {
        Vector3d to2D = sweepCenter - point3D;
        Vector3d normal3DTo2D = to2D.MagnitudeSquared > Fixed64.Epsilon
            ? to2D.Normalized
            : -direction;
        Vector2d planarNormal = new(normal3DTo2D.X, normal3DTo2D.Z);
        Vector2d planarPoint = new(sweepCenter.X, sweepCenter.Z);
        if (planarNormal.MagnitudeSquared > Fixed64.Epsilon)
            planarPoint -= planarNormal.Normalized * radius;

        Vector3d point2D = new(
            planarPoint.X,
            ClampAxis(point3D.Y, slabCenterY - halfThickness, slabCenterY + halfThickness),
            planarPoint.Y);
        return new PhysicsMixedHit(
            collider,
            sourceCollider,
            point3D,
            point2D,
            normal3DTo2D,
            reducerKind,
            distance,
            direction);
    }

    private static Vector3d GetClosestEmbeddedPoint(LSCollider2D collider, Vector3d sweepCenter)
    {
        return MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(collider, sweepCenter);
    }

    private static Vector3d GetSweepSurfacePoint(LSCollider collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return collider.Center - direction * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(sweepCenter);
    }

    private static Vector3d Resolve3DTo2DFallback(LSCollider2D collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d embeddedCenter = new(collider.Center.X, collider.MixedSlabCenterY, collider.Center.Y);
        Vector3d to2D = embeddedCenter - sweepCenter;
        if (to2D.MagnitudeSquared > Fixed64.Epsilon)
            return to2D.Normalized;

        return direction;
    }

    internal static bool TrySweepCircleSlabSide(
        Fixed64 localStartY,
        Fixed64 localEndY,
        Vector2d planarStart,
        Vector2d planarEnd,
        Vector2d planarSpatialDirection,
        Fixed64 length,
        FixedBoundCircle radialBound,
        Fixed64 radiusExpansion,
        Fixed64 halfHeight,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        // The parent reducer admits only a representable 3D segment. Its X/Z
        // projection is therefore representable as well.
        Vector2d planarSegment = planarEnd - planarStart;

        var ray = new FixedRay2d(planarStart, planarSegment);
        if (!ray.TryGetIntersectionInterval(
                radialBound,
                radiusExpansion,
                Fixed64.One,
                out Fixed64 first,
                out Fixed64 second))
        {
            return false;
        }

        bool useExit = !IsCircleSlabSideHit(localStartY, localEndY, halfHeight, first);
        Fixed64 parameter = useExit ? second : first;
        // When entry and exit are the same, the side predicate is identical at
        // both roots, so the second check already rejects an unusable tangent.
        if (!IsCircleSlabSideHit(localStartY, localEndY, halfHeight, parameter))
            return false;

        // A zero-distance side root would already have been admitted by the
        // parent reducer's inclusive starting-overlap check.
        if (parameter == Fixed64.One)
        {
            distance = length;
            return true;
        }

        // Keep the established spatial-distance rounding when the normalized
        // line can represent the same root. The authored segment interval
        // above remains authoritative for admission and finite-Y clipping.
        var spatialRay = new FixedRay2d(planarStart, planarSpatialDirection);
        if (spatialRay.TryGetIntersectionInterval(
                radialBound,
                radiusExpansion,
                length,
                out Fixed64 spatialEntry,
                out Fixed64 spatialExit))
        {
            distance = useExit ? spatialExit : spatialEntry;
            return true;
        }

        distance = FixedMath.Lerp(Fixed64.Zero, length, parameter);
        return true;
    }

    internal static bool TrySweepCircleSlabCap(
        Fixed64 localStartY,
        Fixed64 localEndY,
        Fixed64 directionY,
        Vector2d planarStart,
        Vector2d planarEnd,
        Fixed64 length,
        FixedBoundCircle radialBound,
        Fixed64 radiusExpansion,
        Fixed64 capY,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (!Fixed64.TrySubtract(localEndY, localStartY, out Fixed64 verticalSegment)
            || verticalSegment == Fixed64.Zero
            || !Fixed64.TrySubtract(capY, localStartY, out Fixed64 capOffset))
        {
            return false;
        }

        Fixed64 candidate = capOffset / verticalSegment;
        if (candidate < Fixed64.Zero || candidate > Fixed64.One)
            return false;

        Vector2d planarPoint = Vector2d.Lerp(planarStart, planarEnd, candidate);
        if (!IsInsideExpandedCircle(planarPoint, radialBound, radiusExpansion))
            return false;

        if (candidate == Fixed64.One)
        {
            distance = length;
            return true;
        }

        if (directionY != Fixed64.Zero)
        {
            Fixed64 spatialDistance = capOffset / directionY;
            if (spatialDistance >= Fixed64.Zero && spatialDistance <= length)
            {
                distance = spatialDistance;
                return true;
            }
        }

        distance = FixedMath.Lerp(Fixed64.Zero, length, candidate);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsCircleSlabSideHit(
        Fixed64 localStartY,
        Fixed64 localEndY,
        Fixed64 halfHeight,
        Fixed64 parameter)
    {
        // FixedRay2d.TryGetIntersectionInterval is called with maxDistance=1,
        // so both roots are already guaranteed to be in [0, 1].
        Fixed64 y = FixedMath.Lerp(localStartY, localEndY, parameter);
        return y >= -halfHeight && y <= halfHeight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideCircleSlab(
        Fixed64 localY,
        Vector2d planarPoint,
        FixedBoundCircle radialBound,
        Fixed64 radiusExpansion,
        Fixed64 halfHeight) =>
        localY >= -halfHeight
        && localY <= halfHeight
        && IsInsideExpandedCircle(planarPoint, radialBound, radiusExpansion);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideExpandedCircle(
        Vector2d point,
        FixedBoundCircle radialBound,
        Fixed64 radiusExpansion) =>
        new FixedRay2d(point, Vector2d.Zero).Intersects(
            radialBound,
            radiusExpansion,
            Fixed64.Zero).HasValue;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TryKeepEarlierSweep(
        bool candidateFound,
        Fixed64 candidateDistance,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        if (!candidateFound || candidateDistance >= bestDistance)
            return;

        found = true;
        bestDistance = candidateDistance;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ResetLastQueryCounters()
    {
        LastQueryCandidateCount = 0;
        LastMeshTriangleCandidateCount = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligible2DTarget(
        LSCollider2D collider,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        if (!includeTriggers && collider.IsTrigger)
            return false;

        return excludedCollider == null
            || (!ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !excludedCollider.ExcludesMixedCollisionWith(collider));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligible3DTarget(
        LSCollider collider,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        if (!includeTriggers && collider.IsTrigger)
            return false;

        return excludedCollider == null
            || (!ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !collider.ExcludesMixedCollisionWith(excludedCollider));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void CreateSweepBounds(Vector3d start, Vector3d end, Fixed64 radius, out Vector3d min, out Vector3d max)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        min = Vector3d.Min(start, end) - radiusExtents;
        max = Vector3d.Max(start, end) + radiusExtents;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void CreateCircleSlabSweepBounds(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        out Vector3d min,
        out Vector3d max)
    {
        min = new Vector3d(
            FixedMath.Min(start.X, end.X) - radius,
            slabCenterY - halfThickness,
            FixedMath.Min(start.Y, end.Y) - radius);
        max = new Vector3d(
            FixedMath.Max(start.X, end.X) + radius,
            slabCenterY + halfThickness,
            FixedMath.Max(start.Y, end.Y) + radius);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampAxis(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;

}
