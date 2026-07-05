//=======================================================================
// GravitasQueryMixedService.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
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
            : direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Right;
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

        return direction.MagnitudeSquared > Fixed64.Epsilon ? direction.Normalized : Vector3d.Down;
    }

    private static bool TrySweepBox(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Vector3d min,
        Vector3d max,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (IsInsideBox(start, min, max))
            return true;

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = length;
        if (!ClipSegmentAxis(start.X, direction.X, min.X, max.X, ref entry, ref exit)
            || !ClipSegmentAxis(start.Y, direction.Y, min.Y, max.Y, ref entry, ref exit)
            || !ClipSegmentAxis(start.Z, direction.Z, min.Z, max.Z, ref entry, ref exit))
        {
            return false;
        }

        distance = entry;
        return true;
    }

    private static bool TrySweepCircleSlabSide(
        Vector3d localStart,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 halfHeight,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        Fixed64 a = direction.X * direction.X + direction.Z * direction.Z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localStart.X * direction.X + localStart.Z * direction.Z);
        Fixed64 c = localStart.X * localStart.X + localStart.Z * localStart.Z - radius * radius;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;
        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        TryKeepEarlierSweep(
            IsCircleSlabSideHit(localStart, direction, length, halfHeight, first),
            first,
            ref found,
            ref best);
        TryKeepEarlierSweep(
            IsCircleSlabSideHit(localStart, direction, length, halfHeight, second),
            second,
            ref found,
            ref best);

        distance = best;
        return found;
    }

    private static bool TrySweepCircleSlabCap(
        Vector3d localStart,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 capY,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (direction.Y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 candidate = (capY - localStart.Y) / direction.Y;
        if (candidate < Fixed64.Zero || candidate > length)
            return false;

        Vector3d localPoint = localStart + direction * candidate;
        Fixed64 radialSqr = localPoint.X * localPoint.X + localPoint.Z * localPoint.Z;
        if (radialSqr > radius * radius + Fixed64.Epsilon)
            return false;

        distance = candidate;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsCircleSlabSideHit(
        Vector3d localStart,
        Vector3d direction,
        Fixed64 length,
        Fixed64 halfHeight,
        Fixed64 distance)
    {
        if (distance < Fixed64.Zero || distance > length)
            return false;

        Fixed64 y = localStart.Y + direction.Y * distance;
        return y >= -halfHeight && y <= halfHeight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideCircleSlab(Vector3d localPoint, Fixed64 radius, Fixed64 halfHeight) =>
        localPoint.Y >= -halfHeight
        && localPoint.Y <= halfHeight
        && localPoint.X * localPoint.X + localPoint.Z * localPoint.Z <= radius * radius;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideBox(Vector3d point, Vector3d min, Vector3d max) =>
        point.X >= min.X && point.X <= max.X
        && point.Y >= min.Y && point.Y <= max.Y
        && point.Z >= min.Z && point.Z <= max.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ClipSegmentAxis(
        Fixed64 position,
        Fixed64 direction,
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        if (direction.Abs() <= Fixed64.Epsilon)
            return position >= min && position <= max;

        Fixed64 first = (min - position) / direction;
        Fixed64 second = (max - position) / direction;
        if (first > second)
            (first, second) = (second, first);

        if (first > entry)
            entry = first;
        if (second < exit)
            exit = second;
        return entry <= exit;
    }

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
        if (!collider.IsActive || (!includeTriggers && collider.IsTrigger))
            return false;

        if (staticTargetsOnly)
        {
            SolidBody2D? body = collider.Body;
            if (!collider.IsStatic && (body == null || !body.IsKinematic))
                return false;
        }

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
        if (!collider.IsActive || (!includeTriggers && collider.IsTrigger))
            return false;

        if (staticTargetsOnly)
        {
            SolidBody? body = collider.Body;
            if (!collider.IsStatic && (body == null || !body.IsKinematic))
                return false;
        }

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
