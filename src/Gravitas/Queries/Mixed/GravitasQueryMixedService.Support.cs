//=======================================================================
// GravitasQueryMixedService.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
        Vector3d direction,
        Vector3d featureNormal = default)
    {
        ContactAnchor anchor2D =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                collider,
                sweepCenter);
        bool offsetResolved = anchor2D.TryGetOffsetFrom(
            sweepCenter,
            out Vector3d to2D);
        Vector3d normal3DTo2D;
        if (featureNormal != Vector3d.Zero)
        {
            normal3DTo2D = featureNormal;
        }
        else if (offsetResolved
                 && to2D.Y != Fixed64.Zero
                 && new Vector2d(to2D.X, to2D.Z).MagnitudeSquared <= Fixed64.Epsilon)
        {
            normal3DTo2D = to2D.Y > Fixed64.Zero
                ? Vector3d.Up
                : -Vector3d.Up;
        }
        else
        {
            normal3DTo2D = offsetResolved
                && to2D.MagnitudeSquared > Fixed64.Epsilon
                ? to2D.Normalized
                : Resolve3DTo2DFallback(collider, sweepCenter, direction);
        }
        var anchor3D = new ContactAnchor(
            sweepCenter,
            normal3DTo2D * radius);
        return new PhysicsMixedHit(
            null,
            collider,
            anchor3D,
            anchor2D,
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
        return BuildCircleAgainst3DHit(
            collider,
            GetSweepSurfaceAnchor(
                collider,
                sweepCenter,
                direction),
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
        FixedPointAnchor point3D,
        Vector3d sweepCenter,
        Vector3d direction,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        LSCollider2D? sourceCollider)
    {
        var anchor3D = new ContactAnchor(point3D);
        bool offsetResolved = anchor3D.TryGetOffsetFrom(
            sweepCenter,
            out Vector3d fromSweepCenter);
        Vector3d to2D = offsetResolved
            ? -fromSweepCenter
            : Vector3d.Zero;
        Vector3d normal3DTo2D = offsetResolved
            && to2D.MagnitudeSquared > Fixed64.Epsilon
            ? to2D.Normalized
            : -direction;
        Vector2d planarNormal = new(
            normal3DTo2D.X,
            normal3DTo2D.Z);
        Vector2d planarOffset = planarNormal.MagnitudeSquared > Fixed64.Epsilon
            ? -planarNormal.Normalized * radius
            : Vector2d.Zero;
        Fixed64 verticalOffset = offsetResolved
            ? FixedMath.Clamp(
                fromSweepCenter.Y,
                -halfThickness,
                halfThickness)
            : Fixed64.Zero;
        var anchor2D = new ContactAnchor(
            sweepCenter,
            new Vector3d(
                planarOffset.X,
                verticalOffset,
                planarOffset.Y));
        return new PhysicsMixedHit(
            collider,
            sourceCollider,
            anchor3D,
            anchor2D,
            normal3DTo2D,
            reducerKind,
            distance,
            direction);
    }

    private static FixedPointAnchor GetSweepSurfaceAnchor(
        LSCollider collider,
        Vector3d sweepCenter,
        Vector3d direction)
    {
        if (collider is LSCuboidCollider cuboid)
            return cuboid.OrientedBox.GetClosestPointAnchor(sweepCenter);

        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
        {
            return new FixedPointAnchor(
                collider.Center,
                FixedQuaternion.Identity,
                -direction * collider.ScaledRadius);
        }

        return new FixedPointAnchor(
            collider.ClosestPointOnSurface(sweepCenter),
            FixedQuaternion.Identity,
            Vector3d.Zero);
    }

    private static Vector3d Resolve3DTo2DFallback(LSCollider2D collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d embeddedCenter = new(collider.Center.X, collider.MixedSlabCenterY, collider.Center.Y);
        Vector3d to2D = embeddedCenter - sweepCenter;
        if (to2D.MagnitudeSquared > Fixed64.Epsilon)
            return to2D.Normalized;

        return direction;
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

}
