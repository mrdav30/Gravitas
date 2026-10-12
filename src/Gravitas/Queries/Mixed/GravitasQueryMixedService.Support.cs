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
        // Exact reducers bound each anchor component by the admitted sphere
        // expansion.
        _ = anchor2D.TryGetOffsetFrom(
            sweepCenter,
            out Vector3d to2D);
        Vector3d normal3DTo2D;
        if (featureNormal != Vector3d.Zero)
        {
            normal3DTo2D = featureNormal;
        }
        else if (to2D.Y != Fixed64.Zero
                 && new Vector2d(to2D.X, to2D.Z).MagnitudeSquared <= Fixed64.Epsilon)
        {
            normal3DTo2D = to2D.Y > Fixed64.Zero
                ? Vector3d.Up
                : -Vector3d.Up;
        }
        else
        {
            normal3DTo2D = to2D.MagnitudeSquared > Fixed64.Epsilon
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
        FixedPointAnchor surface = collider.GetClosestSurfaceAnchor(
            sweepCenter, out Vector3d outwardNormal, out bool contained);
        // These owners already resolve radial, cap and corner separation. The
        // cone's exterior apex/rim needs the direction from its selected witness.
        Vector3d normal = contained
            || collider is LSSphereCollider or LSCapsuleCollider or LSCylinderCollider or LSCuboidCollider
            ? outwardNormal
            : ContinuousCollisionContactPolicy.ResolveSurfaceNormal(surface, sweepCenter, outwardNormal);
        return BuildCircleAgainst3DHit(
            collider,
            surface,
            sweepCenter,
            direction,
            radius,
            slabCenterY,
            halfThickness,
            reducerKind,
            distance,
            sourceCollider,
            normal);
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
        LSCollider2D? sourceCollider,
        Vector3d featureNormal = default)
    {
        var anchor3D = new ContactAnchor(point3D);
        // Contained starts retain the solid's actual boundary, which can lie
        // beyond the small source slab; clamp only the slab's vertical witness.
        var center = new FixedPointAnchor(sweepCenter, FixedQuaternion.Identity, Vector3d.Zero);
        Vector3d normal3DTo2D = featureNormal != Vector3d.Zero ? featureNormal
            : ContinuousCollisionContactPolicy.ResolveSurfaceNormal(point3D, sweepCenter, -direction);
        Vector2d planarNormal = new(
            normal3DTo2D.X,
            normal3DTo2D.Z);
        Vector2d planarOffset = planarNormal.MagnitudeSquared > Fixed64.Epsilon
            ? -planarNormal.Normalized * radius
            : Vector2d.Zero;
        // Use the compact relative offset when available. Otherwise project Y
        // independently so X/Z overflow cannot erase it; Y overflow clamps its cap.
        bool hasY = point3D.TryGetOffsetFrom(center, out Vector3d offset);
        Fixed64 y = offset.Y;
        if (!hasY) hasY = point3D.TryGetProjectedOffsetFrom(center, Vector3d.Up, out y);
        Fixed64 verticalOffset = hasY
            ? FixedMath.Clamp(y, -halfThickness, halfThickness)
            : point3D.ProjectNonNegativeOffsetFrom(center, Vector3d.Up) > Fixed64.Zero
                ? halfThickness : -halfThickness;
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
