//=======================================================================
// SolidBody2D.ContinuousCollision.Helpers.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool ShouldUseContinuousCollision(out ContinuousCollisionMode mode)
    {
        mode = ResolveContinuousCollisionMode();
        return mode == ContinuousCollisionMode.Continuous || mode == ContinuousCollisionMode.Auto;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ContinuousCollisionMode ResolveContinuousCollisionMode()
    {
        ContinuousCollisionMode mode = _continuousCollisionMode;
        if (mode != ContinuousCollisionMode.Inherit)
            return mode;

        SolidBody2D? parentBody = Collider.TopParent2D?.Body;
        if (parentBody != null && parentBody._continuousCollisionMode != ContinuousCollisionMode.Inherit)
            return parentBody._continuousCollisionMode;

        mode = Context.Settings.DefaultContinuousCollisionMode;
        return mode == ContinuousCollisionMode.Inherit
            ? ContinuousCollisionMode.Discrete
            : mode;
    }

    internal Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        bool offsetResolved =
            Collider.TryGetCurrentScaledOffset(out Vector2d centerOffset);
        bool distanceResolved =
            centerOffset.TryGetMagnitudeCeiling(
                out Fixed64 offsetDistance);
        bool radiusResolved = Fixed64.TryAdd(
            offsetDistance,
            Collider.CanonicalCenteredProxyRadius,
            out Fixed64 proxyRadius);
        return offsetResolved & distanceResolved & radiusResolved
            ? proxyRadius
            : Fixed64.MaxValue;
    }

    internal Fixed64 ResolveMixedContinuousCollisionProxyRadius()
    {
        Vector2d mixedExtents = new(
            ResolveContinuousCollisionProxyRadius(),
            Collider.MixedHalfThickness);
        return mixedExtents.TryGetMagnitudeCeiling(out Fixed64 radius)
            ? radius
            : Fixed64.MaxValue;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsValidContinuousCollisionHit(Physics2DHit hit) =>
        IsValidContinuousCollisionTarget(hit.Collider);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClosingContinuousCollisionHit(Vector2d displacement, Vector2d normal) =>
        Vector2d.Dot(displacement, normal) < -Fixed64.Epsilon;

    private bool IsValidContinuousCollisionTarget(LSCollider2D hitCollider)
    {
        SolidBody2D? hitBody = hitCollider.Body;
        if (hitBody?.IsKinematic == true)
        {
            hitBody.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
            if (hitBody.HasContinuousCollisionMotion)
                return false;
        }

        return ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(
            ReferenceEquals(hitCollider, Collider),
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider2D),
            hitCollider.IsTrigger,
            Context.Physics2D.RequireCollisionPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

    private bool IsValidMixedContinuousCollisionHit(LSCollider hitCollider)
    {
        SolidBody? hitBody = hitCollider.Body;
        if (hitBody?.IsKinematic == true)
        {
            hitBody.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
            if (hitBody.HasContinuousCollisionMotion)
                return false;
        }

        return ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider3D),
            hitCollider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(hitCollider, Collider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

}
