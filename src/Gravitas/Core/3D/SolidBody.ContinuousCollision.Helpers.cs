//=======================================================================
// SolidBody.ContinuousCollision.Helpers.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    private bool IsEligibleDynamicContinuousCollisionTarget(SolidBody target) =>
        (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
            HasContinuousCollisionRotationalMotion))
        && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
            target.Collider,
            _continuousCollisionHandoffIgnoredCollider3D)
        && ContinuousCollisionTargetPolicy.AllowsIndexed3DTarget(
            ReferenceEquals(target, this),
            target.Active,
            target.IsDynamic,
            target.IsKinematic,
            target.IsKinematic && target.HasContinuousCollisionMotion,
            target.Collider.IsTrigger,
            Context.Physics.RequireCollisionPair(Collider, target.Collider));

    private bool IsEligibleDynamicMixed2DTarget(SolidBody2D target) =>
        (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
            HasContinuousCollisionRotationalMotion))
        && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
            target.Collider,
            _continuousCollisionHandoffIgnoredCollider2D)
        && ContinuousCollisionTargetPolicy.AllowsMixedIndexedTarget(
            target.Active,
            target.IsDynamic,
            target.IsKinematic,
            target.IsKinematic && target.HasContinuousCollisionMotion,
            target.Collider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(Collider, target.Collider));

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

        SolidBody? parentBody = Collider.TopParent3D?.Body;
        if (parentBody != null && parentBody._continuousCollisionMode != ContinuousCollisionMode.Inherit)
            return parentBody._continuousCollisionMode;

        mode = Context.Settings.DefaultContinuousCollisionMode;
        return mode == ContinuousCollisionMode.Inherit
            ? ContinuousCollisionMode.Discrete
            : mode;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        bool offsetResolved =
            Collider.TryGetCurrentScaledOffset(out Vector3d centerOffset);
        bool distanceResolved =
            centerOffset.TryGetMagnitudeCeiling(
                out Fixed64 offsetDistance);
        bool radiusResolved = Fixed64.TryAdd(
            offsetDistance,
            Collider.CanonicalCenteredProxyRadius,
            out Fixed64 radius);
        return offsetResolved & distanceResolved & radiusResolved
            ? radius
            : Fixed64.MaxValue;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClosingContinuousCollisionHit(Vector3d displacement, Vector3d normal) =>
        Vector3d.Dot(displacement, normal) < -Fixed64.Epsilon;

    private bool IsValidContinuousCollisionTarget(LSCollider hitCollider)
    {
        SolidBody? hitBody = hitCollider.Body;
        if (hitBody?.IsKinematic == true)
        {
            hitBody.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
            if (hitBody.HasContinuousCollisionMotion)
                return false;
        }

        return ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(
            ReferenceEquals(hitCollider, Collider),
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider3D),
            hitCollider.IsTrigger,
            Context.Physics.RequireCollisionPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

    private bool IsValidMixedContinuousCollisionHit(LSCollider2D hitCollider)
    {
        SolidBody2D? hitBody = hitCollider.Body;
        if (hitBody?.IsKinematic == true)
        {
            hitBody.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
            if (hitBody.HasContinuousCollisionMotion)
                return false;
        }

        return ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider2D),
            hitCollider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

}
