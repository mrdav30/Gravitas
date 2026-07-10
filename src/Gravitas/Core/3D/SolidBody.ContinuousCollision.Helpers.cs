//=======================================================================
// SolidBody.ContinuousCollision.Helpers.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    private bool IsEligibleDynamicContinuousCollisionTarget(SolidBody target) =>
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(
            ReferenceEquals(target, this),
            target.Active,
            target.IsPositionFullyFrozen,
            target.IsKinematic,
            target.Collider.IsTrigger,
            target.Collider.IsSibling(Collider),
            Context.Physics.IsLayerCollisionDisabled(Collider.Layer, target.Collider.Layer),
            ColliderCollisionFilter.AllowsPhysicalPair(Collider, target.Collider));

    private bool IsEligibleDynamicMixed2DTarget(SolidBody2D target) =>
        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(
            target.Active,
            target.IsPositionFullyFrozen,
            target.IsKinematic,
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
    internal Vector3d ResolveContinuousCollisionFrameVelocity() =>
        ProjectLinearMotion(_continuousCollisionFrameDisplacement / Context.DeltaTime);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        return Collider switch
        {
            LSSphereCollider sphere => sphere.ScaledRadius,
            _ => ResolveBoundsProxyRadius(Collider)
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveBoundsProxyRadius(LSCollider collider)
    {
        Fixed64 radius = collider.Bounds.Scope.Magnitude;
        return radius > Fixed64.Epsilon ? radius : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClosingContinuousCollisionHit(Vector3d displacement, Vector3d normal) =>
        normal.MagnitudeSquared > Fixed64.Epsilon
        && Vector3d.Dot(displacement, normal) < -Fixed64.Epsilon;

    private bool IsValidContinuousCollisionTarget(LSCollider hitCollider)
    {
        SolidBody? hitBody = hitCollider.Body;
        return ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(
            hasCollider: true,
            ReferenceEquals(hitCollider, Collider),
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider3D),
            hitCollider.IsTrigger,
            hitCollider.IsSibling(Collider),
            Context.Physics.IsLayerCollisionDisabled(Collider.Layer, hitCollider.Layer),
            ColliderCollisionFilter.AllowsPhysicalPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

    private bool IsValidMixedContinuousCollisionHit(LSCollider2D hitCollider)
    {
        SolidBody2D? hitBody = hitCollider.Body;
        return ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(
            hasCollider: true,
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider2D),
            hitCollider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

}
