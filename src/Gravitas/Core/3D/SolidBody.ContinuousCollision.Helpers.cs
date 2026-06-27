//=======================================================================
// SolidBody.ContinuousCollision.Helpers.cs
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

public partial class SolidBody
{
    private bool IsEligibleDynamicContinuousCollisionTarget(SolidBody target)
    {
        if (ReferenceEquals(target, this)
            || !target.Active
            || target.IsPositionFullyFrozen
            || target.IsKinematic
            || target.Collider.IsTrigger
            || target.Collider.IsSibling(Collider)
            || Context.Physics.IsLayerCollisionDisabled(Collider.Layer, target.Collider.Layer)
            || !ColliderCollisionFilter.AllowsPhysicalPair(Collider, target.Collider))
        {
            return false;
        }

        return true;
    }

    private bool IsEligibleDynamicMixed2DTarget(SolidBody2D target)
    {
        return target.Active
            && !target.IsPositionFullyFrozen
            && !target.IsKinematic
            && !target.Collider.IsTrigger
            && Context.MixedCollisions.RequireCollisionPair(Collider, target.Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveDynamicContactPoint(
        Vector3d sourceCenter,
        Vector3d targetCenter,
        Vector3d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector3d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
    }

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
    private Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        return ResolveContinuousCollisionProxyRadius(Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadiusForDynamicTarget()
    {
        return ResolveContinuousCollisionProxyRadius(Collider);
    }

    internal Vector3d ResolveContinuousCollisionFrameVelocity()
    {
        Fixed64 deltaTime = Context.DeltaTime;
        return deltaTime > Fixed64.Epsilon
            ? ProjectLinearMotion(_continuousCollisionFrameDisplacement / deltaTime)
            : Vector3d.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveContinuousCollisionProxyRadius(LSCollider collider)
    {
        return collider switch
        {
            LSSphereCollider sphere => sphere.ScaledRadius,
            _ => ResolveBoundsProxyRadius(collider)
        };
    }

    private static Fixed64 ResolveBoundsProxyRadius(LSCollider collider)
    {
        Fixed64 radius = collider.Bounds.Scope.Magnitude;
        return radius > Fixed64.Epsilon ? radius : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsValidContinuousCollisionHit(Physics3DHit hit) =>
        IsValidContinuousCollisionTarget(hit.Collider);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClosingContinuousCollisionHit(Vector3d displacement, Vector3d normal) =>
        normal.MagnitudeSquared > Fixed64.Epsilon
        && Vector3d.Dot(displacement, normal) < -Fixed64.Epsilon;

    private bool IsValidContinuousCollisionTarget(LSCollider? hitCollider)
    {
        if (hitCollider == null
            || ReferenceEquals(hitCollider, Collider)
            || ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider3D)
            || hitCollider.IsTrigger
            || hitCollider.IsSibling(Collider)
            || Context.Physics.IsLayerCollisionDisabled(Collider.Layer, hitCollider.Layer)
            || !ColliderCollisionFilter.AllowsPhysicalPair(Collider, hitCollider))
        {
            return false;
        }

        SolidBody? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.IsPositionFullyFrozen || hitBody.IsKinematic;
    }

    private bool IsValidMixedContinuousCollisionHit(PhysicsMixedHit hit)
    {
        LSCollider2D? hitCollider = hit.Collider2D;
        if (hitCollider == null
            || ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider2D)
            || hitCollider.IsTrigger
            || !Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider))
        {
            return false;
        }

        SolidBody2D? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.IsPositionFullyFrozen || hitBody.IsKinematic;
    }

}
