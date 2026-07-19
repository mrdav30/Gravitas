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
            target.IsPositionFullyFrozen,
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
            target.IsPositionFullyFrozen,
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
        if (Collider is LSSphereCollider sphere)
        {
            bool offsetResolved = Vector3d.TrySubtract(
                sphere.Center,
                Position3d,
                out Vector3d centerOffset);
            bool distanceResolved = Vector3d.TryGetMagnitude(
                centerOffset,
                out Fixed64 offsetDistance);
            bool radiusResolved = Fixed64.TryAdd(
                offsetDistance,
                sphere.ScaledRadius,
                out Fixed64 sphereRadius);
            if (!(offsetResolved & distanceResolved & radiusResolved))
            {
                return Fixed64.MaxValue;
            }

            return sphereRadius;
        }

        Span<Vector3d> corners = stackalloc Vector3d[FixedMathSharp.Bounds.FixedBoundBox.CornerCount];
        Collider.Bounds.CopyCorners(corners);
        Fixed64 bestDistance = Fixed64.Zero;
        for (int i = 0; i < corners.Length; i++)
        {
            bool offsetResolved = Vector3d.TrySubtract(
                corners[i],
                Position3d,
                out Vector3d offset);
            bool distanceResolved = Vector3d.TryGetMagnitude(offset, out Fixed64 distance);
            if (!(offsetResolved & distanceResolved))
            {
                return Fixed64.MaxValue;
            }

            bestDistance = FixedMath.Max(bestDistance, distance);
        }

        return bestDistance;
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
            hasCollider: true,
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
            hasCollider: true,
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider2D),
            hitCollider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

}
