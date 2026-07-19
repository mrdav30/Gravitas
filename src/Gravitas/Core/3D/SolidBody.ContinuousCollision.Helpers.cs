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
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(
            ReferenceEquals(target, this),
            target.Active,
            target.IsPositionFullyFrozen,
            target.IsKinematic,
            target.Collider.IsTrigger,
            Context.Physics.RequireCollisionPair(Collider, target.Collider));

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
        ProjectLinearMotion(ContinuousCollisionFrameDisplacement / Context.DeltaTime);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        if (Collider is LSSphereCollider sphere)
        {
            if (!Vector3d.TrySubtract(sphere.Center, Position3d, out Vector3d centerOffset)
                || !Vector3d.TryGetMagnitude(centerOffset, out Fixed64 offsetDistance)
                || !Fixed64.TryAdd(offsetDistance, sphere.ScaledRadius, out Fixed64 sphereRadius))
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
            if (!Vector3d.TrySubtract(corners[i], Position3d, out Vector3d offset)
                || !Vector3d.TryGetMagnitude(offset, out Fixed64 distance))
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
        return ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(
            hasCollider: true,
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider2D),
            hitCollider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

}
