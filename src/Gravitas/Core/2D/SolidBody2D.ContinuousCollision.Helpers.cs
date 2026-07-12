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
        return Collider switch
        {
            LSCircleCollider2D circle => circle.ScaledRadius,
            LSCapsuleCollider2D capsule => capsule.ScaledHeight * Fixed64.Half,
            LSAABBoxCollider2D box => box.ScaledHalfExtents.Magnitude,
            LSCompoundCollider2D compound => compound.ScaledRadius,
            _ => ResolveConvexContinuousCollisionProxyRadius()
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector2d ResolveContinuousCollisionFrameVelocity() =>
        ProjectLinearMotion(_continuousCollisionFrameDisplacement / Context.DeltaTime);

    private Fixed64 ResolveConvexContinuousCollisionProxyRadius()
    {
        int vertexCount = Collider.VertexCount;
        Vector2d center = Collider.Center;
        Fixed64 bestDistanceSquared = Fixed64.Zero;
        for (int i = 0; i < vertexCount; i++)
        {
            Fixed64 distanceSquared = Vector2d.DistanceSquared(center, Collider.GetVertexUnchecked(i));
            if (distanceSquared > bestDistanceSquared)
                bestDistanceSquared = distanceSquared;
        }

        return FixedMath.Sqrt(bestDistanceSquared);
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
        return ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(
            hasCollider: true,
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
        return ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(
            hasCollider: true,
            ContinuousCollisionCandidateOrdering.IsIgnoredTarget(hitCollider, _continuousCollisionHandoffIgnoredCollider3D),
            hitCollider.IsTrigger,
            Context.MixedCollisions.RequireCollisionPair(hitCollider, Collider),
            hitCollider.IsStatic,
            hitBody != null && hitBody.IsKinematic);
    }

}
