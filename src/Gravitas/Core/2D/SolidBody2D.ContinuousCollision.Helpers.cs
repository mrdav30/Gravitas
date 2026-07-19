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
        if (Collider is LSCircleCollider2D circle)
        {
            if (!Vector2d.TrySubtract(circle.Center, _position, out Vector2d centerOffset)
                || !Vector2d.TryGetMagnitude(centerOffset, out Fixed64 offsetDistance)
                || !Fixed64.TryAdd(offsetDistance, circle.ScaledRadius, out Fixed64 circleRadius))
            {
                return Fixed64.MaxValue;
            }

            return circleRadius;
        }

        return Collider.VertexCount > 0
            ? ResolveConvexContinuousCollisionProxyRadius()
            : ResolveBoundsContinuousCollisionProxyRadius();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector2d ResolveContinuousCollisionFrameVelocity() =>
        ProjectLinearMotion(ContinuousCollisionFrameDisplacement / Context.DeltaTime);

    private Fixed64 ResolveConvexContinuousCollisionProxyRadius()
    {
        int vertexCount = Collider.VertexCount;
        Fixed64 bestDistance = Fixed64.Zero;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d vertex = Collider.GetVertexUnchecked(i);
            if (!TryKeepPivotDistance(vertex, ref bestDistance))
                return Fixed64.MaxValue;
        }

        return bestDistance;
    }

    private Fixed64 ResolveBoundsContinuousCollisionProxyRadius()
    {
        Vector2d min = Collider.Bounds.Min;
        Vector2d max = Collider.Bounds.Max;
        Fixed64 bestDistance = Fixed64.Zero;
        return TryKeepPivotDistance(min, ref bestDistance)
            && TryKeepPivotDistance(new Vector2d(min.X, max.Y), ref bestDistance)
            && TryKeepPivotDistance(new Vector2d(max.X, min.Y), ref bestDistance)
            && TryKeepPivotDistance(max, ref bestDistance)
                ? bestDistance
                : Fixed64.MaxValue;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool TryKeepPivotDistance(Vector2d point, ref Fixed64 bestDistance)
    {
        if (!Vector2d.TrySubtract(point, _position, out Vector2d offset)
            || !Vector2d.TryGetMagnitude(offset, out Fixed64 distance))
        {
            return false;
        }

        bestDistance = FixedMath.Max(bestDistance, distance);
        return true;
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
