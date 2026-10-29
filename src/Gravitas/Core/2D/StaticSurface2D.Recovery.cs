// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using System;

namespace Gravitas;

/// <content>Conservative deep-placement proposals certified against the complete final spatial neighborhood.</content>
public sealed partial class StaticSurface2D
{
    internal bool TryGetRecovery(LSCollider2D mover, out Vector2d center)
    {
        center = mover.Center;
        SolidBody2D body = mover.Body!;
        if (!body.CanTranslate) return false;
        if (IsRecoveryClear(mover, center)) return true;
        foreach (LSCollider2D member in _local.AsReadOnlySpan())
        {
            if (!CollisionDetection2D.DoesPostureCandidatePenetrate(mover, center, member)
                || !TryRecoveryDirection(mover, center, member, out Vector2d normal, out Fixed64 depth)) continue;
            // Existing penetration slop only expands a proposal beyond its
            // rounded member exit; exact final clearance is still mandatory.
            if (!Fixed64.TryAdd(depth, CollisionResponse2D.PenetrationSlop, out Fixed64 distance)
                || !Fixed64.TryMultiplyDivide(-normal.X, distance, Fixed64.One, out Fixed64 x)
                || !Fixed64.TryMultiplyDivide(-normal.Y, distance, Fixed64.One, out Fixed64 y)
                || !Vector2d.TryAdd(center, body.ProjectLinearMotion(new(x, y)), out Vector2d next)) continue;
            center = next;
            if (IsRecoveryClear(mover, center)) return true;
        }
        center = mover.Center;
        return false;
    }

    internal bool IsRecoveryClear(LSCollider2D mover, Vector2d center)
    {
        if (!Vector2d.TrySubtract(center, mover.Center, out Vector2d displacement)) return false;
        SolidBody2D body = mover.Body!;
        if (!Vector2d.TryAdd(body.Position, displacement, out Vector2d position)
            || !mover.TryPrepareRecoveryPose(position, body.Rotation, out Vector2d preparedCenter)) return false;
        // A translated offset can change a midpoint tie by one raw unit;
        // certify the center the body will actually publish.
        center = preparedCenter;
        FixedBoundArea bounds = FixedBoundArea.FromCenterAndScopeClippedToDomain(center,
            Vector2d.One * mover.CanonicalCenteredProxyRadius);
        _context.Collisions2D.CollectBoundsCandidates(bounds.Min, bounds.Max, PhysicsLayerMask.All,
            _context.Query2D.NextOverlapQueryVersion(), false, _candidates);
        foreach (LSCollider2D candidate in _candidates.AsReadOnlySpan())
        {
            if (candidate.IsTrigger || !_context.Physics2D.RequireCollisionPair(mover, candidate)) continue;
            if (CollisionDetection2D.DoesPostureCandidatePenetrate(mover, center, candidate)) return false;
        }
        return true;
    }

    private static bool TryRecoveryDirection(LSCollider2D mover, Vector2d center, LSCollider2D member,
        out Vector2d normal, out Fixed64 depth)
    {
        Span<Vector2d> targetBox = stackalloc Vector2d[4], movingBox = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> target = Offsets(member, targetBox);
        bool hit, clamped;
        if (mover is LSCircleCollider2D or LSCapsuleCollider2D)
        {
            Span<FixedPointAnchor2d> first = stackalloc FixedPointAnchor2d[2], second = stackalloc FixedPointAnchor2d[2];
            hit = WideCenteredCapsule2dRelations.TryGetContacts(center, mover.Rotation, Vector2d.Forward,
                mover is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero,
                mover is LSCircleCollider2D circle ? circle.ScaledRadius : ((LSCapsuleCollider2D)mover).ScaledRadius,
                member.Center, member.ConvexRotation, target, first, second, out _, out normal, out depth, out clamped);
        }
        else
        {
            Span<Vector2d> first = stackalloc Vector2d[2], second = stackalloc Vector2d[2];
            hit = WideConvex2dRelations.TryGetContactOffsets(center, mover.ConvexRotation, Offsets(mover, movingBox),
                member.Center, member.ConvexRotation, target, first, second, out _, out normal, out depth, out clamped);
        }
        // Admitted colliders have reach <= 4,096; an intersecting local
        // pair's penetration cannot approach the scalar depth limit.
        System.Diagnostics.Debug.Assert(!hit || !clamped);
        return hit;
    }
}
