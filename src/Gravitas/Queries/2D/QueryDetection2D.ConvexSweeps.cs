//=======================================================================
// QueryDetection2D.ConvexSweeps.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Queries;

internal static partial class QueryDetection2D
{
    private static bool TrySweepCapsuleMover(
        LSCapsuleCollider2D mover,
        Vector2d displacement,
        Fixed64 length,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            hit = new Physics2DHit(target, overlap.AnchorB, -overlap.Normal, Fixed64.Zero);
            return true;
        }

        Vector2d direction = displacement.Normalized;

        if (target is LSCircleCollider2D circle)
        {
            bool found = FixedSegment2d.TryGetSweptCenteredCapsuleSegmentFirstDistance(
                mover.Center,
                mover.Rotation,
                mover.AxisLength,
                mover.ScaledRadius,
                direction,
                length,
                new FixedSegment2d(circle.Center, circle.Center),
                circle.ScaledRadius,
                out Fixed64 distance,
                out Vector2d normal);
            return TryBuildCapsuleTargetSweepHit(
                mover,
                direction,
                target,
                circle.Center,
                circle.Rotation,
                Fixed64.Zero,
                circle.ScaledRadius,
                found,
                distance,
                normal,
                out hit);
        }

        if (target is LSCapsuleCollider2D targetCapsule)
        {
            bool found = FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
                mover.Center,
                mover.Rotation,
                mover.AxisLength,
                mover.ScaledRadius,
                direction,
                length,
                targetCapsule.Center,
                targetCapsule.Rotation,
                targetCapsule.AxisLength,
                targetCapsule.ScaledRadius,
                out Fixed64 distance,
                out Vector2d normal);
            return TryBuildCapsuleTargetSweepHit(
                mover,
                direction,
                target,
                targetCapsule.Center,
                targetCapsule.Rotation,
                targetCapsule.AxisLength,
                targetCapsule.ScaledRadius,
                found,
                distance,
                normal,
                out hit);
        }

        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> targetOffsets =
            GetConvexVertexOffsets(target, scratch);
        if (!FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
                mover.Center,
                mover.Rotation,
                mover.AxisLength,
                mover.ScaledRadius,
                direction,
                length,
                target.Center,
                target.ConvexRotation,
                targetOffsets,
                out Fixed64 convexDistance,
                out Vector2d convexNormal,
                out FixedPointAnchor2d targetContact))
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(targetContact),
            convexNormal,
            convexDistance);
        return true;
    }

    private static bool TrySweepConvexMoverAgainstCapsule(
        LSCollider2D mover,
        Vector2d displacement,
        Fixed64 displacementLength,
        LSCapsuleCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            hit = new Physics2DHit(target, overlap.AnchorB, -overlap.Normal, Fixed64.Zero);
            return true;
        }

        if (!TrySweepCapsuleMover(
                target,
                -displacement,
                displacementLength,
                mover,
                out Physics2DHit reverseHit))
        {
            hit = default;
            return false;
        }

        Vector2d reverseDirection = -displacement / displacementLength;
        if (!TryOffsetPoint(
                target.Center,
                reverseDirection,
                reverseHit.Distance,
                out Vector2d movedCapsuleCenter))
        {
            hit = default;
            return false;
        }

        Span<Vector2d> moverScratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> moverOffsets =
            GetConvexVertexOffsets(mover, moverScratch);
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (!FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                movedCapsuleCenter,
                target.Rotation,
                Vector2d.Forward,
                target.AxisLength,
                target.ScaledRadius,
                mover.Center,
                mover.ConvexRotation,
                moverOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out Vector2d normal,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(
                target.Center,
                capsuleContacts[contactCount - 1].Rotation,
                capsuleContacts[contactCount - 1].LocalPoint,
                capsuleContacts[contactCount - 1].LocalDisplacement),
            normal,
            reverseHit.Distance);
        return true;
    }

    private static bool TrySweepConvexMoverAgainstConvex(
        LSCollider2D mover,
        Vector2d displacement,
        Fixed64 segmentLength,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            hit = new Physics2DHit(target, overlap.AnchorB, -overlap.Normal, Fixed64.Zero);
            return true;
        }

        Vector2d direction = displacement.Normalized;
        Span<Vector2d> moverScratch = stackalloc Vector2d[4];
        Span<Vector2d> targetScratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> moverOffsets =
            GetConvexVertexOffsets(mover, moverScratch);
        ReadOnlySpan<Vector2d> targetOffsets =
            GetConvexVertexOffsets(target, targetScratch);
        if (!FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
                mover.Center,
                mover.ConvexRotation,
                moverOffsets,
                direction,
                segmentLength,
                target.Center,
                target.ConvexRotation,
                targetOffsets,
                out Fixed64 distance,
                out Vector2d normal,
                out FixedPointAnchor2d targetContact))
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(targetContact),
            normal,
            distance);
        return true;
    }

    private static bool TryBuildCapsuleTargetSweepHit(
        LSCapsuleCollider2D mover,
        Vector2d direction,
        LSCollider2D target,
        Vector2d targetCenter,
        Fixed64 targetRotation,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        bool found,
        Fixed64 distance,
        Vector2d moverToTargetNormal,
        out Physics2DHit hit)
    {
        if (!found)
        {
            hit = default;
            return false;
        }

        if (!TryOffsetPoint(
                mover.Center,
                direction,
                distance,
                out Vector2d movedCenter)
            || !FixedSegment2d.TryGetCenteredCapsulesContact(
                movedCenter,
                mover.Rotation,
                mover.AxisLength,
                mover.ScaledRadius,
                targetCenter,
                targetRotation,
                targetAxisLength,
                targetRadius,
                moverToTargetNormal,
                out FixedContactAnchors2d contact))
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(contact.SecondAnchor),
            -contact.Normal,
            distance);
        return true;
    }
}
