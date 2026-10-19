// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

internal static partial class CollisionDetection2D
{
    // The caller has validated the candidate's circle/capsule family and the
    // physical pair policy. Clearance uses exact classification, not the
    // rounded translation that the ordinary contact solver needs.
    internal static bool DoesPostureCandidatePenetrate(LSCollider2D candidate, LSCollider2D other)
    {
        if (!BoundsOverlap(candidate, other))
            return false;

        Fixed64 axisLength = candidate is LSCapsuleCollider2D capsule
            ? capsule.AxisLength : Fixed64.Zero;
        Fixed64 radius = candidate is LSCircleCollider2D circle
            ? circle.ScaledRadius : ((LSCapsuleCollider2D)candidate).ScaledRadius;

        if (other is LSCompoundCollider2D compound)
        {
            // Compound definitions contain leaves, not nested compounds.
            for (int i = 0; i < compound.PartCount; i++)
            {
                LSCollider2D part = compound.GetPartCollider(i);
                if (BoundsOverlap(candidate, part)
                    && DoesPostureCapsulePenetrate(candidate, candidate.Center, axisLength, radius, part))
                    return true;
            }
            return false;
        }

        return DoesPostureCapsulePenetrate(candidate, candidate.Center, axisLength, radius, other);
    }

    // A recovery certificate tests the proposed center without publishing a
    // transient collider pose, including polygons and all compound blockers.
    internal static bool DoesPostureCandidatePenetrate(LSCollider2D candidate, Vector2d center, LSCollider2D other)
    {
        if (other is LSCompoundCollider2D compound)
        {
            for (int i = 0; i < compound.PartCount; i++)
                if (DoesPostureCandidatePenetrate(candidate, center, compound.GetPartCollider(i))) return true;
            return false;
        }
        if (candidate is LSCircleCollider2D circle)
            return DoesPostureCapsulePenetrate(candidate, center, Fixed64.Zero, circle.ScaledRadius, other);
        if (candidate is LSCapsuleCollider2D capsule)
            return DoesPostureCapsulePenetrate(candidate, center, capsule.AxisLength, capsule.ScaledRadius, other);
        Span<Vector2d> scratch = stackalloc Vector2d[4], otherScratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertices = GetConvexVertexOffsets(candidate, scratch);
        if (other is LSCircleCollider2D otherCircle)
            return FixedSegment2d.DoesCenteredCapsulePenetrateConvex(other.Center, other.Rotation, Fixed64.Zero,
                otherCircle.ScaledRadius, center, candidate.ConvexRotation, vertices);
        if (other is LSCapsuleCollider2D otherCapsule)
            return FixedSegment2d.DoesCenteredCapsulePenetrateConvex(other.Center, other.Rotation, otherCapsule.AxisLength,
                otherCapsule.ScaledRadius, center, candidate.ConvexRotation, vertices);
        return WideConvex2dRelations.DoConvexesOverlap(center, candidate.ConvexRotation, vertices,
            other.Center, other.ConvexRotation, GetConvexVertexOffsets(other, otherScratch));
    }

    private static bool DoesPostureCapsulePenetrate(
        LSCollider2D candidate, Vector2d center, Fixed64 axisLength, Fixed64 radius, LSCollider2D other)
    {
        if (other is LSCircleCollider2D circle)
        {
            return FixedSegment2d.DoCenteredCapsulesOverlapStrict(
                center, candidate.Rotation, axisLength, radius,
                circle.Center, circle.Rotation, Fixed64.Zero, circle.ScaledRadius);
        }
        if (other is LSCapsuleCollider2D capsule)
        {
            return FixedSegment2d.DoCenteredCapsulesOverlapStrict(
                center, candidate.Rotation, axisLength, radius,
                capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius);
        }

        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertices = GetConvexVertexOffsets(other, scratch);
        return FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            center, candidate.Rotation, axisLength, radius,
            other.Center, other.ConvexRotation, vertices);
    }
}
