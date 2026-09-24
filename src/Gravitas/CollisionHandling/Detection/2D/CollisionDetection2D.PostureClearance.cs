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
                    && DoesPostureCapsulePenetrate(candidate, axisLength, radius, part))
                    return true;
            }
            return false;
        }

        return DoesPostureCapsulePenetrate(candidate, axisLength, radius, other);
    }

    private static bool DoesPostureCapsulePenetrate(
        LSCollider2D candidate, Fixed64 axisLength, Fixed64 radius, LSCollider2D other)
    {
        if (other is LSCircleCollider2D circle)
        {
            return FixedSegment2d.DoCenteredCapsulesOverlapStrict(
                candidate.Center, candidate.Rotation, axisLength, radius,
                circle.Center, circle.Rotation, Fixed64.Zero, circle.ScaledRadius);
        }
        if (other is LSCapsuleCollider2D capsule)
        {
            return FixedSegment2d.DoCenteredCapsulesOverlapStrict(
                candidate.Center, candidate.Rotation, axisLength, radius,
                capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius);
        }

        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertices = GetConvexVertexOffsets(other, scratch);
        return FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            candidate.Center, candidate.Rotation, axisLength, radius,
            other.Center, other.ConvexRotation, vertices);
    }
}
