//=======================================================================
// QueryDetection2D.cs
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

/// <summary>
/// Deterministic pure 2D shape checks used by query services.
/// </summary>
internal static partial class QueryDetection2D
{
    internal static bool TryOverlapCircle(
        Vector2d center,
        Fixed64 radius,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        SwiftThrowHelper.ThrowIfArgument(radius < Fixed64.Zero, nameof(radius), "2D query radius cannot be negative.");

        if (collider is LSCompoundCollider2D compound)
            return TryOverlapCircleCompound(center, radius, compound, out hit);
        if (collider is LSCircleCollider2D circle)
            return TryOverlapCircleCenteredCapsule(
                center,
                radius,
                circle.Center,
                circle.Rotation,
                Fixed64.Zero,
                circle.ScaledRadius,
                circle,
                out hit);
        if (collider is LSCapsuleCollider2D capsule)
            return TryOverlapCircleCenteredCapsule(
                center,
                radius,
                capsule.Center,
                capsule.Rotation,
                capsule.AxisLength,
                capsule.ScaledRadius,
                capsule,
                out hit);
        if (collider is not IConvexVertexSource2D)
        {
            hit = default;
            return false;
        }

        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertexOffsets =
            GetConvexVertexOffsets(collider, scratch);
        if (!FixedConvex2dRelations.TryGetCircleContact(
                center,
                Fixed64.Zero,
                radius,
                collider.Center,
                collider.ConvexRotation,
                vertexOffsets,
                out _,
                out FixedPointAnchor2d contactAnchor,
                out Vector2d circleToColliderNormal,
                out Fixed64 depth,
                out _))
        {
            hit = default;
            return false;
        }

        bool containsCenter = FixedConvex2dRelations.ContainsPoint(
            center,
            collider.Center,
            collider.ConvexRotation,
            vertexOffsets);
        Fixed64 distance = Fixed64.Zero;
        Vector2d normal = -circleToColliderNormal;
        if (!containsCenter)
            distance = radius - depth;

        hit = new Physics2DHit(
            collider,
            new ContactAnchor2D(contactAnchor),
            normal,
            distance);
        return true;
    }

    private static bool TryOverlapCircleCenteredCapsule(
        Vector2d queryCenter,
        Fixed64 queryRadius,
        Vector2d targetCenter,
        Fixed64 targetRotation,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        if (!FixedSegment2d.TryGetCenteredCapsulesContact(
                queryCenter,
                Fixed64.Zero,
                Fixed64.Zero,
                queryRadius,
                targetCenter,
                targetRotation,
                targetAxisLength,
                targetRadius,
                ResolveQueryFallbackNormal(queryCenter, targetCenter),
                out FixedContactAnchors2d contact))
        {
            hit = default;
            return false;
        }

        Fixed64 distance = FixedSegment2d.GetDistanceToCenteredCapsule(
            queryCenter,
            targetCenter,
            targetRotation,
            targetAxisLength,
            targetRadius);

        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(contact.SecondAnchor),
            queryCenter == targetCenter
                ? ResolveQueryFallbackNormal(queryCenter, targetCenter)
                : -contact.Normal,
            distance);
        return true;
    }

    internal static bool TryOverlapPolygon(
        ReadOnlySpan<Vector2d> vertices,
        Vector2d center,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        return TryOverlapConvexArea(center, vertices, collider, out hit);
    }

    internal static void ValidateAabbSize(Vector2d size)
    {
        SwiftThrowHelper.ThrowIfArgument(
            size.X <= Fixed64.Zero || size.Y <= Fixed64.Zero,
            nameof(size),
            "2D AABB query size components must be greater than zero.");
    }

    internal static void ValidateConvexQueryPolygon(ReadOnlySpan<Vector2d> vertices)
    {
        SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon query must contain at least three vertices.");
        SwiftThrowHelper.ThrowIfArgument(
            !FixedConvex2dRelations.IsStrictlyConvex(vertices),
            nameof(vertices),
            "2D polygon query vertices must form a strictly convex boundary.");
    }

    internal static Vector2d CalculateAverageCenter(ReadOnlySpan<Vector2d> vertices) =>
        Vector2d.GetAverage(vertices);

    internal static bool TryRaycast(Vector2d start, Vector2d end, LSCollider2D collider, out Physics2DHit hit)
    {
        if (!Vector2d.TrySubtract(end, start, out Vector2d segment))
        {
            hit = default;
            return false;
        }

        if (segment == Vector2d.Zero || !SegmentBoundsOverlap(start, end, collider))
        {
            hit = default;
            return false;
        }

        if (!Vector2d.TryGetMagnitude(segment, out Fixed64 segmentLength))
        {
            hit = default;
            return false;
        }

        if (collider is LSCompoundCollider2D compound)
            return TryRaycastCompound(start, end, compound, out hit);

        Vector2d direction = segment.Normalized;
        if (ContainsPointExact(collider, start))
        {
            hit = new Physics2DHit(
                collider,
                start,
                ResolveQueryFallbackNormal(start, collider.Center),
                Fixed64.Zero);
            return true;
        }

        if (collider is LSCircleCollider2D circle)
            return TryRaycastCircle(start, end, direction, segmentLength, circle, out hit);
        if (collider is LSCapsuleCollider2D capsule)
            return TryRaycastCapsule(start, end, segmentLength, capsule, out hit);
        if (collider is not LSAABBoxCollider2D
            && collider is not LSPolygonCollider2D)
        {
            hit = default;
            return false;
        }

        return TryRaycastConvex(start, direction, segmentLength, collider, out hit);
    }

    internal static bool TrySweepCircle(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        if (!Vector2d.TrySubtract(end, start, out Vector2d segment))
        {
            hit = default;
            return false;
        }

        if (segment == Vector2d.Zero || !SweepBoundsOverlap(start, end, radius, collider))
        {
            hit = default;
            return false;
        }

        if (!Vector2d.TryGetMagnitude(segment, out Fixed64 segmentLength)
            || segmentLength <= Fixed64.Epsilon)
        {
            hit = default;
            return false;
        }

        if (collider is LSCompoundCollider2D compound)
            return TrySweepCircleCompound(start, end, radius, compound, out hit);

        if (TryOverlapCircle(start, radius, collider, out Physics2DHit overlapHit))
        {
            hit = new Physics2DHit(collider, start, overlapHit.Normal, Fixed64.Zero);
            return true;
        }

        Vector2d direction = segment.Normalized;
        if (collider is LSCircleCollider2D circle)
            return TrySweepCircleCircle(start, end, direction, segmentLength, radius, circle, out hit);
        if (collider is LSCapsuleCollider2D capsule)
            return TrySweepCircleCapsule(start, end, segmentLength, radius, capsule, out hit);
        if (collider is not LSAABBoxCollider2D
            && collider is not LSPolygonCollider2D)
        {
            hit = default;
            return false;
        }

        return TrySweepCircleConvex(start, direction, segmentLength, radius, collider, out hit);
    }

    internal static bool TrySweepMoverShape(
        LSCollider2D mover,
        Vector2d displacement,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        SwiftThrowHelper.ThrowIfNull(mover, nameof(mover));
        SwiftThrowHelper.ThrowIfNull(target, nameof(target));

        if (!Vector2d.TryGetMagnitude(displacement, out Fixed64 displacementLength)
            || displacementLength <= Fixed64.Epsilon)
        {
            hit = default;
            return false;
        }

        if (mover is LSCompoundCollider2D moverCompound)
            return TrySweepMoverCompound(moverCompound, displacement, target, out hit);

        if (target is LSCompoundCollider2D targetCompound)
            return TrySweepMoverAgainstCompound(mover, displacement, targetCompound, out hit);

        if (mover is LSCircleCollider2D circle)
        {
            if (!Vector2d.TryAdd(
                    circle.Center,
                    displacement,
                    out Vector2d end))
            {
                hit = default;
                return false;
            }

            return TrySweepCircle(
                circle.Center,
                end,
                circle.ScaledRadius,
                target,
                out hit);
        }
        if (mover is LSCapsuleCollider2D moverCapsule)
            return TrySweepCapsuleMover(
                moverCapsule,
                displacement,
                displacementLength,
                target,
                out hit);

        if (target is LSCircleCollider2D targetCircle)
            return TrySweepConvexMoverAgainstCircle(mover, displacement, targetCircle, out hit);
        if (target is LSCapsuleCollider2D targetCapsule)
            return TrySweepConvexMoverAgainstCapsule(
                mover,
                displacement,
                displacementLength,
                targetCapsule,
                out hit);

        return TrySweepConvexMoverAgainstConvex(
            mover,
            displacement,
            displacementLength,
            target,
            out hit);
    }

    private static bool TryRaycastCircle(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        if (!RadialSweepAdmission.TryIntersect(
                start,
                direction,
                segmentLength,
                circle.Center,
                circle.ScaledRadius,
                Fixed64.Zero,
                end,
                circle.Center,
                out Fixed64 parameter))
        {
            hit = default;
            return false;
        }

        Fixed64 distance = parameter;
        Vector2d point = distance == segmentLength
            ? end
            : start + direction * distance;
        Vector2d normal = (point - circle.Center).Normalized;
        hit = new Physics2DHit(
            circle,
            new ContactAnchor2D(
                circle.Center,
                normal * circle.ScaledRadius),
            normal,
            distance);
        return true;
    }

    private static bool TrySweepCircleCircle(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 segmentLength,
        Fixed64 radius,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        if (!RadialSweepAdmission.TryIntersect(
                start,
                direction,
                segmentLength,
                circle.Center,
                circle.ScaledRadius,
                radius,
                end,
                circle.Center,
                out Fixed64 parameter))
        {
            hit = default;
            return false;
        }

        Fixed64 distance = parameter;
        Vector2d sweptCenter = distance == segmentLength
            ? end
            : start + direction * distance;
        Vector2d normal = (sweptCenter - circle.Center).Normalized;
        hit = new Physics2DHit(
            circle,
            new ContactAnchor2D(
                circle.Center,
                normal * circle.ScaledRadius),
            normal,
            distance);
        return true;
    }

    private static bool TryRaycastCapsule(
        Vector2d start,
        Vector2d end,
        Fixed64 segmentLength,
        LSCapsuleCollider2D capsule,
        out Physics2DHit hit)
    {
        var query = new FixedSegment2d(start, end);
        if (!query.TryGetCapsuleIntersectionDistanceInterval(
                capsule.Center,
                capsule.Rotation,
                capsule.AxisLength,
                capsule.ScaledRadius,
                Fixed64.Zero,
                segmentLength,
                out Fixed64 distance,
                out _,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        Vector2d pointOnRay = query.GetPointAtDistance(distance, segmentLength);
        Vector2d normal = capsule.GetNormalFromCenteredAxis(pointOnRay);
        hit = new Physics2DHit(
            capsule,
            ContactAnchor2D.FromWorldPoint(pointOnRay),
            normal,
            distance);
        return true;
    }

    private static bool TrySweepCircleCapsule(
        Vector2d start,
        Vector2d end,
        Fixed64 segmentLength,
        Fixed64 radius,
        LSCapsuleCollider2D capsule,
        out Physics2DHit hit)
    {
        var query = new FixedSegment2d(start, end);
        if (!query.TryGetCapsuleIntersectionDistanceInterval(
                capsule.Center,
                capsule.Rotation,
                capsule.AxisLength,
                capsule.ScaledRadius,
                radius,
                segmentLength,
                out Fixed64 distance,
                out _,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        Vector2d sweptCenter = query.GetPointAtDistance(distance, segmentLength);
        Vector2d normal = capsule.GetNormalFromCenteredAxis(sweptCenter);
        bool hasPoint = TryOffsetPoint(
            sweptCenter,
            -normal,
            radius,
            out Vector2d point);

        hit = new Physics2DHit(
            capsule,
            hasPoint
                ? ContactAnchor2D.FromWorldPoint(point)
                : new ContactAnchor2D(
                    sweptCenter,
                    -normal * radius),
            normal,
            distance);
        return true;
    }

    private static bool TryOverlapCircleCompound(
        Vector2d center,
        Fixed64 radius,
        LSCompoundCollider2D compound,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TryOverlapCircle(center, radius, part, out Physics2DHit candidate))
                continue;

            TryKeepEarlierHit(candidate, ref found, ref best);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Anchor, best.Normal, best.Distance);
        return true;
    }

    private static bool TryOverlapConvexArea(
        Vector2d center,
        ReadOnlySpan<Vector2d> vertices,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        if (collider is LSCompoundCollider2D compound)
            return TryOverlapConvexAreaCompound(center, vertices, compound, out hit);

        return collider switch
        {
            LSCircleCollider2D circle =>
                TryOverlapConvexAreaCircle(center, vertices, circle, out hit),
            LSCapsuleCollider2D capsule =>
                TryOverlapConvexAreaCapsule(center, vertices, capsule, out hit),
            _ => TryOverlapConvexAreaConvex(center, vertices, collider, out hit)
        };
    }

    private static bool TryOverlapConvexAreaCompound(
        Vector2d center,
        ReadOnlySpan<Vector2d> vertices,
        LSCompoundCollider2D compound,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TryOverlapConvexArea(center, vertices, part, out Physics2DHit candidate))
                continue;

            TryKeepEarlierHit(candidate, ref found, ref best);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Anchor, best.Normal, best.Distance);
        return true;
    }

    private static bool TryOverlapConvexAreaCircle(
        Vector2d center,
        ReadOnlySpan<Vector2d> vertices,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        if (!FixedConvex2dRelations.TryGetCircleContact(
                circle.Center,
                circle.Rotation,
                circle.ScaledRadius,
                Vector2d.Zero,
                Fixed64.Zero,
                vertices,
                out _,
                out _,
                out _,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        return TryBuildCenteredCapsuleAreaHit(
            center,
            circle.Center,
            circle.Rotation,
            Fixed64.Zero,
            circle.ScaledRadius,
            circle,
            out hit);
    }

    private static bool TryOverlapConvexAreaCapsule(
        Vector2d center,
        ReadOnlySpan<Vector2d> vertices,
        LSCapsuleCollider2D capsule,
        out Physics2DHit hit)
    {
        if (!FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                capsule.Center,
                capsule.Rotation,
                capsule.AxisLength,
                capsule.ScaledRadius,
                Vector2d.Zero,
                vertices,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        return TryBuildCenteredCapsuleAreaHit(
            center,
            capsule.Center,
            capsule.Rotation,
            capsule.AxisLength,
            capsule.ScaledRadius,
            capsule,
            out hit);
    }

    private static bool TryOverlapConvexAreaConvex(
        Vector2d center,
        ReadOnlySpan<Vector2d> vertices,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        Span<Vector2d> targetScratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> targetOffsets =
            GetConvexVertexOffsets(collider, targetScratch);
        Span<FixedPointAnchor2d> queryContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> targetContacts =
            stackalloc FixedPointAnchor2d[2];
        if (!FixedConvex2dRelations.TryGetConvexContacts(
                Vector2d.Zero,
                Fixed64.Zero,
                vertices,
                collider.Center,
                collider.ConvexRotation,
                targetOffsets,
                queryContacts,
                targetContacts,
                out _,
                out _,
                out _,
                out _))
        {
            hit = default;
            return false;
        }

        return TryBuildConvexAreaHit(
            center,
            collider,
            targetOffsets,
            out hit);
    }

    private static bool TryBuildCenteredCapsuleAreaHit(
        Vector2d queryCenter,
        Vector2d targetCenter,
        Fixed64 targetRotation,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        bool containsCenter = FixedSegment2d.ContainsPointInCenteredCapsule(
            queryCenter,
            targetCenter,
            targetRotation,
            targetAxisLength,
            targetRadius,
            Fixed64.Zero);
        if (containsCenter)
        {
            hit = new Physics2DHit(
                target,
                ContactAnchor2D.FromWorldPoint(queryCenter),
                ResolveQueryFallbackNormal(queryCenter, targetCenter),
                Fixed64.Zero);
            return true;
        }

        Vector2d normal = FixedSegment2d.GetDirectionFromCenteredAxis(
            queryCenter,
            targetCenter,
            targetRotation,
            targetAxisLength);

        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(
                GetCenteredCapsuleSurfaceAnchor(
                    queryCenter,
                    targetCenter,
                    targetRotation,
                    targetAxisLength,
                    targetRadius,
                    normal)),
            normal,
            FixedSegment2d.GetDistanceToCenteredCapsule(
                queryCenter,
                targetCenter,
                targetRotation,
                targetAxisLength,
                targetRadius));
        return true;
    }

    private static FixedPointAnchor2d GetCenteredCapsuleSurfaceAnchor(
        Vector2d point,
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d worldNormal)
    {
        // A unit rotation preserves the admitted normalized direction.
        _ = Vector2d.TryRotate(
            worldNormal,
            -rotation,
            out Vector2d localNormal);
        return FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            rotation,
            Vector2d.Forward,
            axisLength,
            radius,
            localNormal.Normalized);
    }

    private static bool TryBuildConvexAreaHit(
        Vector2d queryCenter,
        LSCollider2D target,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Physics2DHit hit)
    {
        if (FixedConvex2dRelations.ContainsPoint(
                queryCenter,
                target.Center,
                target.ConvexRotation,
                targetVertexOffsets))
        {
            hit = new Physics2DHit(
                target,
                ContactAnchor2D.FromWorldPoint(queryCenter),
                ResolveQueryFallbackNormal(queryCenter, target.Center),
                Fixed64.Zero);
            return true;
        }

        FixedPointAnchor2d targetAnchor =
            FixedConvex2dRelations.GetClosestPointAnchor(
                queryCenter,
                target.Center,
                target.ConvexRotation,
                targetVertexOffsets);

        var anchor = new ContactAnchor2D(targetAnchor);
        if (!anchor.TryGetOffsetFrom(queryCenter, out Vector2d centerToTarget)
            || !Vector2d.TryGetMagnitude(centerToTarget, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d normal = -centerToTarget / distance;
        hit = new Physics2DHit(target, anchor, normal, distance);
        return true;
    }

    private static bool TryRaycastConvex(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertexOffsets =
            GetConvexVertexOffsets(collider, scratch);
        if (!FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(
                start,
                direction,
                segmentLength,
                collider.Center,
                collider.ConvexRotation,
                vertexOffsets,
                out Fixed64 distance,
                out Vector2d normal,
                out FixedPointAnchor2d contactAnchor))
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(
            collider,
            new ContactAnchor2D(contactAnchor),
            normal,
            distance);
        return true;
    }

    private static bool TrySweepCircleConvex(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Fixed64 radius,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertexOffsets =
            GetConvexVertexOffsets(collider, scratch);
        if (!FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
                start,
                radius,
                direction,
                segmentLength,
                collider.Center,
                collider.ConvexRotation,
                vertexOffsets,
                out Fixed64 distance,
                out Vector2d normal,
                out FixedPointAnchor2d contactAnchor))
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(
            collider,
            new ContactAnchor2D(contactAnchor),
            normal,
            distance);
        return true;
    }

    private static bool TrySweepConvexMoverAgainstCircle(
        LSCollider2D mover,
        Vector2d displacement,
        LSCircleCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            Vector2d normal = -overlap.Normal;
            hit = new Physics2DHit(
                target,
                overlap.AnchorB,
                normal,
                Fixed64.Zero);
            return true;
        }

        if (!TrySweepCircle(
                target.Center,
                target.Center - displacement,
                target.ScaledRadius,
                mover,
                out Physics2DHit reverseHit))
        {
            hit = default;
            return false;
        }

        Vector2d hitNormal = -reverseHit.Normal;
        hit = new Physics2DHit(
            target,
            new ContactAnchor2D(
                target.Center,
                hitNormal * target.ScaledRadius),
            hitNormal,
            reverseHit.Distance);
        return true;
    }

}
