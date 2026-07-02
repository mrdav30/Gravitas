//=======================================================================
// QueryDetection2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Deterministic pure 2D shape checks used by query services.
/// </summary>
internal static class QueryDetection2D
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

        Vector2d closest = collider.GetClosestPoint(center);
        bool containsCenter = collider.ContainsPoint(center);
        Vector2d toCenter = center - closest;
        Fixed64 distanceSquared = containsCenter ? Fixed64.Zero : toCenter.MagnitudeSquared;
        if (distanceSquared > radius * radius)
        {
            hit = default;
            return false;
        }

        Fixed64 distance = distanceSquared > Fixed64.Zero ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector2d normal = distance > Fixed64.Zero
            ? toCenter / distance
            : ResolveQueryFallbackNormal(center, collider.Center);
        hit = new Physics2DHit(collider, closest, normal, distance);
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

        int sign = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d a = vertices[i];
            Vector2d b = vertices[(i + 1) % vertices.Length];
            Vector2d c = vertices[(i + 2) % vertices.Length];
            Fixed64 cross = Vector2d.CrossProduct(b - a, c - b);
            SwiftThrowHelper.ThrowIfArgument(cross.Abs() <= Fixed64.Epsilon, nameof(vertices), "2D polygon query vertices must not be collinear.");

            int currentSign = cross > Fixed64.Zero ? 1 : -1;
            if (sign == 0)
            {
                sign = currentSign;
                continue;
            }

            SwiftThrowHelper.ThrowIfArgument(currentSign != sign, nameof(vertices), "2D polygon query must be convex.");
        }
    }

    internal static Vector2d CalculateAverageCenter(ReadOnlySpan<Vector2d> vertices)
    {
        Vector2d center = Vector2d.Zero;
        for (int i = 0; i < vertices.Length; i++)
            center += vertices[i];

        return center / (Fixed64)vertices.Length;
    }

    internal static bool TryRaycast(Vector2d start, Vector2d end, LSCollider2D collider, out Physics2DHit hit)
    {
        Vector2d segment = end - start;
        Fixed64 segmentLengthSquared = segment.MagnitudeSquared;
        if (segmentLengthSquared == Fixed64.Zero || !SegmentBoundsOverlap(start, end, collider))
        {
            hit = default;
            return false;
        }

        if (collider is LSCompoundCollider2D compound)
            return TryRaycastCompound(start, end, compound, out hit);

        Fixed64 segmentLength = FixedMath.Sqrt(segmentLengthSquared);
        Vector2d direction = segment / segmentLength;
        if (collider.ContainsPoint(start))
        {
            hit = new Physics2DHit(
                collider,
                start,
                ResolveQueryFallbackNormal(start, collider.Center),
                Fixed64.Zero);
            return true;
        }

        if (collider is LSCircleCollider2D circle)
            return TryRaycastCircle(start, direction, segmentLength, circle, out hit);
        if (collider is LSCapsuleCollider2D capsule)
            return TryRaycastCapsule(start, direction, segmentLength, capsule, out hit);

        return TryRaycastConvex(start, segment, direction, segmentLength, collider, out hit);
    }

    internal static bool TrySweepCircle(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        Vector2d segment = end - start;
        Fixed64 segmentLengthSquared = segment.MagnitudeSquared;
        if (segmentLengthSquared <= Fixed64.Epsilon || !SweepBoundsOverlap(start, end, radius, collider))
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

        Fixed64 segmentLength = FixedMath.Sqrt(segmentLengthSquared);
        Vector2d direction = segment / segmentLength;
        if (collider is LSCircleCollider2D circle)
            return TrySweepCircleCircle(start, direction, segmentLength, radius, circle, out hit);
        if (collider is LSCapsuleCollider2D capsule)
            return TrySweepCircleCapsule(start, direction, segmentLength, radius, capsule, out hit);

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

        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
        {
            hit = default;
            return false;
        }

        if (mover is LSCompoundCollider2D moverCompound)
            return TrySweepMoverCompound(moverCompound, displacement, target, out hit);

        if (target is LSCompoundCollider2D targetCompound)
            return TrySweepMoverAgainstCompound(mover, displacement, targetCompound, out hit);

        if (mover is LSCircleCollider2D circle)
            return TrySweepCircle(circle.Center, circle.Center + displacement, circle.ScaledRadius, target, out hit);
        if (mover is LSCapsuleCollider2D moverCapsule)
            return TrySweepCapsuleMover(moverCapsule, displacement, target, out hit);

        if (target is LSCircleCollider2D targetCircle)
            return TrySweepConvexMoverAgainstCircle(mover, displacement, targetCircle, out hit);
        if (target is LSCapsuleCollider2D targetCapsule)
            return TrySweepConvexMoverAgainstCapsule(mover, displacement, targetCapsule, out hit);

        return TrySweepConvexMoverAgainstConvex(mover, displacement, target, out hit);
    }

    private static bool TryRaycastCircle(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        Vector2d originFromCenter = start - circle.Center;
        Fixed64 scaledRadius = circle.ScaledRadius;
        Fixed64 c = originFromCenter.MagnitudeSquared - scaledRadius * scaledRadius;
        Fixed64 b = Vector2d.Dot(originFromCenter, direction);
        if (c > Fixed64.Zero && b > Fixed64.Zero)
        {
            hit = default;
            return false;
        }

        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
        {
            hit = default;
            return false;
        }

        Fixed64 distance = -b - FixedMath.Sqrt(discriminant);
        if (distance < Fixed64.Zero)
            distance = Fixed64.Zero;
        if (distance > segmentLength)
        {
            hit = default;
            return false;
        }

        Vector2d point = start + direction * distance;
        Vector2d normal = point == circle.Center
            ? ResolveQueryFallbackNormal(point, circle.Center)
            : (point - circle.Center).Normalized;
        hit = new Physics2DHit(circle, point, normal, distance);
        return true;
    }

    private static bool TrySweepCircleCircle(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Fixed64 radius,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        Fixed64 combinedRadius = radius + circle.ScaledRadius;
        if (!TryRaycastCircleDistance(start, direction, segmentLength, circle.Center, combinedRadius, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d sweptCenter = start + direction * distance;
        Vector2d normal = sweptCenter == circle.Center
            ? ResolveQueryFallbackNormal(sweptCenter, circle.Center)
            : (sweptCenter - circle.Center).Normalized;
        Vector2d point = circle.Center + normal * circle.ScaledRadius;
        hit = new Physics2DHit(circle, point, normal, distance);
        return true;
    }

    private static bool TryRaycastCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCapsuleCollider2D capsule,
        out Physics2DHit hit)
    {
        if (!TrySweepPointAgainstSegmentCapsule(
                start,
                direction,
                segmentLength,
                capsule.SegmentStart,
                capsule.SegmentEnd,
                capsule.ScaledRadius,
                out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d pointOnRay = start + direction * distance;
        Vector2d segmentPoint = PlanarSegmentGeometry.ClosestPoint(pointOnRay, capsule.SegmentStart, capsule.SegmentEnd);
        Vector2d normal = ResolveCapsuleNormal(pointOnRay, segmentPoint);
        hit = new Physics2DHit(
            capsule,
            segmentPoint + normal * capsule.ScaledRadius,
            normal,
            distance);
        return true;
    }

    private static bool TrySweepCircleCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Fixed64 radius,
        LSCapsuleCollider2D capsule,
        out Physics2DHit hit)
    {
        Fixed64 combinedRadius = radius + capsule.ScaledRadius;
        if (!TrySweepPointAgainstSegmentCapsule(
                start,
                direction,
                segmentLength,
                capsule.SegmentStart,
                capsule.SegmentEnd,
                combinedRadius,
                out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d sweptCenter = start + direction * distance;
        Vector2d segmentPoint = PlanarSegmentGeometry.ClosestPoint(sweptCenter, capsule.SegmentStart, capsule.SegmentEnd);
        Vector2d normal = ResolveCapsuleNormal(sweptCenter, segmentPoint);
        hit = new Physics2DHit(
            capsule,
            segmentPoint + normal * capsule.ScaledRadius,
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

            if (!found || Physics2DHitSorter.ComesBefore(candidate, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Point, best.Normal, best.Distance);
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

        bool overlaps = collider switch
        {
            LSCircleCollider2D circle => TryOverlapConvexAreaCircle(vertices, circle),
            LSCapsuleCollider2D capsule => TryOverlapConvexAreaCapsule(vertices, capsule),
            _ => TryOverlapConvexAreaConvex(vertices, collider)
        };
        if (!overlaps)
        {
            hit = default;
            return false;
        }

        hit = BuildAreaOverlapHit(center, collider);
        return true;
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

            if (!found || Physics2DHitSorter.ComesBefore(candidate, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Point, best.Normal, best.Distance);
        return true;
    }

    private static bool TryOverlapConvexAreaCircle(ReadOnlySpan<Vector2d> vertices, LSCircleCollider2D circle)
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d edge = vertices[(i + 1) % vertices.Length] - vertices[i];
            if (!TryTestAreaCircleAxis(edge.RightHandNormal, vertices, circle))
                return false;
        }

        Vector2d closest = ClosestPointOnConvexArea(circle.Center, vertices);
        Vector2d circleAxis = closest - circle.Center;
        return circleAxis.MagnitudeSquared <= Fixed64.Epsilon
            || TryTestAreaCircleAxis(circleAxis, vertices, circle);
    }

    private static bool TryOverlapConvexAreaCapsule(ReadOnlySpan<Vector2d> vertices, LSCapsuleCollider2D capsule)
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d edge = vertices[(i + 1) % vertices.Length] - vertices[i];
            if (!TryTestAreaCapsuleAxis(edge.RightHandNormal, vertices, capsule))
                return false;
        }

        Vector2d closestAxis = FindCapsuleAreaClosestAxis(capsule, vertices);
        return closestAxis.MagnitudeSquared <= Fixed64.Epsilon
            || TryTestAreaCapsuleAxis(closestAxis, vertices, capsule);
    }

    private static bool TryOverlapConvexAreaConvex(ReadOnlySpan<Vector2d> vertices, LSCollider2D collider)
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d edge = vertices[(i + 1) % vertices.Length] - vertices[i];
            if (!TryTestAreaConvexAxis(edge.RightHandNormal, vertices, collider))
                return false;
        }

        for (int i = 0; i < collider.VertexCount; i++)
        {
            Vector2d edge = collider.GetVertexUnchecked((i + 1) % collider.VertexCount) - collider.GetVertexUnchecked(i);
            if (!TryTestAreaConvexAxis(edge.RightHandNormal, vertices, collider))
                return false;
        }

        return true;
    }

    private static Physics2DHit BuildAreaOverlapHit(Vector2d center, LSCollider2D collider)
    {
        Vector2d point = collider.ContainsPoint(center)
            ? center
            : collider.GetClosestPoint(center);
        Vector2d delta = center - point;
        Fixed64 distanceSquared = delta.MagnitudeSquared;
        Fixed64 distance = distanceSquared > Fixed64.Zero ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector2d normal = distance > Fixed64.Zero
            ? delta / distance
            : ResolveQueryFallbackNormal(center, collider.Center);
        return new Physics2DHit(collider, point, normal, distance);
    }

    private static bool TryTestAreaCircleAxis(Vector2d axis, ReadOnlySpan<Vector2d> vertices, LSCircleCollider2D circle)
    {
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        axis = axis.Normalized;
        ProjectVertices(vertices, axis, out Fixed64 areaMin, out Fixed64 areaMax);
        Fixed64 circleCenter = Vector2d.Dot(circle.Center, axis);
        Fixed64 circleMin = circleCenter - circle.ScaledRadius;
        Fixed64 circleMax = circleCenter + circle.ScaledRadius;
        return areaMax >= circleMin && circleMax >= areaMin;
    }

    private static bool TryTestAreaCapsuleAxis(Vector2d axis, ReadOnlySpan<Vector2d> vertices, LSCapsuleCollider2D capsule)
    {
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        axis = axis.Normalized;
        ProjectVertices(vertices, axis, out Fixed64 areaMin, out Fixed64 areaMax);
        ProjectCapsule(capsule, axis, out Fixed64 capsuleMin, out Fixed64 capsuleMax);
        return areaMax >= capsuleMin && capsuleMax >= areaMin;
    }

    private static bool TryTestAreaConvexAxis(Vector2d axis, ReadOnlySpan<Vector2d> vertices, LSCollider2D collider)
    {
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        axis = axis.Normalized;
        ProjectVertices(vertices, axis, out Fixed64 areaMin, out Fixed64 areaMax);
        ProjectConvex(collider, axis, out Fixed64 colliderMin, out Fixed64 colliderMax);
        return areaMax >= colliderMin && colliderMax >= areaMin;
    }

    private static void ProjectVertices(ReadOnlySpan<Vector2d> vertices, Vector2d axis, out Fixed64 min, out Fixed64 max)
    {
        min = Vector2d.Dot(vertices[0], axis);
        max = min;
        for (int i = 1; i < vertices.Length; i++)
        {
            Fixed64 projection = Vector2d.Dot(vertices[i], axis);
            if (projection < min)
                min = projection;
            else if (projection > max)
                max = projection;
        }
    }

    private static Vector2d ClosestPointOnConvexArea(Vector2d point, ReadOnlySpan<Vector2d> vertices)
    {
        if (ContainsPointInConvexArea(point, vertices))
            return point;

        Fixed64 bestDistance = Fixed64.MaxValue;
        Vector2d bestPoint = vertices[0];
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d candidate = PlanarSegmentGeometry.ClosestPoint(point, vertices[i], vertices[(i + 1) % vertices.Length]);
            Fixed64 distance = Vector2d.DistanceSquared(point, candidate);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bestPoint = candidate;
        }

        return bestPoint;
    }

    private static bool ContainsPointInConvexArea(Vector2d point, ReadOnlySpan<Vector2d> vertices)
    {
        bool hasPositive = false;
        bool hasNegative = false;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d a = vertices[i];
            Vector2d b = vertices[(i + 1) % vertices.Length];
            Fixed64 cross = Vector2d.CrossProduct(b - a, point - a);
            if (cross > Fixed64.Epsilon)
                hasPositive = true;
            else if (cross < -Fixed64.Epsilon)
                hasNegative = true;

            if (hasPositive && hasNegative)
                return false;
        }

        return true;
    }

    private static bool TryRaycastCompound(
        Vector2d start,
        Vector2d end,
        LSCompoundCollider2D compound,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TryRaycast(start, end, part, out Physics2DHit candidate))
                continue;

            if (!found || Physics2DHitSorter.ComesBefore(candidate, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Point, best.Normal, best.Distance);
        return true;
    }

    private static bool TrySweepCircleCompound(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        LSCompoundCollider2D compound,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TrySweepCircle(start, end, radius, part, out Physics2DHit candidate))
                continue;

            if (!found || Physics2DHitSorter.ComesBefore(candidate, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Point, best.Normal, best.Distance);
        return true;
    }

    private static bool TrySweepMoverCompound(
        LSCompoundCollider2D mover,
        Vector2d displacement,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < mover.PartCount; i++)
        {
            LSCollider2D part = mover.GetPartCollider(i);
            if (!TrySweepMoverShape(part, displacement, target, out Physics2DHit candidate))
                continue;

            if (!found || Physics2DHitSorter.ComesBefore(candidate, best))
            {
                best = candidate;
                found = true;
            }
        }

        hit = best;
        return found;
    }

    private static bool TrySweepMoverAgainstCompound(
        LSCollider2D mover,
        Vector2d displacement,
        LSCompoundCollider2D target,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < target.PartCount; i++)
        {
            LSCollider2D part = target.GetPartCollider(i);
            if (!TrySweepMoverShape(mover, displacement, part, out Physics2DHit candidate))
                continue;

            if (!found || candidate.Distance < best.Distance)
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(target, best.Point, best.Normal, best.Distance);
        return true;
    }

    private static bool TryRaycastConvex(
        Vector2d start,
        Vector2d segment,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCollider2D collider,
        out Physics2DHit hit)
    {
        bool found = false;
        Fixed64 bestT = Fixed64.MaxValue;
        Vector2d bestPoint = Vector2d.Zero;
        Vector2d bestNormal = Vector2d.Right;

        for (int i = 0; i < collider.VertexCount; i++)
        {
            Vector2d a = collider.GetVertexUnchecked(i);
            Vector2d b = collider.GetVertexUnchecked((i + 1) % collider.VertexCount);
            if (!TryIntersectSegments(start, segment, a, b - a, out Fixed64 t))
                continue;

            if (found && t >= bestT)
                continue;

            Vector2d edge = b - a;
            Vector2d normal = edge.LeftHandNormal;
            if (normal.MagnitudeSquared > Fixed64.Epsilon)
                normal = normal.Normalized;
            if (Vector2d.Dot(normal, direction) > Fixed64.Zero)
                normal = -normal;

            found = true;
            bestT = t;
            bestPoint = start + segment * t;
            bestNormal = normal.MagnitudeSquared > Fixed64.Epsilon ? normal : ResolveQueryFallbackNormal(bestPoint, collider.Center);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(collider, bestPoint, bestNormal, segmentLength * bestT);
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
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        Vector2d bestPoint = Vector2d.Zero;
        Vector2d bestNormal = Vector2d.Right;

        int vertexCount = collider.VertexCount;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d a = collider.GetVertexUnchecked(i);
            Vector2d b = collider.GetVertexUnchecked((i + 1) % vertexCount);
            if (TrySweepCircleEdge(
                    start,
                    direction,
                    segmentLength,
                    radius,
                    collider.Center,
                    a,
                    b,
                    out Fixed64 edgeDistance,
                    out Vector2d edgePoint,
                    out Vector2d edgeNormal)
                && (!found || edgeDistance < bestDistance))
            {
                found = true;
                bestDistance = edgeDistance;
                bestPoint = edgePoint;
                bestNormal = edgeNormal;
            }

            if (TrySweepCircleVertex(
                    start,
                    direction,
                    segmentLength,
                    radius,
                    a,
                    out Fixed64 vertexDistance,
                    out Vector2d vertexNormal)
                && (!found || vertexDistance < bestDistance))
            {
                found = true;
                bestDistance = vertexDistance;
                bestPoint = a;
                bestNormal = vertexNormal;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(collider, bestPoint, bestNormal, bestDistance);
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
            hit = new Physics2DHit(target, target.Center + normal * target.ScaledRadius, normal, Fixed64.Zero);
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
            target.Center + hitNormal * target.ScaledRadius,
            hitNormal,
            reverseHit.Distance);
        return true;
    }

    private static bool TrySweepCapsuleMover(
        LSCapsuleCollider2D mover,
        Vector2d displacement,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            hit = new Physics2DHit(target, overlap.PointB, -overlap.Normal, Fixed64.Zero);
            return true;
        }

        Fixed64 length = displacement.Magnitude;
        if (length <= Fixed64.Epsilon)
        {
            hit = default;
            return false;
        }

        Vector2d direction = displacement / length;
        bool found = false;
        Physics2DHit best = default;

        TryKeepCapsuleMoverCapHit(mover.SegmentStart, mover.ScaledRadius, displacement, target, ref found, ref best);
        TryKeepCapsuleMoverCapHit(mover.SegmentEnd, mover.ScaledRadius, displacement, target, ref found, ref best);

        if (target is LSCircleCollider2D circle)
        {
            TryKeepReversePointCapsuleHit(circle.Center, circle.ScaledRadius, mover, direction, length, target, ref found, ref best);
        }
        else if (target is LSCapsuleCollider2D targetCapsule)
        {
            TryKeepReversePointCapsuleHit(targetCapsule.SegmentStart, targetCapsule.ScaledRadius, mover, direction, length, target, ref found, ref best);
            TryKeepReversePointCapsuleHit(targetCapsule.SegmentEnd, targetCapsule.ScaledRadius, mover, direction, length, target, ref found, ref best);
        }
        else
        {
            TrySweepCapsuleSegmentAgainstConvexEdges(mover, direction, length, target, ref found, ref best);
            for (int i = 0; i < target.VertexCount; i++)
                TryKeepReversePointCapsuleHit(target.GetVertexUnchecked(i), Fixed64.Zero, mover, direction, length, target, ref found, ref best);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = best;
        return true;
    }

    private static bool TrySweepConvexMoverAgainstCapsule(
        LSCollider2D mover,
        Vector2d displacement,
        LSCapsuleCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            hit = new Physics2DHit(target, overlap.PointB, -overlap.Normal, Fixed64.Zero);
            return true;
        }

        if (!TrySweepCapsuleMover(target, -displacement, mover, out Physics2DHit reverseHit))
        {
            hit = default;
            return false;
        }

        Vector2d normal = -reverseHit.Normal;
        hit = new Physics2DHit(target, target.GetSupportPoint(normal), normal, reverseHit.Distance);
        return true;
    }

    private static bool TrySweepConvexMoverAgainstConvex(
        LSCollider2D mover,
        Vector2d displacement,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        if (CollisionDetection2D.TryCollide(mover, target, out Contact2D overlap))
        {
            hit = new Physics2DHit(target, overlap.PointB, -overlap.Normal, Fixed64.Zero);
            return true;
        }

        Fixed64 segmentLength = displacement.Magnitude;
        if (segmentLength <= Fixed64.Epsilon)
        {
            hit = default;
            return false;
        }

        Fixed64 entryTime = Fixed64.Zero;
        Fixed64 exitTime = Fixed64.One;
        Vector2d entryNormal = ResolveQueryFallbackNormal(mover.Center, target.Center);

        if (!TrySweepConvexAxes(mover, mover, target, displacement, ref entryTime, ref exitTime, ref entryNormal)
            || !TrySweepConvexAxes(target, mover, target, displacement, ref entryTime, ref exitTime, ref entryNormal)
            || entryTime > Fixed64.One
            || exitTime < Fixed64.Zero)
        {
            hit = default;
            return false;
        }

        if (entryTime < Fixed64.Zero)
            entryTime = Fixed64.Zero;

        Vector2d point = target.GetSupportPoint(entryNormal);
        hit = new Physics2DHit(target, point, entryNormal, segmentLength * entryTime);
        return true;
    }

    private static void TryKeepCapsuleMoverCapHit(
        Vector2d capCenter,
        Fixed64 radius,
        Vector2d displacement,
        LSCollider2D target,
        ref bool found,
        ref Physics2DHit best)
    {
        if (!TrySweepCircle(capCenter, capCenter + displacement, radius, target, out Physics2DHit candidate))
            return;

        TryKeepEarlierHit(candidate, ref found, ref best);
    }

    private static void TryKeepReversePointCapsuleHit(
        Vector2d targetPoint,
        Fixed64 targetRadius,
        LSCapsuleCollider2D mover,
        Vector2d direction,
        Fixed64 length,
        LSCollider2D target,
        ref bool found,
        ref Physics2DHit best)
    {
        Fixed64 combinedRadius = mover.ScaledRadius + targetRadius;
        if (!TrySweepPointAgainstSegmentCapsule(
                targetPoint,
                -direction,
                length,
                mover.SegmentStart,
                mover.SegmentEnd,
                combinedRadius,
                out Fixed64 distance))
        {
            return;
        }

        Vector2d movedSegmentStart = mover.SegmentStart + direction * distance;
        Vector2d movedSegmentEnd = mover.SegmentEnd + direction * distance;
        Vector2d segmentPoint = PlanarSegmentGeometry.ClosestPoint(targetPoint, movedSegmentStart, movedSegmentEnd);
        Vector2d normal = ResolveCapsuleNormal(segmentPoint, targetPoint);
        Vector2d point = targetRadius > Fixed64.Zero
            ? targetPoint + normal * targetRadius
            : targetPoint;
        TryKeepEarlierHit(new Physics2DHit(target, point, normal, distance), ref found, ref best);
    }

    private static void TrySweepCapsuleSegmentAgainstConvexEdges(
        LSCapsuleCollider2D mover,
        Vector2d direction,
        Fixed64 length,
        LSCollider2D target,
        ref bool found,
        ref Physics2DHit best)
    {
        Vector2d segmentStart = mover.SegmentStart;
        Vector2d segmentEnd = mover.SegmentEnd;
        for (int i = 0; i < target.VertexCount; i++)
        {
            Vector2d edgeStart = target.GetVertexUnchecked(i);
            Vector2d edgeEnd = target.GetVertexUnchecked((i + 1) % target.VertexCount);
            Vector2d edge = edgeEnd - edgeStart;
            Fixed64 edgeLengthSquared = edge.MagnitudeSquared;
            if (edgeLengthSquared <= Fixed64.Epsilon)
                continue;

            Vector2d normal = ResolveOutwardEdgeNormal(edgeStart, edge, target.Center);
            Fixed64 normalVelocity = Vector2d.Dot(direction, normal);
            if (normalVelocity >= -Fixed64.Epsilon)
                continue;

            Fixed64 offsetStart = Vector2d.Dot(segmentStart - edgeStart, normal);
            Fixed64 offsetEnd = Vector2d.Dot(segmentEnd - edgeStart, normal);
            Fixed64 nearestOffset = FixedMath.Min(offsetStart, offsetEnd);
            if (nearestOffset <= mover.ScaledRadius)
                continue;

            Fixed64 distance = (mover.ScaledRadius - nearestOffset) / normalVelocity;
            if (distance < Fixed64.Zero || distance > length)
                continue;

            Vector2d movedStart = segmentStart + direction * distance;
            Vector2d movedEnd = segmentEnd + direction * distance;
            Vector2d tangent = edge / FixedMath.Sqrt(edgeLengthSquared);
            ProjectSegmentWithRadius(movedStart, movedEnd, tangent, mover.ScaledRadius, out Fixed64 moverMin, out Fixed64 moverMax);
            Fixed64 edgeA = Vector2d.Dot(edgeStart, tangent);
            Fixed64 edgeB = Vector2d.Dot(edgeEnd, tangent);
            Fixed64 edgeMin = FixedMath.Min(edgeA, edgeB);
            Fixed64 edgeMax = FixedMath.Max(edgeA, edgeB);
            if (moverMax < edgeMin || edgeMax < moverMin)
                continue;

            ClosestPointsOnSegments(movedStart, movedEnd, edgeStart, edgeEnd, out Vector2d moverPoint, out Vector2d edgePoint);
            Vector2d point = Vector2d.DistanceSquared(moverPoint, edgePoint) > Fixed64.Epsilon
                ? moverPoint - normal * mover.ScaledRadius
                : edgePoint;
            TryKeepEarlierHit(new Physics2DHit(target, point, normal, distance), ref found, ref best);
        }
    }

    private static bool TrySweepCircleEdge(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Fixed64 radius,
        Vector2d colliderCenter,
        Vector2d edgeStart,
        Vector2d edgeEnd,
        out Fixed64 distance,
        out Vector2d point,
        out Vector2d normal)
    {
        Vector2d edge = edgeEnd - edgeStart;
        Fixed64 edgeLengthSquared = edge.MagnitudeSquared;
        if (edgeLengthSquared <= Fixed64.Epsilon)
        {
            distance = default;
            point = default;
            normal = default;
            return false;
        }

        normal = ResolveOutwardEdgeNormal(edgeStart, edge, colliderCenter);
        Fixed64 startOffset = Vector2d.Dot(start - edgeStart, normal);
        Fixed64 directionOffset = Vector2d.Dot(direction, normal);
        if (startOffset <= radius || directionOffset >= -Fixed64.Epsilon)
        {
            distance = default;
            point = default;
            return false;
        }

        distance = (radius - startOffset) / directionOffset;
        if (distance < Fixed64.Zero || distance > segmentLength)
        {
            point = default;
            return false;
        }

        Vector2d sweptCenter = start + direction * distance;
        point = sweptCenter - normal * radius;
        Fixed64 edgeT = Vector2d.Dot(point - edgeStart, edge) / edgeLengthSquared;
        if (edgeT < Fixed64.Zero || edgeT > Fixed64.One)
        {
            distance = default;
            point = default;
            normal = default;
            return false;
        }

        return true;
    }

    private static bool TrySweepCircleVertex(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Fixed64 radius,
        Vector2d vertex,
        out Fixed64 distance,
        out Vector2d normal)
    {
        if (!TryRaycastCircleDistance(start, direction, segmentLength, vertex, radius, out distance))
        {
            normal = default;
            return false;
        }

        Vector2d sweptCenter = start + direction * distance;
        normal = sweptCenter == vertex
            ? ResolveQueryFallbackNormal(sweptCenter, vertex)
            : (sweptCenter - vertex).Normalized;
        return true;
    }

    private static bool TryRaycastCircleDistance(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Vector2d circleCenter,
        Fixed64 radius,
        out Fixed64 distance)
    {
        Vector2d originFromCenter = start - circleCenter;
        Fixed64 c = originFromCenter.MagnitudeSquared - radius * radius;
        Fixed64 b = Vector2d.Dot(originFromCenter, direction);
        if (c > Fixed64.Zero && b > Fixed64.Zero)
        {
            distance = default;
            return false;
        }

        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
        {
            distance = default;
            return false;
        }

        distance = -b - FixedMath.Sqrt(discriminant);
        if (distance < Fixed64.Zero)
            distance = Fixed64.Zero;
        return distance <= segmentLength;
    }

    private static bool TrySweepPointAgainstSegmentCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Vector2d segmentStart,
        Vector2d segmentEnd,
        Fixed64 radius,
        out Fixed64 distance)
    {
        Vector2d closestAtStart = PlanarSegmentGeometry.ClosestPoint(start, segmentStart, segmentEnd);
        if (Vector2d.DistanceSquared(start, closestAtStart) <= radius * radius)
        {
            distance = Fixed64.Zero;
            return true;
        }

        Vector2d segment = segmentEnd - segmentStart;
        Fixed64 segmentLengthSquared = segment.MagnitudeSquared;
        if (segmentLengthSquared <= Fixed64.Epsilon)
            return TryRaycastCircleDistance(start, direction, segmentLength, segmentStart, radius, out distance);

        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        Fixed64 coreLength = FixedMath.Sqrt(segmentLengthSquared);
        Vector2d axis = segment / coreLength;
        Vector2d normal = axis.RightHandNormal;
        TryKeepSegmentCapsuleSideDistance(start, direction, segmentLength, segmentStart, axis, normal, coreLength, radius, ref found, ref bestDistance);
        TryKeepSegmentCapsuleSideDistance(start, direction, segmentLength, segmentStart, axis, -normal, coreLength, radius, ref found, ref bestDistance);

        if (TryRaycastCircleDistance(start, direction, segmentLength, segmentStart, radius, out Fixed64 startCapDistance))
            KeepEarlierDistance(startCapDistance, ref found, ref bestDistance);
        if (TryRaycastCircleDistance(start, direction, segmentLength, segmentEnd, radius, out Fixed64 endCapDistance))
            KeepEarlierDistance(endCapDistance, ref found, ref bestDistance);

        distance = bestDistance;
        return found;
    }

    private static void TryKeepSegmentCapsuleSideDistance(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        Vector2d segmentStart,
        Vector2d axis,
        Vector2d normal,
        Fixed64 coreLength,
        Fixed64 radius,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        Fixed64 sideOffset = Vector2d.Dot(start - segmentStart, normal);
        Fixed64 sideVelocity = Vector2d.Dot(direction, normal);
        if (sideOffset <= radius || sideVelocity >= -Fixed64.Epsilon)
            return;

        Fixed64 distance = (radius - sideOffset) / sideVelocity;
        if (distance < Fixed64.Zero || distance > segmentLength)
            return;

        Vector2d point = start + direction * distance;
        Fixed64 along = Vector2d.Dot(point - segmentStart, axis);
        if (along < Fixed64.Zero || along > coreLength)
            return;

        KeepEarlierDistance(distance, ref found, ref bestDistance);
    }

    private static bool TrySweepConvexAxes(
        LSCollider2D axisSource,
        LSCollider2D mover,
        LSCollider2D target,
        Vector2d displacement,
        ref Fixed64 entryTime,
        ref Fixed64 exitTime,
        ref Vector2d entryNormal)
    {
        int vertexCount = axisSource.VertexCount;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d edge = axisSource.GetVertexUnchecked((i + 1) % vertexCount) - axisSource.GetVertexUnchecked(i);
            Vector2d axis = edge.RightHandNormal;
            if (axis.MagnitudeSquared <= Fixed64.Epsilon)
                continue;

            axis = axis.Normalized;
            ProjectConvex(mover, axis, out Fixed64 moverMin, out Fixed64 moverMax);
            ProjectConvex(target, axis, out Fixed64 targetMin, out Fixed64 targetMax);

            Fixed64 velocity = Vector2d.Dot(displacement, axis);
            if (velocity.Abs() <= Fixed64.Epsilon)
            {
                if (moverMax < targetMin || targetMax < moverMin)
                    return false;

                continue;
            }

            Fixed64 axisEntry;
            Fixed64 axisExit;
            Vector2d axisNormal;
            if (velocity > Fixed64.Zero)
            {
                axisEntry = (targetMin - moverMax) / velocity;
                axisExit = (targetMax - moverMin) / velocity;
                axisNormal = -axis;
            }
            else
            {
                axisEntry = (targetMax - moverMin) / velocity;
                axisExit = (targetMin - moverMax) / velocity;
                axisNormal = axis;
            }

            if (axisEntry > entryTime)
            {
                entryTime = axisEntry;
                entryNormal = axisNormal;
            }

            if (axisExit < exitTime)
                exitTime = axisExit;

            if (entryTime > exitTime)
                return false;
        }

        return true;
    }

    private static void ProjectConvex(LSCollider2D collider, Vector2d axis, out Fixed64 min, out Fixed64 max)
    {
        min = Vector2d.Dot(collider.GetVertexUnchecked(0), axis);
        max = min;
        for (int i = 1; i < collider.VertexCount; i++)
        {
            Fixed64 projection = Vector2d.Dot(collider.GetVertexUnchecked(i), axis);
            if (projection < min)
                min = projection;
            else if (projection > max)
                max = projection;
        }
    }

    private static void ProjectCapsule(LSCapsuleCollider2D capsule, Vector2d axis, out Fixed64 min, out Fixed64 max)
    {
        Fixed64 start = Vector2d.Dot(capsule.SegmentStart, axis);
        Fixed64 end = Vector2d.Dot(capsule.SegmentEnd, axis);
        Fixed64 radius = capsule.ScaledRadius;
        min = FixedMath.Min(start, end) - radius;
        max = FixedMath.Max(start, end) + radius;
    }

    private static void ProjectSegmentWithRadius(
        Vector2d segmentStart,
        Vector2d segmentEnd,
        Vector2d axis,
        Fixed64 radius,
        out Fixed64 min,
        out Fixed64 max)
    {
        Fixed64 start = Vector2d.Dot(segmentStart, axis);
        Fixed64 end = Vector2d.Dot(segmentEnd, axis);
        min = FixedMath.Min(start, end) - radius;
        max = FixedMath.Max(start, end) + radius;
    }

    private static Vector2d FindCapsuleAreaClosestAxis(LSCapsuleCollider2D capsule, ReadOnlySpan<Vector2d> vertices)
    {
        Vector2d segmentStart = capsule.SegmentStart;
        Vector2d segmentEnd = capsule.SegmentEnd;
        Fixed64 bestDistance = Fixed64.MaxValue;
        Vector2d bestAxis = Vector2d.Zero;

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d vertex = vertices[i];
            Vector2d segmentPoint = PlanarSegmentGeometry.ClosestPoint(vertex, segmentStart, segmentEnd);
            KeepClosestAxis(vertex - segmentPoint, ref bestDistance, ref bestAxis);
        }

        Vector2d closestToStart = ClosestPointOnConvexArea(segmentStart, vertices);
        KeepClosestAxis(closestToStart - segmentStart, ref bestDistance, ref bestAxis);
        Vector2d closestToEnd = ClosestPointOnConvexArea(segmentEnd, vertices);
        KeepClosestAxis(closestToEnd - segmentEnd, ref bestDistance, ref bestAxis);

        return bestAxis.MagnitudeSquared > Fixed64.Epsilon
            ? bestAxis
            : CalculateAverageCenter(vertices) - capsule.Center;
    }

    private static void ClosestPointsOnSegments(
        Vector2d firstStart,
        Vector2d firstEnd,
        Vector2d secondStart,
        Vector2d secondEnd,
        out Vector2d firstPoint,
        out Vector2d secondPoint)
    {
        if (TryIntersectSegments(firstStart, firstEnd - firstStart, secondStart, secondEnd - secondStart, out Fixed64 t))
        {
            firstPoint = firstStart + (firstEnd - firstStart) * t;
            secondPoint = firstPoint;
            return;
        }

        firstPoint = firstStart;
        secondPoint = PlanarSegmentGeometry.ClosestPoint(firstStart, secondStart, secondEnd);
        Fixed64 bestDistance = Vector2d.DistanceSquared(firstPoint, secondPoint);

        KeepClosestSegmentPair(firstEnd, PlanarSegmentGeometry.ClosestPoint(firstEnd, secondStart, secondEnd), ref firstPoint, ref secondPoint, ref bestDistance);
        KeepClosestSegmentPair(PlanarSegmentGeometry.ClosestPoint(secondStart, firstStart, firstEnd), secondStart, ref firstPoint, ref secondPoint, ref bestDistance);
        KeepClosestSegmentPair(PlanarSegmentGeometry.ClosestPoint(secondEnd, firstStart, firstEnd), secondEnd, ref firstPoint, ref secondPoint, ref bestDistance);
    }

    private static void KeepClosestAxis(Vector2d axis, ref Fixed64 bestDistance, ref Vector2d bestAxis)
    {
        Fixed64 distance = axis.MagnitudeSquared;
        if (distance >= bestDistance)
            return;

        bestDistance = distance;
        bestAxis = axis;
    }

    private static void KeepClosestSegmentPair(
        Vector2d candidateA,
        Vector2d candidateB,
        ref Vector2d bestA,
        ref Vector2d bestB,
        ref Fixed64 bestDistance)
    {
        Fixed64 distance = Vector2d.DistanceSquared(candidateA, candidateB);
        if (distance >= bestDistance)
            return;

        bestA = candidateA;
        bestB = candidateB;
        bestDistance = distance;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveCapsuleNormal(Vector2d point, Vector2d segmentPoint)
    {
        Vector2d normal = point - segmentPoint;
        return normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : Vector2d.Right;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void KeepEarlierDistance(Fixed64 candidate, ref bool found, ref Fixed64 bestDistance)
    {
        if (found && candidate >= bestDistance)
            return;

        found = true;
        bestDistance = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TryKeepEarlierHit(Physics2DHit candidate, ref bool found, ref Physics2DHit best)
    {
        if (found && !Physics2DHitSorter.ComesBefore(candidate, best))
            return;

        found = true;
        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SegmentBoundsOverlap(Vector2d start, Vector2d end, LSCollider2D collider)
    {
        Fixed64 minX = FixedMath.Min(start.X, end.X);
        Fixed64 maxX = FixedMath.Max(start.X, end.X);
        Fixed64 minY = FixedMath.Min(start.Y, end.Y);
        Fixed64 maxY = FixedMath.Max(start.Y, end.Y);
        return maxX >= collider.MinX
            && minX <= collider.MaxX
            && maxY >= collider.MinY
            && minY <= collider.MaxY;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SweepBoundsOverlap(Vector2d start, Vector2d end, Fixed64 radius, LSCollider2D collider)
    {
        Fixed64 minX = FixedMath.Min(start.X, end.X) - radius;
        Fixed64 maxX = FixedMath.Max(start.X, end.X) + radius;
        Fixed64 minY = FixedMath.Min(start.Y, end.Y) - radius;
        Fixed64 maxY = FixedMath.Max(start.Y, end.Y) + radius;
        return maxX >= collider.MinX
            && minX <= collider.MaxX
            && maxY >= collider.MinY
            && minY <= collider.MaxY;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryIntersectSegments(
        Vector2d rayStart,
        Vector2d raySegment,
        Vector2d edgeStart,
        Vector2d edgeSegment,
        out Fixed64 rayT)
    {
        Fixed64 denominator = Vector2d.CrossProduct(raySegment, edgeSegment);
        if (denominator == Fixed64.Zero || denominator.Abs() <= Fixed64.Epsilon)
        {
            rayT = default;
            return false;
        }

        Vector2d delta = edgeStart - rayStart;
        Fixed64 t = Vector2d.CrossProduct(delta, edgeSegment) / denominator;
        Fixed64 u = Vector2d.CrossProduct(delta, raySegment) / denominator;
        if (t < Fixed64.Zero || t > Fixed64.One || u < Fixed64.Zero || u > Fixed64.One)
        {
            rayT = default;
            return false;
        }

        rayT = t;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveQueryFallbackNormal(Vector2d center, Vector2d colliderCenter)
    {
        Vector2d direction = center - colliderCenter;
        return direction.MagnitudeSquared > Fixed64.Epsilon
            ? direction.Normalized
            : Vector2d.Right;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveOutwardEdgeNormal(Vector2d edgeStart, Vector2d edge, Vector2d colliderCenter)
    {
        Vector2d normal = edge.LeftHandNormal;
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return ResolveQueryFallbackNormal(edgeStart, colliderCenter);

        normal = normal.Normalized;
        if (Vector2d.Dot(colliderCenter - edgeStart, normal) > Fixed64.Zero)
            normal = -normal;
        return normal;
    }
}
