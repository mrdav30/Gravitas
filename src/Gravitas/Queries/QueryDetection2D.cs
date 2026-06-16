using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
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

    internal static bool TryRaycast(Vector2d start, Vector2d end, LSCollider2D collider, out Physics2DHit hit)
    {
        Vector2d segment = end - start;
        Fixed64 segmentLengthSquared = segment.MagnitudeSquared;
        if (segmentLengthSquared == Fixed64.Zero || !SegmentBoundsOverlap(start, end, collider))
        {
            hit = default;
            return false;
        }

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

        return collider is LSCircleCollider2D circle
            ? TryRaycastCircle(start, direction, segmentLength, circle, out hit)
            : TryRaycastConvex(start, segment, direction, segmentLength, collider, out hit);
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

        if (TryOverlapCircle(start, radius, collider, out Physics2DHit overlapHit))
        {
            hit = new Physics2DHit(collider, start, overlapHit.Normal, Fixed64.Zero);
            return true;
        }

        Fixed64 segmentLength = FixedMath.Sqrt(segmentLengthSquared);
        Vector2d direction = segment / segmentLength;
        return collider is LSCircleCollider2D circle
            ? TrySweepCircleCircle(start, direction, segmentLength, radius, circle, out hit)
            : TrySweepCircleConvex(start, direction, segmentLength, radius, collider, out hit);
    }

    private static bool TryRaycastCircle(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        Vector2d originFromCenter = start - circle.Center;
        Fixed64 c = originFromCenter.MagnitudeSquared - circle.Radius * circle.Radius;
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
        Fixed64 combinedRadius = radius + circle.Radius;
        if (!TryRaycastCircleDistance(start, direction, segmentLength, circle.Center, combinedRadius, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d sweptCenter = start + direction * distance;
        Vector2d normal = sweptCenter == circle.Center
            ? ResolveQueryFallbackNormal(sweptCenter, circle.Center)
            : (sweptCenter - circle.Center).Normalized;
        Vector2d point = circle.Center + normal * circle.Radius;
        hit = new Physics2DHit(circle, point, normal, distance);
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
        if (denominator.Abs() <= Fixed64.Epsilon)
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
