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
        Fixed64 distanceSquared = containsCenter ? Fixed64.Zero : toCenter.SqrMagnitude;
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
        Fixed64 segmentLengthSquared = segment.SqrMagnitude;
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

    private static bool TryRaycastCircle(
        Vector2d start,
        Vector2d direction,
        Fixed64 segmentLength,
        LSCircleCollider2D circle,
        out Physics2DHit hit)
    {
        Vector2d originFromCenter = start - circle.Center;
        Fixed64 c = originFromCenter.SqrMagnitude - circle.Radius * circle.Radius;
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
            : (point - circle.Center).Normal;
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
        Fixed64 bestT = Fixed64.MAX_VALUE;
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
            if (normal.SqrMagnitude > Fixed64.Epsilon)
                normal = normal.Normal;
            if (Vector2d.Dot(normal, direction) > Fixed64.Zero)
                normal = -normal;

            found = true;
            bestT = t;
            bestPoint = start + segment * t;
            bestNormal = normal.SqrMagnitude > Fixed64.Epsilon ? normal : ResolveQueryFallbackNormal(bestPoint, collider.Center);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(collider, bestPoint, bestNormal, segmentLength * bestT);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SegmentBoundsOverlap(Vector2d start, Vector2d end, LSCollider2D collider)
    {
        Fixed64 minX = FixedMath.Min(start.x, end.x);
        Fixed64 maxX = FixedMath.Max(start.x, end.x);
        Fixed64 minY = FixedMath.Min(start.y, end.y);
        Fixed64 maxY = FixedMath.Max(start.y, end.y);
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
        return direction.SqrMagnitude > Fixed64.Epsilon
            ? direction.Normal
            : Vector2d.Right;
    }
}
