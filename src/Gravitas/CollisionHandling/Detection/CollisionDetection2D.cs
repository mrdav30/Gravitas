using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Deterministic pure 2D narrow-phase collision checks.
/// </summary>
public static class CollisionDetection2D
{
    public static bool TryCollide(LSCollider2D colliderA, LSCollider2D colliderB, out Contact2D contact)
    {
        SwiftThrowHelper.ThrowIfNull(colliderA, nameof(colliderA));
        SwiftThrowHelper.ThrowIfNull(colliderB, nameof(colliderB));

        if (!BoundsOverlap(colliderA, colliderB))
        {
            contact = default;
            return false;
        }

        if (colliderA is LSCircleCollider2D circleA && colliderB is LSCircleCollider2D circleB)
            return TryCircleCircle(circleA, circleB, out contact);

        if (colliderA is LSCircleCollider2D circle && colliderB is not LSCircleCollider2D)
            return TryCircleConvex(circle, colliderB, out contact);

        if (colliderB is LSCircleCollider2D circleOther && colliderA is not LSCircleCollider2D)
        {
            bool result = TryCircleConvex(circleOther, colliderA, out Contact2D reversed);
            contact = result
                ? new Contact2D(reversed.PointB, reversed.PointA, -reversed.Normal, reversed.Depth)
                : default;
            return result;
        }

        return TryConvexConvex(colliderA, colliderB, out contact);
    }

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

    private static bool TryCircleCircle(LSCircleCollider2D colliderA, LSCircleCollider2D colliderB, out Contact2D contact)
    {
        Vector2d delta = colliderB.Center - colliderA.Center;
        Fixed64 radius = colliderA.Radius + colliderB.Radius;
        Fixed64 distanceSquared = delta.SqrMagnitude;
        if (distanceSquared > radius * radius)
        {
            contact = default;
            return false;
        }

        Fixed64 distance = distanceSquared > Fixed64.Zero ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector2d normal = distance > Fixed64.Zero ? delta / distance : Vector2d.Right;
        Fixed64 depth = radius - distance;
        contact = new Contact2D(
            colliderA.Center + normal * colliderA.Radius,
            colliderB.Center - normal * colliderB.Radius,
            normal,
            depth);
        return true;
    }

    private static bool TryCircleConvex(LSCircleCollider2D circle, LSCollider2D convex, out Contact2D contact)
    {
        Fixed64 bestOverlap = Fixed64.MAX_VALUE;
        Vector2d bestAxis = Vector2d.Zero;

        for (int i = 0; i < convex.VertexCount; i++)
        {
            Vector2d edge = convex.GetVertexUnchecked((i + 1) % convex.VertexCount) - convex.GetVertexUnchecked(i);
            if (!TryTestAxis(edge.RightHandNormal, circle, convex, ref bestOverlap, ref bestAxis))
            {
                contact = default;
                return false;
            }
        }

        Vector2d closest = convex.GetClosestPoint(circle.Center);
        Vector2d closestAxis = closest - circle.Center;
        if (closestAxis.SqrMagnitude > Fixed64.Epsilon
            && !TryTestAxis(closestAxis, circle, convex, ref bestOverlap, ref bestAxis))
        {
            contact = default;
            return false;
        }

        Vector2d direction = convex.Center - circle.Center;
        Vector2d normal = OrientAxis(bestAxis, direction);
        contact = new Contact2D(
            circle.GetSupportPoint(normal),
            convex.GetSupportPoint(-normal),
            normal,
            bestOverlap);
        return true;
    }

    private static bool TryConvexConvex(LSCollider2D colliderA, LSCollider2D colliderB, out Contact2D contact)
    {
        Fixed64 bestOverlap = Fixed64.MAX_VALUE;
        Vector2d bestAxis = Vector2d.Zero;

        if (!TryTestConvexAxes(colliderA, colliderA, colliderB, ref bestOverlap, ref bestAxis)
            || !TryTestConvexAxes(colliderB, colliderA, colliderB, ref bestOverlap, ref bestAxis))
        {
            contact = default;
            return false;
        }

        Vector2d normal = OrientAxis(bestAxis, colliderB.Center - colliderA.Center);
        contact = new Contact2D(
            colliderA.GetSupportPoint(normal),
            colliderB.GetSupportPoint(-normal),
            normal,
            bestOverlap);
        return true;
    }

    private static bool TryTestConvexAxes(
        LSCollider2D axisSource,
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        ref Fixed64 bestOverlap,
        ref Vector2d bestAxis)
    {
        for (int i = 0; i < axisSource.VertexCount; i++)
        {
            Vector2d edge = axisSource.GetVertexUnchecked((i + 1) % axisSource.VertexCount) - axisSource.GetVertexUnchecked(i);
            if (!TryTestAxis(edge.RightHandNormal, colliderA, colliderB, ref bestOverlap, ref bestAxis))
                return false;
        }

        return true;
    }

    private static bool TryTestAxis(
        Vector2d axis,
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        ref Fixed64 bestOverlap,
        ref Vector2d bestAxis)
    {
        if (axis.SqrMagnitude <= Fixed64.Epsilon)
            return true;

        Vector2d normal = axis.Normal;
        Project(colliderA, normal, out Fixed64 minA, out Fixed64 maxA);
        Project(colliderB, normal, out Fixed64 minB, out Fixed64 maxB);
        Fixed64 overlap = FixedMath.Min(maxA, maxB) - FixedMath.Max(minA, minB);
        if (overlap < Fixed64.Zero)
            return false;

        if (overlap < bestOverlap)
        {
            bestOverlap = overlap;
            bestAxis = normal;
        }

        return true;
    }

    private static bool TryTestAxis(
        Vector2d axis,
        LSCircleCollider2D circle,
        LSCollider2D convex,
        ref Fixed64 bestOverlap,
        ref Vector2d bestAxis)
    {
        if (axis.SqrMagnitude <= Fixed64.Epsilon)
            return true;

        Vector2d normal = axis.Normal;
        Fixed64 centerProjection = Vector2d.Dot(circle.Center, normal);
        Fixed64 minA = centerProjection - circle.Radius;
        Fixed64 maxA = centerProjection + circle.Radius;
        Project(convex, normal, out Fixed64 minB, out Fixed64 maxB);
        Fixed64 overlap = FixedMath.Min(maxA, maxB) - FixedMath.Max(minA, minB);
        if (overlap < Fixed64.Zero)
            return false;

        if (overlap < bestOverlap)
        {
            bestOverlap = overlap;
            bestAxis = normal;
        }

        return true;
    }

    private static void Project(LSCollider2D collider, Vector2d axis, out Fixed64 min, out Fixed64 max)
    {
        Vector2d first = collider.GetVertexUnchecked(0);
        min = Vector2d.Dot(first, axis);
        max = min;
        for (int i = 1; i < collider.VertexCount; i++)
        {
            Fixed64 projection = Vector2d.Dot(collider.GetVertexUnchecked(i), axis);
            if (projection < min)
                min = projection;
            if (projection > max)
                max = projection;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool BoundsOverlap(LSCollider2D colliderA, LSCollider2D colliderB) =>
        colliderA.MinX <= colliderB.MaxX
        && colliderA.MaxX >= colliderB.MinX
        && colliderA.MinY <= colliderB.MaxY
        && colliderA.MaxY >= colliderB.MinY;

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
    private static Vector2d OrientAxis(Vector2d axis, Vector2d direction)
    {
        Vector2d normal = axis.SqrMagnitude > Fixed64.Epsilon ? axis.Normal : Vector2d.Right;
        if (direction.SqrMagnitude > Fixed64.Epsilon && Vector2d.Dot(normal, direction) < Fixed64.Zero)
            return -normal;

        return normal;
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
