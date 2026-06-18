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

        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
        return TryCollide(new CollisionWorkItem2D(colliderA, colliderB, collisionType), out contact);
    }

    internal static bool TryCollide(CollisionPair2D pair, out Contact2D contact) =>
        TryCollide(CollisionWorkItem2D.Create(pair), out contact);

    internal static bool TryCollide(CollisionWorkItem2D item, out Contact2D contact)
    {
        LSCollider2D colliderA = item.ColliderA;
        LSCollider2D colliderB = item.ColliderB;
        if (!BoundsOverlap(colliderA, colliderB))
        {
            contact = default;
            return false;
        }

        switch (item.CollisionType)
        {
            case CollisionType2D.Circle_Circle:
                return TryCircleCircle((LSCircleCollider2D)colliderA, (LSCircleCollider2D)colliderB, out contact);
            case CollisionType2D.Circle_Convex:
                return TryCircleConvex((LSCircleCollider2D)colliderA, colliderB, out contact);
            case CollisionType2D.Convex_Circle:
                bool result = TryCircleConvex((LSCircleCollider2D)colliderB, colliderA, out Contact2D reversed);
                contact = result
                    ? new Contact2D(reversed.PointB, reversed.PointA, -reversed.Normal, reversed.Depth)
                    : default;
                return result;
            case CollisionType2D.Convex_Convex:
                return TryConvexConvex(colliderA, colliderB, out contact);
            case CollisionType2D.Compound:
                return TryCompound(colliderA, colliderB, out contact);
            default:
                contact = default;
                return false;
        }
    }

    private static bool TryCircleCircle(LSCircleCollider2D colliderA, LSCircleCollider2D colliderB, out Contact2D contact)
    {
        Vector2d delta = colliderB.Center - colliderA.Center;
        Fixed64 radius = colliderA.ScaledRadius + colliderB.ScaledRadius;
        Fixed64 distanceSquared = delta.MagnitudeSquared;
        if (distanceSquared > radius * radius)
        {
            contact = default;
            return false;
        }

        Fixed64 distance = distanceSquared > Fixed64.Zero ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector2d normal = distance > Fixed64.Zero ? delta / distance : Vector2d.Right;
        Fixed64 depth = radius - distance;
        contact = new Contact2D(
            colliderA.Center + normal * colliderA.ScaledRadius,
            colliderB.Center - normal * colliderB.ScaledRadius,
            normal,
            depth);
        return true;
    }

    private static bool TryCircleConvex(LSCircleCollider2D circle, LSCollider2D convex, out Contact2D contact)
    {
        Fixed64 bestOverlap = Fixed64.MaxValue;
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
        if (closestAxis.MagnitudeSquared > Fixed64.Epsilon
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
        Fixed64 bestOverlap = Fixed64.MaxValue;
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
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        Vector2d normal = axis.Normalized;
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
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        Vector2d normal = axis.Normalized;
        Fixed64 centerProjection = Vector2d.Dot(circle.Center, normal);
        Fixed64 radius = circle.ScaledRadius;
        Fixed64 minA = centerProjection - radius;
        Fixed64 maxA = centerProjection + radius;
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

    private static bool TryCompound(LSCollider2D colliderA, LSCollider2D colliderB, out Contact2D contact)
    {
        if (colliderA is LSCompoundCollider2D compoundA)
        {
            if (colliderB is LSCompoundCollider2D compoundB)
                return TryCompoundCompound(compoundA, compoundB, out contact);

            return TryCompoundOther(compoundA, colliderB, compoundIsA: true, out contact);
        }

        if (colliderB is LSCompoundCollider2D compound)
            return TryCompoundOther(compound, colliderA, compoundIsA: false, out contact);

        contact = default;
        return false;
    }

    private static bool TryCompoundCompound(
        LSCompoundCollider2D compoundA,
        LSCompoundCollider2D compoundB,
        out Contact2D contact)
    {
        bool found = false;
        Contact2D best = default;

        for (int i = 0; i < compoundA.PartCount; i++)
        {
            LSCollider2D partA = compoundA.GetPartCollider(i);
            for (int j = 0; j < compoundB.PartCount; j++)
            {
                LSCollider2D partB = compoundB.GetPartCollider(j);
                if (!TryCollide(partA, partB, out Contact2D candidate))
                    continue;

                if (!found || candidate.Depth < best.Depth)
                {
                    best = candidate;
                    found = true;
                }
            }
        }

        if (!found)
        {
            contact = default;
            return false;
        }

        contact = best;
        return true;
    }

    private static bool TryCompoundOther(
        LSCompoundCollider2D compound,
        LSCollider2D other,
        bool compoundIsA,
        out Contact2D contact)
    {
        bool found = false;
        Contact2D best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            Contact2D candidate;
            bool collided = compoundIsA
                ? TryCollide(part, other, out candidate)
                : TryCollide(other, part, out candidate);

            if (!collided)
                continue;

            if (!found || candidate.Depth < best.Depth)
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            contact = default;
            return false;
        }

        contact = best;
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
    private static Vector2d OrientAxis(Vector2d axis, Vector2d direction)
    {
        Vector2d normal = axis.MagnitudeSquared > Fixed64.Epsilon ? axis.Normalized : Vector2d.Right;
        if (direction.MagnitudeSquared > Fixed64.Epsilon && Vector2d.Dot(normal, direction) < Fixed64.Zero)
            return -normal;

        return normal;
    }

}
