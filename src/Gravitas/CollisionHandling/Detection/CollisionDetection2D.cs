using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
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

    internal static bool TryCollide(CollisionPair2D pair, ContactManifold2D manifold, int frame)
    {
        SwiftThrowHelper.ThrowIfNull(pair, nameof(pair));
        return TryCollide(CollisionWorkItem2D.Create(pair), manifold, frame);
    }

    internal static bool TryCollide(CollisionWorkItem2D item, ContactManifold2D manifold, int frame)
    {
        SwiftThrowHelper.ThrowIfNull(manifold, nameof(manifold));
        manifold.BeginUpdate(frame);
        return TryCollide(item, manifold);
    }

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

    private static bool TryCollide(CollisionWorkItem2D item, ContactManifold2D manifold)
    {
        LSCollider2D colliderA = item.ColliderA;
        LSCollider2D colliderB = item.ColliderB;
        if (!BoundsOverlap(colliderA, colliderB))
            return false;

        return item.CollisionType switch
        {
            CollisionType2D.Circle_Circle => TryCircleCircle((LSCircleCollider2D)colliderA, (LSCircleCollider2D)colliderB, manifold),
            CollisionType2D.Circle_Convex => TryCircleConvex((LSCircleCollider2D)colliderA, colliderB, manifold),
            CollisionType2D.Convex_Circle => TryCircleConvexReversed((LSCircleCollider2D)colliderB, colliderA, manifold),
            CollisionType2D.Convex_Convex => TryConvexConvex(colliderA, colliderB, manifold),
            CollisionType2D.Compound => TryCompound(colliderA, colliderB, manifold),
            _ => false
        };
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

    private static bool TryCircleCircle(
        LSCircleCollider2D colliderA,
        LSCircleCollider2D colliderB,
        ContactManifold2D manifold)
    {
        if (!TryCircleCircle(colliderA, colliderB, out Contact2D contact))
            return false;

        AddContact(manifold, contact);
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

    private static bool TryCircleConvex(
        LSCircleCollider2D circle,
        LSCollider2D convex,
        ContactManifold2D manifold)
    {
        if (!TryCircleConvex(circle, convex, out Contact2D contact))
            return false;

        AddContact(manifold, contact);
        return true;
    }

    private static bool TryCircleConvexReversed(
        LSCircleCollider2D circle,
        LSCollider2D convex,
        ContactManifold2D manifold)
    {
        if (!TryCircleConvex(circle, convex, out Contact2D contact))
            return false;

        manifold.AddContact(contact.PointB, contact.PointA, contact.Depth, -contact.Normal);
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

    private static bool TryConvexConvex(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        ContactManifold2D manifold)
    {
        if (!TryFindMinimumPenetrationAxis(colliderA, colliderB, out MinimumAxis2D axis))
            return false;

        bool referenceIsA = axis.SourceIsA;
        LSCollider2D reference = referenceIsA ? colliderA : colliderB;
        LSCollider2D incident = referenceIsA ? colliderB : colliderA;
        Vector2d ownerNormal = axis.Normal;
        Vector2d referenceNormal = referenceIsA ? ownerNormal : -ownerNormal;

        Edge2D referenceEdge = FindReferenceEdge(reference, referenceNormal);
        Edge2D incidentEdge = FindIncidentEdge(incident, referenceNormal);
        var first = new ClipPoint2D(incidentEdge.Start);
        var second = new ClipPoint2D(incidentEdge.End);

        int count = ClipSegment(referenceEdge.Start, referenceEdge.Direction, ref first, ref second);
        if (count == 0)
            return false;

        count = ClipSegment(referenceEdge.End, -referenceEdge.Direction, ref first, ref second, count);
        if (count == 0)
            return false;

        bool found = false;
        for (int i = 0; i < count; i++)
        {
            Vector2d incidentPoint = i == 0 ? first.Point : second.Point;
            Fixed64 separation = Vector2d.Dot(incidentPoint - referenceEdge.Start, referenceNormal);
            if (separation > Fixed64.Epsilon)
                continue;

            Fixed64 depth = separation < Fixed64.Zero ? -separation : Fixed64.Zero;
            Vector2d referencePoint = incidentPoint - referenceNormal * separation;
            if (referenceIsA)
                manifold.AddContact(referencePoint, incidentPoint, depth, ownerNormal);
            else
                manifold.AddContact(incidentPoint, referencePoint, depth, ownerNormal);

            found = true;
        }

        return found;
    }

    private static bool TryFindMinimumPenetrationAxis(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        out MinimumAxis2D axis)
    {
        axis = new MinimumAxis2D(Fixed64.MaxValue, Vector2d.Zero, sourceIsA: true, hasAxis: false);
        return TryTestConvexAxes(colliderA, colliderA, colliderB, sourceIsA: true, ref axis)
            && TryTestConvexAxes(colliderB, colliderA, colliderB, sourceIsA: false, ref axis)
            && axis.HasAxis;
    }

    private static bool TryTestConvexAxes(
        LSCollider2D axisSource,
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        bool sourceIsA,
        ref MinimumAxis2D bestAxis)
    {
        for (int i = 0; i < axisSource.VertexCount; i++)
        {
            Vector2d edge = axisSource.GetVertexUnchecked((i + 1) % axisSource.VertexCount) - axisSource.GetVertexUnchecked(i);
            if (!TryTestManifoldAxis(edge.RightHandNormal, colliderA, colliderB, sourceIsA, ref bestAxis))
                return false;
        }

        return true;
    }

    private static bool TryTestManifoldAxis(
        Vector2d axis,
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        bool sourceIsA,
        ref MinimumAxis2D bestAxis)
    {
        if (axis.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        Vector2d normal = axis.Normalized;
        Project(colliderA, normal, out Fixed64 minA, out Fixed64 maxA);
        Project(colliderB, normal, out Fixed64 minB, out Fixed64 maxB);
        Fixed64 overlap = FixedMath.Min(maxA, maxB) - FixedMath.Max(minA, minB);
        if (overlap < Fixed64.Zero)
            return false;

        if (!bestAxis.HasAxis || overlap < bestAxis.Overlap)
        {
            Vector2d orientedNormal = OrientAxis(normal, colliderB.Center - colliderA.Center);
            bestAxis = new MinimumAxis2D(overlap, orientedNormal, sourceIsA, hasAxis: true);
        }

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

                if (!found || candidate.Depth > best.Depth)
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

            if (!found || candidate.Depth > best.Depth)
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

    private static bool TryCompound(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        ContactManifold2D manifold)
    {
        if (colliderA is LSCompoundCollider2D compoundA)
        {
            if (colliderB is LSCompoundCollider2D compoundB)
                return TryCompoundCompound(compoundA, compoundB, manifold);

            return TryCompoundOther(compoundA, colliderB, compoundIsA: true, manifold);
        }

        if (colliderB is LSCompoundCollider2D compound)
            return TryCompoundOther(compound, colliderA, compoundIsA: false, manifold);

        return false;
    }

    private static bool TryCompoundCompound(
        LSCompoundCollider2D compoundA,
        LSCompoundCollider2D compoundB,
        ContactManifold2D manifold)
    {
        bool found = false;
        for (int i = 0; i < compoundA.PartCount; i++)
        {
            LSCollider2D partA = compoundA.GetPartCollider(i);
            for (int j = 0; j < compoundB.PartCount; j++)
            {
                LSCollider2D partB = compoundB.GetPartCollider(j);
                CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(partA.Shape, partB.Shape);
                found |= TryCollide(new CollisionWorkItem2D(partA, partB, collisionType), manifold);
            }
        }

        return found;
    }

    private static bool TryCompoundOther(
        LSCompoundCollider2D compound,
        LSCollider2D other,
        bool compoundIsA,
        ContactManifold2D manifold)
    {
        bool found = false;
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            LSCollider2D colliderA = compoundIsA ? part : other;
            LSCollider2D colliderB = compoundIsA ? other : part;
            CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
            found |= TryCollide(new CollisionWorkItem2D(colliderA, colliderB, collisionType), manifold);
        }

        return found;
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

    private static Edge2D FindReferenceEdge(LSCollider2D collider, Vector2d outwardNormal)
    {
        int bestIndex = 0;
        Fixed64 bestDot = -Fixed64.MaxValue;
        for (int i = 0; i < collider.VertexCount; i++)
        {
            Vector2d start = collider.GetVertexUnchecked(i);
            Vector2d end = collider.GetVertexUnchecked((i + 1) % collider.VertexCount);
            Vector2d edge = end - start;
            if (edge.MagnitudeSquared <= Fixed64.Epsilon)
                continue;

            Fixed64 dot = Vector2d.Dot(edge.LeftHandNormal.Normalized, outwardNormal);
            if (dot > bestDot)
            {
                bestDot = dot;
                bestIndex = i;
            }
        }

        return CreateEdge(collider, bestIndex);
    }

    private static Edge2D FindIncidentEdge(LSCollider2D collider, Vector2d referenceNormal)
    {
        int bestIndex = 0;
        Fixed64 bestDot = Fixed64.MaxValue;
        for (int i = 0; i < collider.VertexCount; i++)
        {
            Vector2d start = collider.GetVertexUnchecked(i);
            Vector2d end = collider.GetVertexUnchecked((i + 1) % collider.VertexCount);
            Vector2d edge = end - start;
            if (edge.MagnitudeSquared <= Fixed64.Epsilon)
                continue;

            Fixed64 dot = Vector2d.Dot(edge.LeftHandNormal.Normalized, referenceNormal);
            if (dot < bestDot)
            {
                bestDot = dot;
                bestIndex = i;
            }
        }

        return CreateEdge(collider, bestIndex);
    }

    private static Edge2D CreateEdge(LSCollider2D collider, int index)
    {
        Vector2d start = collider.GetVertexUnchecked(index);
        Vector2d end = collider.GetVertexUnchecked((index + 1) % collider.VertexCount);
        Vector2d direction = end - start;
        return new Edge2D(start, end, direction.MagnitudeSquared > Fixed64.Epsilon ? direction.Normalized : Vector2d.Zero);
    }

    private static int ClipSegment(
        Vector2d planePoint,
        Vector2d insideNormal,
        ref ClipPoint2D first,
        ref ClipPoint2D second,
        int count = 2)
    {
        if (count <= 0)
            return 0;

        ClipPoint2D input0 = first;
        ClipPoint2D input1 = count > 1 ? second : first;
        Fixed64 distance0 = Vector2d.Dot(input0.Point - planePoint, insideNormal);
        Fixed64 distance1 = Vector2d.Dot(input1.Point - planePoint, insideNormal);
        bool inside0 = distance0 >= Fixed64.Zero;
        bool inside1 = count > 1 && distance1 >= Fixed64.Zero;
        int outputCount = 0;
        ClipPoint2D output0 = default;
        ClipPoint2D output1 = default;

        if (inside0)
            AddClippedPoint(input0, ref output0, ref output1, ref outputCount);

        if (count > 1 && inside0 != inside1)
        {
            Fixed64 denominator = distance0 - distance1;
            if (denominator != Fixed64.Zero)
            {
                Fixed64 t = distance0 / denominator;
                AddClippedPoint(
                    new ClipPoint2D(input0.Point + (input1.Point - input0.Point) * t),
                    ref output0,
                    ref output1,
                    ref outputCount);
            }
        }

        if (inside1)
            AddClippedPoint(input1, ref output0, ref output1, ref outputCount);

        first = output0;
        second = outputCount > 1 ? output1 : default;
        return outputCount;
    }

    private static void AddClippedPoint(
        ClipPoint2D point,
        ref ClipPoint2D first,
        ref ClipPoint2D second,
        ref int count)
    {
        if (count == 0)
        {
            first = point;
            count = 1;
            return;
        }

        second = point;
        count = 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddContact(ContactManifold2D manifold, Contact2D contact) =>
        manifold.AddContact(contact.PointA, contact.PointB, contact.Depth, contact.Normal);

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

    private readonly struct MinimumAxis2D
    {
        public MinimumAxis2D(Fixed64 overlap, Vector2d normal, bool sourceIsA, bool hasAxis)
        {
            Overlap = overlap;
            Normal = normal;
            SourceIsA = sourceIsA;
            HasAxis = hasAxis;
        }

        public Fixed64 Overlap { get; }

        public Vector2d Normal { get; }

        public bool SourceIsA { get; }

        public bool HasAxis { get; }
    }

    private readonly struct Edge2D
    {
        public Edge2D(Vector2d start, Vector2d end, Vector2d direction)
        {
            Start = start;
            End = end;
            Direction = direction;
        }

        public Vector2d Start { get; }

        public Vector2d End { get; }

        public Vector2d Direction { get; }
    }

    private readonly struct ClipPoint2D
    {
        public ClipPoint2D(Vector2d point)
        {
            Point = point;
        }

        public Vector2d Point { get; }
    }

}
