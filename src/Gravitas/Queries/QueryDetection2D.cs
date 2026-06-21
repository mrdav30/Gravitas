using FixedMathSharp;
using Gravitas.CollisionHandling;
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

        if (collider is LSCompoundCollider2D compound)
            return TrySweepCircleCompound(start, end, radius, compound, out hit);

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

        if (target is LSCircleCollider2D targetCircle)
            return TrySweepConvexMoverAgainstCircle(mover, displacement, targetCircle, out hit);

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
