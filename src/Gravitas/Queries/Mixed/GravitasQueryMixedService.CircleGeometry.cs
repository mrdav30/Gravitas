//=======================================================================
// GravitasQueryMixedService.CircleGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns shared projected-circle sweep geometry helpers for mixed query reducers.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    internal static bool TrySweepCircleAgainstSphere(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSSphereCollider sphere,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 sphereRadius = sphere.ScaledRadius;
        if (!FixedMath.TryGetSphereSlabCrossSectionRadius(
                sphere.Center.Y,
                sphereRadius,
                slabCenterY,
                halfThickness,
                out Fixed64 planarSphereRadius))
        {
            hit = default;
            return false;
        }

        Vector2d sphereCenter = new(sphere.Center.X, sphere.Center.Z);
        if (!TrySweepPointInPlane(
            start,
            end,
            direction,
            length,
            sphereCenter,
            planarSphereRadius,
            radius,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D = distance == length ? end : start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            sphere,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
        return true;
    }

    private static bool TrySweepPointInPlane(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Vector2d point,
        Fixed64 radius,
        out Fixed64 distance)
    {
        return TrySweepPointInPlane(
            start,
            end,
            direction,
            length,
            point,
            radius,
            Fixed64.Zero,
            out distance);
    }

    private static bool TrySweepPointInPlane(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Vector2d point,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 distance)
    {
        return RadialSweepAdmission.TryIntersect(
            start,
            direction,
            length,
            point,
            radius,
            radiusExpansion,
            end,
            point,
            out distance);
    }

    private static bool TrySweepCircleAgainstTriangleProjection(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector3d first,
        Vector3d second,
        Vector3d third,
        out Fixed64 distance,
        out Vector3d point3D)
    {
        Span<Vector3d> clipped = stackalloc Vector3d[8];
        if (!TryClipTriangleToSlab(first, second, third, slabMinY, slabMaxY, clipped, out int clippedCount))
        {
            distance = default;
            point3D = default;
            return false;
        }

        Span<Vector2d> projection = stackalloc Vector2d[8];
        int projectionCount = 0;
        for (int i = 0; i < clippedCount; i++)
            TryAddUniqueProjectionPoint(projection, ref projectionCount, ToPlanar(clipped[i]));

        BuildConvexHullInPlace(projection, ref projectionCount);
        if (!TrySweepCircleAgainstConvexProjection(
            start,
            end,
            direction,
            length,
            radius,
            projection.Slice(0, projectionCount),
            out distance))
        {
            point3D = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        point3D = FindClosestPointOnClippedProjection(
            clipped.Slice(0, clippedCount),
            center2D,
            (slabMinY + slabMaxY) * Fixed64.Half);
        return true;
    }

    private static bool TryClipTriangleToSlab(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Span<Vector3d> clipped,
        out int clippedCount)
    {
        Span<Vector3d> source = stackalloc Vector3d[8];
        Span<Vector3d> intermediate = stackalloc Vector3d[8];
        source[0] = first;
        source[1] = second;
        source[2] = third;

        ClipPolygonAgainstYPlane(source, 3, intermediate, slabMinY, keepAbove: true, out int minClipCount);
        if (minClipCount == 0)
        {
            clippedCount = 0;
            return false;
        }

        ClipPolygonAgainstYPlane(intermediate, minClipCount, clipped, slabMaxY, keepAbove: false, out clippedCount);
        return clippedCount > 0;
    }

    private static void ClipPolygonAgainstYPlane(
        ReadOnlySpan<Vector3d> input,
        int inputCount,
        Span<Vector3d> output,
        Fixed64 planeY,
        bool keepAbove,
        out int outputCount)
    {
        outputCount = 0;
        Vector3d previous = input[inputCount - 1];
        bool previousInside = IsInsideYPlane(previous, planeY, keepAbove);
        for (int i = 0; i < inputCount; i++)
        {
            Vector3d current = input[i];
            bool currentInside = IsInsideYPlane(current, planeY, keepAbove);
            if (currentInside)
            {
                if (!previousInside)
                    AddClippedPoint(output, ref outputCount, IntersectYPlane(previous, current, planeY));

                AddClippedPoint(output, ref outputCount, current);
            }
            else if (previousInside)
            {
                AddClippedPoint(output, ref outputCount, IntersectYPlane(previous, current, planeY));
            }

            previous = current;
            previousInside = currentInside;
        }

    }

    private static Vector3d FindClosestPointOnClippedProjection(
        ReadOnlySpan<Vector3d> polygon,
        Vector2d point,
        Fixed64 referenceY)
    {
        Vector3d best = polygon[0];
        Fixed64 bestDistanceSqr = (ToPlanar(best) - point).MagnitudeSquared;
        Fixed64 bestYDistance = (best.Y - referenceY).Abs();

        for (int i = 0; i < polygon.Length; i++)
        {
            Vector3d first = polygon[i];
            Vector3d second = polygon[(i + 1) % polygon.Length];
            Vector2d first2D = ToPlanar(first);
            Vector2d second2D = ToPlanar(second);
            Vector2d edge = second2D - first2D;
            Fixed64 edgeLengthSqr = edge.MagnitudeSquared;
            Vector3d candidate;
            if (edgeLengthSqr <= Fixed64.Epsilon)
            {
                Fixed64 firstYDistance = (first.Y - referenceY).Abs();
                Fixed64 secondYDistance = (second.Y - referenceY).Abs();
                candidate = firstYDistance <= secondYDistance ? first : second;
            }
            else
            {
                Fixed64 t = Vector2d.Dot(point - first2D, edge) / edgeLengthSqr;
                t = FixedMath.Clamp01(t);
                candidate = first + (second - first) * t;
            }

            Fixed64 distanceSqr = (ToPlanar(candidate) - point).MagnitudeSquared;
            Fixed64 yDistance = (candidate.Y - referenceY).Abs();
            if (distanceSqr > bestDistanceSqr
                || (distanceSqr == bestDistanceSqr && yDistance >= bestYDistance))
            {
                continue;
            }

            best = candidate;
            bestDistanceSqr = distanceSqr;
            bestYDistance = yDistance;
        }

        return best;
    }

    private static Vector3d IntersectYPlane(Vector3d first, Vector3d second, Fixed64 planeY)
    {
        Fixed64 deltaY = second.Y - first.Y;
        Fixed64 t = (planeY - first.Y) / deltaY;
        return first + (second - first) * FixedMath.Clamp01(t);
    }

    private static void AddClippedPoint(Span<Vector3d> points, ref int count, Vector3d point)
    {
        if (count > 0 && PointsEquivalent(points[count - 1], point))
            return;

        points[count++] = point;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideYPlane(Vector3d point, Fixed64 planeY, bool keepAbove) =>
        keepAbove ? point.Y >= planeY : point.Y <= planeY;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PointsEquivalent(Vector3d first, Vector3d second) =>
        (first - second).MagnitudeSquared <= Fixed64.Epsilon;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ToPlanar(Vector3d point) => new(point.X, point.Z);

    private static bool TryBuildCuboidSlabProjection(
        LSCuboidCollider cuboid,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Span<Vector2d> projection,
        out int projectionCount)
    {
        projectionCount = 0;
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        Vector3d[] vertices = cuboid.Vertices;

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3d vertex = vertices[i];
            if (vertex.Y >= slabMinY && vertex.Y <= slabMaxY)
                TryAddUniqueProjectionPoint(projection, ref projectionCount, new Vector2d(vertex.X, vertex.Z));
        }

        for (int i = 0; i < LSCuboidCollider.EdgeDefinitions.Length; i++)
        {
            int[] edge = LSCuboidCollider.EdgeDefinitions[i];
            Vector3d first = vertices[edge[0]];
            Vector3d second = vertices[edge[1]];
            TryAddSlabPlaneIntersection(first, second, slabMinY, projection, ref projectionCount);
            TryAddSlabPlaneIntersection(first, second, slabMaxY, projection, ref projectionCount);
        }

        if (projectionCount == 0)
            return false;

        BuildConvexHullInPlace(projection, ref projectionCount);
        return projectionCount > 0;
    }

    private static void TryAddSlabPlaneIntersection(
        Vector3d first,
        Vector3d second,
        Fixed64 planeY,
        Span<Vector2d> projection,
        ref int projectionCount)
    {
        Fixed64 deltaY = second.Y - first.Y;
        if (deltaY.Abs() <= Fixed64.Epsilon)
            return;

        Fixed64 t = (planeY - first.Y) / deltaY;
        if (t < Fixed64.Zero || t > Fixed64.One)
            return;

        Vector3d point = first + (second - first) * t;
        TryAddUniqueProjectionPoint(projection, ref projectionCount, new Vector2d(point.X, point.Z));
    }

    private static void TryAddUniqueProjectionPoint(Span<Vector2d> projection, ref int projectionCount, Vector2d point)
    {
        for (int i = 0; i < projectionCount; i++)
        {
            Vector2d delta = projection[i] - point;
            if (delta.MagnitudeSquared <= Fixed64.Epsilon)
                return;
        }

        projection[projectionCount++] = point;
    }

    private static void BuildConvexHullInPlace(Span<Vector2d> projection, ref int projectionCount)
    {
        if (projectionCount <= 2)
            return;

        SortProjectionPoints(projection.Slice(0, projectionCount));
        Span<Vector2d> hull = stackalloc Vector2d[64];
        int hullCount = 0;

        for (int i = 0; i < projectionCount; i++)
        {
            while (hullCount >= 2 && Cross(hull[hullCount - 2], hull[hullCount - 1], projection[i]) <= Fixed64.Zero)
                hullCount--;

            hull[hullCount++] = projection[i];
        }

        int lowerCount = hullCount;
        for (int i = projectionCount - 2; i >= 0; i--)
        {
            while (hullCount > lowerCount && Cross(hull[hullCount - 2], hull[hullCount - 1], projection[i]) <= Fixed64.Zero)
                hullCount--;

            hull[hullCount++] = projection[i];
        }

        hullCount--;

        for (int i = 0; i < hullCount; i++)
            projection[i] = hull[i];

        projectionCount = hullCount;
    }

    private static void SortProjectionPoints(Span<Vector2d> points)
    {
        for (int i = 1; i < points.Length; i++)
        {
            Vector2d candidate = points[i];
            int j = i - 1;
            while (j >= 0 && ComesAfter(points[j], candidate))
            {
                points[j + 1] = points[j];
                j--;
            }

            points[j + 1] = candidate;
        }
    }

    private static bool TrySweepCircleAgainstConvexProjection(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        ReadOnlySpan<Vector2d> projection,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (projection.Length == 1)
            return TrySweepPointInPlane(start, end, direction, length, projection[0], radius, out distance);

        if (projection.Length == 2)
            return TrySweepPointAgainstSegmentCapsule(start, end, direction, length, projection[0], projection[1], radius, out distance);

        Fixed64 radiusSqr = radius * radius;
        if (IsPointInsideConvexProjection(start, projection)
            || DistanceSquaredToConvexProjection(start, projection) <= radiusSqr)
        {
            return true;
        }

        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        for (int i = 0; i < projection.Length; i++)
        {
            Vector2d first = projection[i];
            Vector2d second = projection[(i + 1) % projection.Length];
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule(start, end, direction, length, first, second, radius, out Fixed64 candidate),
                candidate,
                ref found,
                ref best);
        }

        distance = best;
        return found;
    }

    private static bool TrySweepPointAgainstSegmentCapsule(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Vector2d segmentStart,
        Vector2d segmentEnd,
        Fixed64 radius,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        Fixed64 radiusSqr = radius * radius;
        if (new FixedSegment2d(segmentStart, segmentEnd).DistanceSquared(start) <= radiusSqr)
            return true;

        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        TryKeepEarlierSweep(
            TrySweepPointInPlane(start, end, direction, length, segmentStart, radius, out Fixed64 startDistance),
            startDistance,
            ref found,
            ref best);
        TryKeepEarlierSweep(
            TrySweepPointInPlane(start, end, direction, length, segmentEnd, radius, out Fixed64 endDistance),
            endDistance,
            ref found,
            ref best);

        Vector2d edge = segmentEnd - segmentStart;
        Fixed64 edgeLength = edge.Magnitude;
        Vector2d edgeDirection = edge / edgeLength;
        Vector2d normal = new(-edgeDirection.Y, edgeDirection.X);
        Fixed64 signedStart = Vector2d.Dot(start - segmentStart, normal);
        Fixed64 signedDirection = Vector2d.Dot(direction, normal);
        if (signedDirection.Abs() > Fixed64.Epsilon)
        {
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentOffsetLine(
                    start,
                    direction,
                    length,
                    segmentStart,
                    edgeDirection,
                    edgeLength,
                    signedStart,
                    signedDirection,
                    radius,
                    out Fixed64 positiveDistance),
                positiveDistance,
                ref found,
                ref best);
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentOffsetLine(
                    start,
                    direction,
                    length,
                    segmentStart,
                    edgeDirection,
                    edgeLength,
                    signedStart,
                    signedDirection,
                    -radius,
                    out Fixed64 negativeDistance),
                negativeDistance,
                ref found,
                ref best);
        }

        distance = best;
        return found;
    }

    private static bool TrySweepPointAgainstSegmentOffsetLine(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Vector2d segmentStart,
        Vector2d edgeDirection,
        Fixed64 edgeLength,
        Fixed64 signedStart,
        Fixed64 signedDirection,
        Fixed64 signedRadius,
        out Fixed64 distance)
    {
        distance = (signedRadius - signedStart) / signedDirection;
        if (distance < Fixed64.Zero || distance > length)
            return false;

        Vector2d point = start + direction * distance;
        Fixed64 projection = Vector2d.Dot(point - segmentStart, edgeDirection);
        return projection >= Fixed64.Zero && projection <= edgeLength;
    }

    private static bool IsPointInsideConvexProjection(Vector2d point, ReadOnlySpan<Vector2d> projection)
    {
        bool hasPositive = false;
        bool hasNegative = false;
        for (int i = 0; i < projection.Length; i++)
        {
            Fixed64 cross = Cross(projection[i], projection[(i + 1) % projection.Length], point);
            hasPositive |= cross > Fixed64.Epsilon;
            hasNegative |= cross < -Fixed64.Epsilon;
            if (hasPositive && hasNegative)
                return false;
        }

        return true;
    }

    private static Fixed64 DistanceSquaredToConvexProjection(Vector2d point, ReadOnlySpan<Vector2d> projection)
    {
        Fixed64 best = Fixed64.MaxValue;
        for (int i = 0; i < projection.Length; i++)
        {
            Fixed64 distanceSqr = new FixedSegment2d(
                projection[i],
                projection[(i + 1) % projection.Length]).DistanceSquared(point);
            if (distanceSqr < best)
                best = distanceSqr;
        }

        return best;
    }

    private static bool TryGetVerticalSegmentInterval(Vector3d start, Vector3d end, out Fixed64 minY, out Fixed64 maxY)
    {
        Vector3d segment = end - start;
        if (segment.X * segment.X + segment.Z * segment.Z > Fixed64.Epsilon)
        {
            minY = Fixed64.Zero;
            maxY = Fixed64.Zero;
            return false;
        }

        minY = FixedMath.Min(start.Y, end.Y);
        maxY = FixedMath.Max(start.Y, end.Y);
        return true;
    }

    private static Fixed64 GetIntervalDistance(Fixed64 firstMin, Fixed64 firstMax, Fixed64 secondMin, Fixed64 secondMax)
    {
        if (firstMax < secondMin)
            return secondMin - firstMax;

        if (secondMax < firstMin)
            return firstMin - secondMax;

        return Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IntervalsOverlap(Fixed64 firstMin, Fixed64 firstMax, Fixed64 secondMin, Fixed64 secondMax) =>
        firstMin <= secondMax && secondMin <= firstMax;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ComesAfter(Vector2d first, Vector2d second) =>
        first.X > second.X || (first.X == second.X && first.Y > second.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Cross(Vector2d origin, Vector2d first, Vector2d second) =>
        (first.X - origin.X) * (second.Y - origin.Y) - (first.Y - origin.Y) * (second.X - origin.X);

}
