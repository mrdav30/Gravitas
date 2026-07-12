//=======================================================================
// MixedEmbedded2DGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Shared fixed-point geometry helpers for embedded 2D mixed slabs.
/// </summary>
internal static class MixedEmbedded2DGeometry
{
    public static Vector3d GetClosestPointOnEmbeddedVolume(LSCollider2D embedded, Vector3d point)
    {
        Vector2d planarPoint = new(point.X, point.Z);
        Fixed64 slabMinY = embedded.MixedBounds3D.Min.Y;
        Fixed64 slabMaxY = embedded.MixedBounds3D.Max.Y;
        bool planarInside = embedded.ContainsPoint(planarPoint);
        bool yInside = point.Y >= slabMinY && point.Y <= slabMaxY;

        if (!planarInside || !yInside)
        {
            Vector2d closestPlanar = planarInside ? planarPoint : embedded.GetClosestPoint(planarPoint);
            return new Vector3d(closestPlanar.X, FixedMath.Clamp(point.Y, slabMinY, slabMaxY), closestPlanar.Y);
        }

        Fixed64 minYDistance = point.Y - slabMinY;
        Fixed64 maxYDistance = slabMaxY - point.Y;
        Fixed64 bestDistance = minYDistance;
        Vector3d closest = new(planarPoint.X, slabMinY, planarPoint.Y);

        if (maxYDistance < bestDistance)
        {
            bestDistance = maxYDistance;
            closest = new Vector3d(planarPoint.X, slabMaxY, planarPoint.Y);
        }

        if (TryGetPlanarBoundaryPoint(embedded, planarPoint, out Vector2d planarBoundary, out Fixed64 planarDistance)
            && planarDistance < bestDistance)
        {
            closest = new Vector3d(planarBoundary.X, point.Y, planarBoundary.Y);
        }

        return closest;
    }

    public static bool TryGetPlanarBoundaryPoint(
        LSCollider2D embedded,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        switch (embedded.Shape)
        {
            case ColliderType2D.Circle:
                return TryGetCircleBoundary((LSCircleCollider2D)embedded, point, out boundary, out distance);
            case ColliderType2D.Capsule:
                return TryGetCapsuleBoundary((LSCapsuleCollider2D)embedded, point, out boundary, out distance);
            case ColliderType2D.AABox:
                return TryGetAABoxBoundary((LSAABBoxCollider2D)embedded, point, out boundary, out distance);
            case ColliderType2D.ConvexPolygon:
                return TryGetConvexBoundary(embedded, point, out boundary, out distance);
            case ColliderType2D.Compound:
                return TryGetCompoundBoundary((LSCompoundCollider2D)embedded, point, out boundary, out distance);
            default:
                boundary = default;
                distance = Fixed64.Zero;
                return false;
        }
    }

    private static bool TryGetCircleBoundary(
        LSCircleCollider2D circle,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        Vector2d delta = point - circle.Center;
        Fixed64 magnitude = delta.Magnitude;
        Vector2d direction = magnitude > Fixed64.Epsilon ? delta / magnitude : Vector2d.Right;
        Fixed64 radius = circle.ScaledRadius;
        boundary = circle.Center + direction * radius;
        distance = (radius - magnitude).Abs();
        return true;
    }

    private static bool TryGetCapsuleBoundary(
        LSCapsuleCollider2D capsule,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        Vector2d segmentPoint = PlanarSegmentGeometry.ClosestPoint(point, capsule.SegmentStart, capsule.SegmentEnd);
        Vector2d delta = point - segmentPoint;
        Fixed64 magnitude = delta.Magnitude;
        Vector2d direction = magnitude > Fixed64.Epsilon ? delta / magnitude : Vector2d.Right;
        Fixed64 radius = capsule.ScaledRadius;
        boundary = segmentPoint + direction * radius;
        distance = (radius - magnitude).Abs();
        return true;
    }

    private static bool TryGetAABoxBoundary(
        LSAABBoxCollider2D box,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        Fixed64 distanceMinX = (point.X - box.MinX).Abs();
        Fixed64 distanceMaxX = (box.MaxX - point.X).Abs();
        Fixed64 distanceMinY = (point.Y - box.MinY).Abs();
        Fixed64 distanceMaxY = (box.MaxY - point.Y).Abs();

        distance = distanceMinX;
        boundary = new Vector2d(box.MinX, point.Y);

        if (distanceMaxX < distance)
        {
            distance = distanceMaxX;
            boundary = new Vector2d(box.MaxX, point.Y);
        }

        if (distanceMinY < distance)
        {
            distance = distanceMinY;
            boundary = new Vector2d(point.X, box.MinY);
        }

        if (distanceMaxY < distance)
        {
            distance = distanceMaxY;
            boundary = new Vector2d(point.X, box.MaxY);
        }

        return true;
    }

    private static bool TryGetConvexBoundary(
        LSCollider2D convex,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        int vertexCount = convex.VertexCount;
        Vector2d bestPoint = convex.GetVertexUnchecked(0);
        Fixed64 bestDistanceSquared = Fixed64.MaxValue;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d a = convex.GetVertexUnchecked(i);
            Vector2d b = convex.GetVertexUnchecked((i + 1) % vertexCount);
            Vector2d candidate = PlanarSegmentGeometry.ClosestPoint(point, a, b);
            Fixed64 candidateDistanceSquared = Vector2d.DistanceSquared(point, candidate);
            if (candidateDistanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = candidateDistanceSquared;
            bestPoint = candidate;
        }

        boundary = bestPoint;
        distance = bestDistanceSquared > Fixed64.Epsilon ? FixedMath.Sqrt(bestDistanceSquared) : Fixed64.Zero;
        return true;
    }

    private static bool TryGetCompoundBoundary(
        LSCompoundCollider2D compound,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        if (TryGetCompoundBoundary(compound, point, containingPartsOnly: true, out boundary, out distance))
            return true;

        return TryGetCompoundBoundary(compound, point, containingPartsOnly: false, out boundary, out distance);
    }

    private static bool TryGetCompoundBoundary(
        LSCompoundCollider2D compound,
        Vector2d point,
        bool containingPartsOnly,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        boundary = default;
        distance = Fixed64.Zero;
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (containingPartsOnly && !part.ContainsPoint(point))
                continue;

            TryGetPlanarBoundaryPoint(part, point, out Vector2d candidate, out Fixed64 candidateDistance);
            if (candidateDistance >= bestDistance)
                continue;

            boundary = candidate;
            distance = candidateDistance;
            bestDistance = candidateDistance;
            found = true;
        }

        return found;
    }
}
