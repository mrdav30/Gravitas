using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Deterministic mixed 2D/3D narrow-phase collision checks.
/// </summary>
public static class CollisionDetectionMixed
{
    public static bool TryCollide(LSCollider collider3D, LSCollider2D collider2D, out MixedContact contact)
    {
        SwiftThrowHelper.ThrowIfNull(collider3D, nameof(collider3D));
        SwiftThrowHelper.ThrowIfNull(collider2D, nameof(collider2D));

        if (!BoundsOverlap(collider3D.Bounds, collider2D.MixedBounds3D))
        {
            contact = default;
            return false;
        }

        return collider3D.Shape switch
        {
            ColliderType.Sphere => TrySphereEmbedded2D((LSSphereCollider)collider3D, collider2D, out contact),
            _ => NoContact(out contact)
        };
    }

    private static bool TrySphereEmbedded2D(LSSphereCollider sphere, LSCollider2D embedded, out MixedContact contact)
    {
        Vector3d sphereCenter = sphere.Center;
        Vector2d planarCenter = new(sphereCenter.x, sphereCenter.z);
        Fixed64 slabMinY = embedded.MixedBounds3D.Min.y;
        Fixed64 slabMaxY = embedded.MixedBounds3D.Max.y;
        bool planarInside = embedded.ContainsPoint(planarCenter);
        bool yInside = sphereCenter.y >= slabMinY && sphereCenter.y <= slabMaxY;

        if (planarInside && yInside)
            return TrySphereFromInsideEmbedded2D(sphere, embedded, planarCenter, slabMinY, slabMaxY, out contact);

        Vector2d closestPlanar = planarInside ? planarCenter : embedded.GetClosestPoint(planarCenter);
        Fixed64 closestY = Clamp(sphereCenter.y, slabMinY, slabMaxY);
        Vector3d closestEmbeddedPoint = new(closestPlanar.x, closestY, closestPlanar.y);
        Vector3d delta = closestEmbeddedPoint - sphereCenter;
        Fixed64 distanceSquared = delta.SqrMagnitude;
        if (distanceSquared > sphere.ScaledRadiusSqr)
        {
            contact = default;
            return false;
        }

        Fixed64 distance = distanceSquared > Fixed64.Epsilon ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector3d normal = distance > Fixed64.Zero
            ? delta / distance
            : ResolveFallbackNormal(sphereCenter, embedded);
        contact = new MixedContact(
            sphereCenter + normal * sphere.ScaledRadius,
            closestEmbeddedPoint,
            normal,
            sphere.ScaledRadius - distance);
        return true;
    }

    private static bool TrySphereFromInsideEmbedded2D(
        LSSphereCollider sphere,
        LSCollider2D embedded,
        Vector2d planarCenter,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        out MixedContact contact)
    {
        Vector3d sphereCenter = sphere.Center;
        Fixed64 minYDistance = sphereCenter.y - slabMinY;
        Fixed64 maxYDistance = slabMaxY - sphereCenter.y;
        Fixed64 bestDistance = minYDistance;
        Vector3d normal = -Vector3d.Up;
        Vector3d embeddedPoint = new(planarCenter.x, slabMinY, planarCenter.y);

        if (maxYDistance < bestDistance)
        {
            bestDistance = maxYDistance;
            normal = Vector3d.Up;
            embeddedPoint = new Vector3d(planarCenter.x, slabMaxY, planarCenter.y);
        }

        if (TryGetPlanarBoundaryPoint(embedded, planarCenter, out Vector2d planarBoundary, out Fixed64 planarDistance)
            && planarDistance < bestDistance)
        {
            bestDistance = planarDistance;
            Vector2d planarNormal = planarDistance > Fixed64.Epsilon
                ? (planarBoundary - planarCenter) / planarDistance
                : Vector2d.Right;
            normal = new Vector3d(planarNormal.x, Fixed64.Zero, planarNormal.y);
            embeddedPoint = new Vector3d(planarBoundary.x, sphereCenter.y, planarBoundary.y);
        }

        contact = new MixedContact(
            sphereCenter + normal * sphere.ScaledRadius,
            embeddedPoint,
            normal,
            sphere.ScaledRadius + bestDistance);
        return true;
    }

    private static bool TryGetPlanarBoundaryPoint(
        LSCollider2D embedded,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        switch (embedded.Shape)
        {
            case ColliderType2D.Circle:
                return TryGetCircleBoundary((LSCircleCollider2D)embedded, point, out boundary, out distance);
            case ColliderType2D.AABox:
                return TryGetAABoxBoundary((LSAABBoxCollider2D)embedded, point, out boundary, out distance);
            case ColliderType2D.ConvexPolygon:
                return TryGetConvexBoundary(embedded, point, out boundary, out distance);
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
        boundary = circle.Center + direction * circle.Radius;
        distance = circle.Radius - magnitude;
        return true;
    }

    private static bool TryGetAABoxBoundary(
        LSAABBoxCollider2D box,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        Fixed64 distanceMinX = point.x - box.MinX;
        Fixed64 distanceMaxX = box.MaxX - point.x;
        Fixed64 distanceMinY = point.y - box.MinY;
        Fixed64 distanceMaxY = box.MaxY - point.y;

        distance = distanceMinX;
        boundary = new Vector2d(box.MinX, point.y);

        if (distanceMaxX < distance)
        {
            distance = distanceMaxX;
            boundary = new Vector2d(box.MaxX, point.y);
        }

        if (distanceMinY < distance)
        {
            distance = distanceMinY;
            boundary = new Vector2d(point.x, box.MinY);
        }

        if (distanceMaxY < distance)
        {
            distance = distanceMaxY;
            boundary = new Vector2d(point.x, box.MaxY);
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
        if (vertexCount <= 0)
        {
            boundary = default;
            distance = Fixed64.Zero;
            return false;
        }

        Vector2d bestPoint = convex.GetVertexUnchecked(0);
        Fixed64 bestDistanceSquared = Fixed64.MAX_VALUE;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d a = convex.GetVertexUnchecked(i);
            Vector2d b = convex.GetVertexUnchecked((i + 1) % vertexCount);
            Vector2d candidate = ClosestPointOnSegment(point, a, b);
            Fixed64 candidateDistanceSquared = Vector2d.SqrDistance(point, candidate);
            if (candidateDistanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = candidateDistanceSquared;
            bestPoint = candidate;
        }

        boundary = bestPoint;
        distance = bestDistanceSquared > Fixed64.Epsilon ? FixedMath.Sqrt(bestDistanceSquared) : Fixed64.Zero;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ClosestPointOnSegment(Vector2d point, Vector2d a, Vector2d b)
    {
        Vector2d segment = b - a;
        Fixed64 lengthSquared = segment.SqrMagnitude;
        if (lengthSquared <= Fixed64.Epsilon)
            return a;

        Fixed64 t = Vector2d.Dot(point - a, segment) / lengthSquared;
        t = Clamp(t, Fixed64.Zero, Fixed64.One);
        return a + segment * t;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlap(BoundingBox first, BoundingBox second) =>
        first.Max.x >= second.Min.x
        && first.Min.x <= second.Max.x
        && first.Max.y >= second.Min.y
        && first.Min.y <= second.Max.y
        && first.Max.z >= second.Min.z
        && first.Min.z <= second.Max.z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveFallbackNormal(Vector3d sphereCenter, LSCollider2D embedded)
    {
        Vector3d fallback = new(
            embedded.Center.x - sphereCenter.x,
            embedded.MixedSlabCenterY - sphereCenter.y,
            embedded.Center.y - sphereCenter.z);
        return fallback.SqrMagnitude > Fixed64.Epsilon ? fallback.Normal : Vector3d.Right;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Clamp(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool NoContact(out MixedContact contact)
    {
        contact = default;
        return false;
    }
}
