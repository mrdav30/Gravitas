//=======================================================================
// CollisionDetectionMixed.Separation.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetectionMixed
{
    internal static bool TryGetRotationalSeparationGap(
        LSCollider collider3D,
        LSCollider2D collider2D,
        out Fixed64 separationGap,
        out bool supported)
    {
        separationGap = Fixed64.Zero;
        supported = false;

        if (collider3D is LSSphereCollider sphere
            && collider2D is LSPolygonCollider2D polygon)
        {
            supported = true;
            return TryGetSpherePolygonSlabSeparationGap(
                sphere,
                polygon,
                out separationGap);
        }

        if (collider3D is LSCuboidCollider cuboid
            && collider2D is LSCircleCollider2D circle
            && IsWorldYAligned(cuboid))
        {
            supported = true;
            return TryGetYawCuboidCircleSlabSeparationGap(
                cuboid,
                circle,
                out separationGap);
        }

        return false;
    }

    private static bool TryGetSpherePolygonSlabSeparationGap(
        LSSphereCollider sphere,
        LSPolygonCollider2D polygon,
        out Fixed64 separationGap)
    {
        Vector3d center3D = sphere.Center;
        Vector2d center2D = new(center3D.X, center3D.Z);
        if (!TryGetPolygonExteriorGap(polygon, center2D, out Fixed64 planarGap)
            || !TryGetPointIntervalGap(
                center3D.Y,
                polygon.MixedBounds3D.Min.Y,
                polygon.MixedBounds3D.Max.Y,
                out Fixed64 verticalGap))
        {
            separationGap = default;
            return false;
        }

        // Both gaps are non-negative representable distances, so subtracting a
        // positive representable radius cannot leave Fixed64's signed domain.
        separationGap = FixedMath.Max(planarGap, verticalGap) - sphere.ScaledRadius;
        if (separationGap < Fixed64.Zero)
            separationGap = Fixed64.Zero;
        return true;
    }

    private static bool TryGetYawCuboidCircleSlabSeparationGap(
        LSCuboidCollider cuboid,
        LSCircleCollider2D circle,
        out Fixed64 separationGap)
    {
        Vector3d center3D = cuboid.Center;
        Vector2d cuboidCenter = new(center3D.X, center3D.Z);
        if (!Vector2d.TrySubtract(circle.Center, cuboidCenter, out Vector2d delta))
        {
            separationGap = default;
            return false;
        }

        // IsWorldYAligned and quaternion admission guarantee non-degenerate
        // planar X/Z axes. Dot saturation is intentional here: each result is
        // immediately clamped to the corresponding representable half extent.
        Vector3d worldAxisX = cuboid.Rotation.Rotate(Vector3d.Right);
        Vector3d worldAxisZ = cuboid.Rotation.Rotate(Vector3d.Forward);
        Vector2d axisX = new Vector2d(worldAxisX.X, worldAxisX.Z).Normalized;
        Vector2d axisZ = new Vector2d(worldAxisZ.X, worldAxisZ.Z).Normalized;
        Fixed64 halfX = cuboid.ScaledSize.X * Fixed64.Half;
        Fixed64 halfY = cuboid.ScaledSize.Y * Fixed64.Half;
        Fixed64 halfZ = cuboid.ScaledSize.Z * Fixed64.Half;
        Fixed64 localX = Vector2d.Dot(delta, axisX);
        Fixed64 localZ = Vector2d.Dot(delta, axisZ);
        localX = FixedMath.Clamp(localX, -halfX, halfX);
        localZ = FixedMath.Clamp(localZ, -halfZ, halfZ);
        // Saturation can only pull an out-of-domain box feature back onto the
        // representable boundary, which underestimates this pruning gap and is
        // therefore conservative.
        Vector2d closestOffset = axisX * localX + axisZ * localZ;
        Vector2d closestPoint = cuboidCenter + closestOffset;
        if (!Vector2d.TryGetDistance(circle.Center, closestPoint, out Fixed64 planarDistance)
            || !Fixed64.TrySubtract(center3D.Y, halfY, out Fixed64 cuboidMinY)
            || !Fixed64.TryAdd(center3D.Y, halfY, out Fixed64 cuboidMaxY)
            || !TryGetIntervalGap(
                cuboidMinY,
                cuboidMaxY,
                circle.MixedBounds3D.Min.Y,
                circle.MixedBounds3D.Max.Y,
                out Fixed64 verticalGap))
        {
            separationGap = default;
            return false;
        }

        Fixed64 planarGap = planarDistance - circle.ScaledRadius;
        separationGap = FixedMath.Max(
            FixedMath.Max(planarGap, Fixed64.Zero),
            verticalGap);
        return true;
    }

    private static bool TryGetPolygonExteriorGap(
        LSPolygonCollider2D polygon,
        Vector2d point,
        out Fixed64 exteriorGap)
    {
        Vector2d fanOrigin = polygon.GetVertexUnchecked(0);
        for (int i = 1; i < polygon.VertexCount - 1; i++)
        {
            if (new FixedTriangle2d(
                    fanOrigin,
                    polygon.GetVertexUnchecked(i),
                    polygon.GetVertexUnchecked(i + 1)).Contains(point))
            {
                exteriorGap = Fixed64.Zero;
                return true;
            }
        }

        Fixed64 bestDistance = Fixed64.MaxValue;
        int vertexCount = polygon.VertexCount;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d start = polygon.GetVertexUnchecked(i);
            Vector2d end = polygon.GetVertexUnchecked((i + 1) % vertexCount);
            Vector2d candidate = new FixedSegment2d(start, end).ClosestPoint(point);
            if (!Vector2d.TryGetDistance(point, candidate, out Fixed64 distance))
            {
                exteriorGap = default;
                return false;
            }

            if (distance < bestDistance)
                bestDistance = distance;
        }

        exteriorGap = bestDistance;
        return true;
    }

    private static bool IsWorldYAligned(LSCuboidCollider cuboid)
    {
        Vector3d yAxis = cuboid.Rotation.Rotate(Vector3d.Up);
        return yAxis.ToVector2d() == Vector2d.Zero
            && yAxis.Y.Abs() > Fixed64.Epsilon;
    }

    private static bool TryGetPointIntervalGap(
        Fixed64 point,
        Fixed64 min,
        Fixed64 max,
        out Fixed64 gap) =>
        TryGetIntervalGap(point, point, min, max, out gap);

    private static bool TryGetIntervalGap(
        Fixed64 firstMin,
        Fixed64 firstMax,
        Fixed64 secondMin,
        Fixed64 secondMax,
        out Fixed64 gap)
    {
        if (firstMax < secondMin)
            return Fixed64.TrySubtract(secondMin, firstMax, out gap);
        if (secondMax < firstMin)
            return Fixed64.TrySubtract(firstMin, secondMax, out gap);

        gap = Fixed64.Zero;
        return true;
    }
}
