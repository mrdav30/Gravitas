//=======================================================================
// CollisionDetectionMixed.Separation.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

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
            && collider2D is LSCircleCollider2D circle)
        {
            supported = true;
            separationGap = cuboid.OrientedBox.GetCircleSlabSeparationLowerBound(
                new Vector3d(
                    circle.Center.X,
                    circle.MixedSlabCenterY,
                    circle.Center.Y),
                circle.MixedHalfThickness,
                circle.ScaledRadius);
            return true;
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

    private static bool TryGetPolygonExteriorGap(
        LSPolygonCollider2D polygon,
        Vector2d point,
        out Fixed64 exteriorGap)
    {
        ReadOnlySpan<Vector2d> offsets = polygon.ScaledLocalVertices;
        if (FixedConvex2dRelations.ContainsPoint(
                point,
                polygon.Center,
                polygon.Rotation,
                offsets))
        {
            exteriorGap = Fixed64.Zero;
            return true;
        }

        FixedPointAnchor2d closestPoint =
            FixedConvex2dRelations.GetClosestPointAnchor(
                point,
                polygon.Center,
                polygon.Rotation,
                offsets);
        if (!new ContactAnchor2D(closestPoint)
                .TryGetOffsetFrom(point, out Vector2d difference)
            || !Vector2d.TryGetMagnitude(difference, out exteriorGap))
        {
            exteriorGap = default;
            return false;
        }

        return true;
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
