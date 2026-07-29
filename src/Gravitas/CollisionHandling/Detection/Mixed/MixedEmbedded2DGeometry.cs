//=======================================================================
// MixedEmbedded2DGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Shared fixed-point geometry helpers for embedded 2D mixed slabs.
/// </summary>
internal static class MixedEmbedded2DGeometry
{
    public static Vector3d GetCenter3D(LSCollider2D embedded) =>
        new(embedded.Center.X, embedded.MixedSlabCenterY, embedded.Center.Y);

    public static ContactAnchor GetSupportAnchor(LSCollider2D embedded, Vector3d worldDirection)
    {
        Vector2d planarDirection = new(worldDirection.X, worldDirection.Z);
        if (!TryGetPlanarSupportAnchor(
                embedded,
                planarDirection,
                out ContactAnchor2D planarAnchor))
        {
            throw new InvalidOperationException(
                "Committed embedded 2D geometry has no representable owner-relative support anchor.");
        }

        Fixed64 yOffset = worldDirection.Y > Fixed64.Zero
            ? embedded.MixedHalfThickness
            : worldDirection.Y < Fixed64.Zero
                ? -embedded.MixedHalfThickness
                : Fixed64.Zero;
        return EmbedPlanarAnchor(
            planarAnchor,
            embedded.MixedSlabCenterY,
            yOffset);
    }

    public static ContactAnchor GetClosestAnchorOnEmbeddedVolume(
        LSCollider2D embedded,
        Vector3d point)
    {
        Vector2d planarPoint = new(point.X, point.Z);
        bool planarInside = embedded.ContainsPoint(planarPoint);
        GetClosestSlabOffset(
            point.Y,
            embedded.MixedSlabCenterY,
            embedded.MixedHalfThickness,
            out Fixed64 closestYOffset,
            out bool yInside,
            out Fixed64 signedCenterOffset);

        if (!planarInside || !yInside)
        {
            if (planarInside)
            {
                return new ContactAnchor(
                    new Vector3d(
                        planarPoint.X,
                        embedded.MixedSlabCenterY,
                        planarPoint.Y),
                    new Vector3d(
                        Fixed64.Zero,
                        closestYOffset,
                        Fixed64.Zero));
            }

            if (TryGetPlanarBoundaryAnchor(
                    embedded,
                    planarPoint,
                    out ContactAnchor2D closestPlanar))
            {
                return EmbedPlanarAnchor(
                    closestPlanar,
                    embedded.MixedSlabCenterY,
                    closestYOffset);
            }

            throw new InvalidOperationException(
                "Committed embedded 2D geometry has no semantic boundary anchor.");
        }

        bool upperYBoundary = signedCenterOffset > Fixed64.Zero;
        Fixed64 yBoundaryOffset = upperYBoundary
            ? embedded.MixedHalfThickness
            : -embedded.MixedHalfThickness;
        Fixed64 bestDistance = upperYBoundary
            ? embedded.MixedHalfThickness - signedCenterOffset
            : embedded.MixedHalfThickness + signedCenterOffset;
        ContactAnchor closest = new(
            new Vector3d(
                planarPoint.X,
                embedded.MixedSlabCenterY,
                planarPoint.Y),
            new Vector3d(
                Fixed64.Zero,
                yBoundaryOffset,
                Fixed64.Zero));

        // A point inside an admitted built-in shape is no farther from its
        // nearest boundary than that shape's representable dimensions.
        _ = TryGetPlanarBoundaryAnchor(
            embedded,
            planarPoint,
            out ContactAnchor2D planarBoundary,
            out Fixed64 planarDistance);
        if (planarDistance < bestDistance)
        {
            closest = EmbedPlanarAnchor(
                planarBoundary,
                embedded.MixedSlabCenterY,
                signedCenterOffset);
        }

        return closest;
    }

    public static bool ContainsPointInSlab(LSCollider2D embedded, Fixed64 y)
    {
        GetClosestSlabOffset(
            y,
            embedded.MixedSlabCenterY,
            embedded.MixedHalfThickness,
            out _,
            out bool inside,
            out _);
        return inside;
    }

    public static bool TryGetPlanarBoundaryPoint(
        LSCollider2D embedded,
        Vector2d point,
        out Vector2d boundary,
        out Fixed64 distance)
    {
        if (TryGetPlanarBoundaryAnchor(
                embedded,
                point,
                out ContactAnchor2D anchor,
                out distance)
            && anchor.TryGetWorldPoint(out boundary))
        {
            return true;
        }

        boundary = default;
        return false;
    }

    public static bool TryGetPlanarBoundaryAnchor(
        LSCollider2D embedded,
        Vector2d point,
        out ContactAnchor2D boundary)
    {
        if (embedded.TryGetClosestBoundaryAnchor(
                point,
                out FixedPointAnchor2d anchor))
        {
            boundary = new ContactAnchor2D(anchor);
            return true;
        }

        boundary = default;
        return false;
    }

    public static bool TryGetPlanarBoundaryAnchor(
        LSCollider2D embedded,
        Vector2d point,
        out ContactAnchor2D boundary,
        out Fixed64 distance)
    {
        if (embedded.TryGetClosestBoundaryAnchor(
                point,
                out FixedPointAnchor2d anchor,
                out distance))
        {
            boundary = new ContactAnchor2D(anchor);
            return true;
        }

        boundary = default;
        return false;
    }

    private static bool TryGetPlanarSupportAnchor(
        LSCollider2D embedded,
        Vector2d direction,
        out ContactAnchor2D anchor)
    {
        switch (embedded.Shape)
        {
            case ColliderType2D.Circle:
                {
                    var circle = (LSCircleCollider2D)embedded;
                    return TryGetRoundSupportAnchor(
                        circle.Center,
                        circle.Rotation,
                        Fixed64.Zero,
                        circle.ScaledRadius,
                        direction,
                        out anchor);
                }
            case ColliderType2D.Capsule:
                {
                    var capsule = (LSCapsuleCollider2D)embedded;
                    return TryGetRoundSupportAnchor(
                        capsule.Center,
                        capsule.Rotation,
                        capsule.AxisLength,
                        capsule.ScaledRadius,
                        direction,
                        out anchor);
                }
            case ColliderType2D.AABox:
            case ColliderType2D.ConvexPolygon:
                anchor = new ContactAnchor2D(
                    embedded.GetConvexSupportAnchor(
                        direction == Vector2d.Zero
                            ? Vector2d.Right
                            : direction));
                return true;
            default:
                anchor = default;
                return false;
        }
    }

    private static bool TryGetRoundSupportAnchor(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        out ContactAnchor2D anchor)
    {
        Vector2d localDirection = direction == Vector2d.Zero
            ? Vector2d.Zero
            : Vector2d.Rotate(direction.Normalized, -rotation);

        FixedPointAnchor2d localAnchor =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                center,
                rotation,
                Vector2d.Forward,
                axisLength,
                radius,
                localDirection);
        anchor = new ContactAnchor2D(localAnchor);
        return true;
    }

    private static ContactAnchor EmbedPlanarAnchor(
        ContactAnchor2D planar,
        Fixed64 slabCenterY,
        Fixed64 localYDisplacement = default)
    {
        return new ContactAnchor(
            new Vector3d(planar.Origin.X, slabCenterY, planar.Origin.Y),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -planar.Rotation),
            new Vector3d(
                planar.LocalPoint.X,
                Fixed64.Zero,
                planar.LocalPoint.Y),
            new Vector3d(
                planar.LocalDisplacement.X,
                localYDisplacement,
                planar.LocalDisplacement.Y));
    }

    private static void GetClosestSlabOffset(
        Fixed64 y,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        out Fixed64 closestOffset,
        out bool inside,
        out Fixed64 signedCenterOffset)
    {
        if (!Fixed64.TrySubtract(y, slabCenterY, out signedCenterOffset))
        {
            closestOffset = y < slabCenterY
                ? -halfThickness
                : halfThickness;
            inside = false;
            signedCenterOffset = closestOffset;
            return;
        }

        if (signedCenterOffset < -halfThickness)
        {
            closestOffset = -halfThickness;
            inside = false;
            return;
        }

        if (signedCenterOffset > halfThickness)
        {
            closestOffset = halfThickness;
            inside = false;
            return;
        }

        closestOffset = signedCenterOffset;
        inside = true;
    }
}
