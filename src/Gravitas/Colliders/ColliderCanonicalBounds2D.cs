//=======================================================================
// ColliderCanonicalBounds2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>
/// Contains methods for computing canonical bounds of 2D colliders, 
/// including proxy radii and relative bounds, based on collider type and properties.
/// </summary>
internal static class ColliderCanonicalBounds2D
{
    internal static Fixed64 GetCurrentCenteredProxyRadius(
        LSCollider2D collider)
    {
        if (collider is LSCompoundCollider2D compound)
            return GetCurrentCompoundCenteredProxyRadius(compound);

        return GetShapeCenteredProxyRadius(collider);
    }

    internal static Fixed64 GetCenteredProxyRadius(
        LSCollider2D collider)
    {
        if (collider is LSCompoundCollider2D compound)
            return GetCompoundCenteredProxyRadius(compound);

        return GetShapeCenteredProxyRadius(collider);
    }

    private static Fixed64 GetShapeCenteredProxyRadius(
        LSCollider2D collider)
    {
        if (collider is LSCircleCollider2D circle)
            return circle.ScaledRadius;
        if (collider is LSCapsuleCollider2D capsule)
        {
            FixedBoundArea localBounds =
                FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                    Vector2d.Zero,
                    Vector2d.Forward,
                    capsule.AxisLength,
                    capsule.ScaledRadius);
            return localBounds.Max.Y;
        }
        if (collider.VertexCount == 0)
            return Fixed64.MaxValue;

        Fixed64 radius = Fixed64.Zero;
        for (int index = 0; index < collider.VertexCount; index++)
        {
            if (!collider.GetScaledLocalVertexUnchecked(index)
                    .TryGetMagnitudeCeiling(out Fixed64 vertexRadius))
            {
                return Fixed64.MaxValue;
            }

            radius = FixedMath.Max(radius, vertexRadius);
        }

        return radius;
    }

    private static Fixed64 GetCurrentCompoundCenteredProxyRadius(
        LSCompoundCollider2D compound)
    {
        Fixed64 radius = Fixed64.Zero;
        for (int index = 0; index < compound.PartCount; index++)
        {
            LSCollider2D part = compound.GetPartCollider(index);
            bool distanceResolved = part.TryGetCurrentScaledOffset(
                    out Vector2d centerOffset)
                & centerOffset.TryGetMagnitudeCeiling(
                    out Fixed64 centerDistance);
            bool radiusResolved = Fixed64.TryAdd(
                centerDistance,
                GetCurrentCenteredProxyRadius(part),
                out Fixed64 partRadius);
            if (!(distanceResolved & radiusResolved))
                return Fixed64.MaxValue;

            radius = FixedMath.Max(radius, partRadius);
        }

        return radius;
    }

    internal static Fixed64 GetGroundProbeRadius(
        LSCollider2D collider)
    {
        if (collider is LSCircleCollider2D circle)
            return circle.ScaledRadius;
        if (collider is LSCapsuleCollider2D capsule)
            return capsule.ScaledRadius;
        if (collider is LSCompoundCollider2D compound)
        {
            return TryGetCompoundRelativeBounds(
                compound,
                compound.CanonicalCenter,
                out Vector2d minimum,
                out Vector2d maximum)
                ? GetMinimumAbsoluteExtent(minimum, maximum)
                : Fixed64.MaxValue;
        }
        if (collider.VertexCount == 0)
            return Fixed64.Zero;

        return TryGetConvexBoundsFromRelativeCenter(
            collider,
            Vector2d.Zero,
            out Vector2d convexMinimum,
            out Vector2d convexMaximum)
            ? GetMinimumAbsoluteExtent(convexMinimum, convexMaximum)
            : Fixed64.MaxValue;
    }

    private static Fixed64 GetCompoundCenteredProxyRadius(
        LSCompoundCollider2D compound)
    {
        Fixed64 radius = Fixed64.Zero;
        for (int index = 0; index < compound.PartCount; index++)
        {
            LSCollider2D part = compound.GetPartCollider(index);
            bool offsetResolved = Vector2d.TrySubtract(
                part.CanonicalCenter,
                compound.CanonicalCenter,
                out Vector2d centerOffset);
            bool distanceResolved =
                centerOffset.TryGetMagnitudeCeiling(
                    out Fixed64 centerDistance);
            bool radiusResolved = Fixed64.TryAdd(
                centerDistance,
                part.CanonicalCenteredProxyRadius,
                out Fixed64 partRadius);
            if (!(offsetResolved & distanceResolved & radiusResolved))
                return Fixed64.MaxValue;

            radius = FixedMath.Max(radius, partRadius);
        }

        return radius;
    }

    private static bool TryGetCompoundRelativeBounds(
        LSCompoundCollider2D compound,
        Vector2d referenceCenter,
        out Vector2d minimum,
        out Vector2d maximum)
    {
        minimum = default;
        maximum = default;
        bool hasBounds = false;
        for (int index = 0; index < compound.PartCount; index++)
        {
            LSCollider2D part = compound.GetPartCollider(index);
            if (!TryGetRelativeBounds(
                    part,
                    referenceCenter,
                    out Vector2d partMinimum,
                    out Vector2d partMaximum))
            {
                minimum = default;
                maximum = default;
                return false;
            }

            if (!hasBounds)
            {
                minimum = partMinimum;
                maximum = partMaximum;
                hasBounds = true;
                continue;
            }

            minimum = new Vector2d(
                FixedMath.Min(minimum.X, partMinimum.X),
                FixedMath.Min(minimum.Y, partMinimum.Y));
            maximum = new Vector2d(
                FixedMath.Max(maximum.X, partMaximum.X),
                FixedMath.Max(maximum.Y, partMaximum.Y));
        }

        return hasBounds;
    }

    private static bool TryGetRelativeBounds(
        LSCollider2D collider,
        Vector2d referenceCenter,
        out Vector2d minimum,
        out Vector2d maximum)
    {
        if (!Vector2d.TrySubtract(
                collider.CanonicalCenter,
                referenceCenter,
                out Vector2d relativeCenter))
        {
            minimum = default;
            maximum = default;
            return false;
        }

        if (collider is LSCircleCollider2D circle)
        {
            return TryGetRoundBounds(
                relativeCenter,
                circle.ScaledRadius,
                circle.ScaledRadius,
                out minimum,
                out maximum);
        }

        if (collider is LSCapsuleCollider2D capsule)
        {
            FixedBoundArea centeredBounds =
                FixedBoundArea.FromCenteredRotatedCapsuleClippedToDomain(
                    Vector2d.Zero,
                    capsule.Rotation,
                    capsule.AxisLength,
                    capsule.ScaledRadius);
            return TryGetRoundBounds(
                relativeCenter,
                centeredBounds.Max.X,
                centeredBounds.Max.Y,
                out minimum,
                out maximum);
        }

        return TryGetConvexBoundsFromRelativeCenter(
            collider,
            relativeCenter,
            out minimum,
            out maximum);
    }

    private static bool TryGetRoundBounds(
        Vector2d center,
        Fixed64 extentX,
        Fixed64 extentY,
        out Vector2d minimum,
        out Vector2d maximum)
    {
        bool representable =
            Fixed64.TrySubtract(center.X, extentX, out Fixed64 minimumX)
            & Fixed64.TrySubtract(center.Y, extentY, out Fixed64 minimumY)
            & Fixed64.TryAdd(center.X, extentX, out Fixed64 maximumX)
            & Fixed64.TryAdd(center.Y, extentY, out Fixed64 maximumY);
        minimum = representable
            ? new Vector2d(minimumX, minimumY)
            : default;
        maximum = representable
            ? new Vector2d(maximumX, maximumY)
            : default;
        return representable;
    }

    private static bool TryGetConvexBoundsFromRelativeCenter(
        LSCollider2D collider,
        Vector2d relativeCenter,
        out Vector2d minimum,
        out Vector2d maximum)
    {
        if (!Vector2d.TryTransformPoint(
                relativeCenter,
                collider.GetScaledLocalVertexUnchecked(0),
                collider.ConvexRotation,
                out minimum))
        {
            minimum = default;
            maximum = default;
            return false;
        }

        maximum = minimum;
        for (int index = 1; index < collider.VertexCount; index++)
        {
            if (!Vector2d.TryTransformPoint(
                    relativeCenter,
                    collider.GetScaledLocalVertexUnchecked(index),
                    collider.ConvexRotation,
                    out Vector2d point))
            {
                minimum = default;
                maximum = default;
                return false;
            }

            minimum = new Vector2d(
                FixedMath.Min(minimum.X, point.X),
                FixedMath.Min(minimum.Y, point.Y));
            maximum = new Vector2d(
                FixedMath.Max(maximum.X, point.X),
                FixedMath.Max(maximum.Y, point.Y));
        }

        return true;
    }

    private static Fixed64 GetMinimumAbsoluteExtent(
        Vector2d minimum,
        Vector2d maximum) =>
        FixedMath.Min(
            FixedMath.Max(minimum.X.Abs(), maximum.X.Abs()),
            FixedMath.Max(minimum.Y.Abs(), maximum.Y.Abs()));
}
