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
    // The caller supplies a compound leaf. Retain its authored local box until
    // the final mesh-relative transform; clipped world bounds lose boundary geometry.
    internal static void GetMixedRelativeBounds(LSCollider2D collider,
        Vector3d referenceOrigin, FixedQuaternion referenceRotation, out FixedBoundBox relativeBounds,
        out FixedOrientedBox orientedBounds)
    {
        orientedBounds = default;
        Vector2d minimum, maximum;
        Fixed64 rotation = collider.Rotation;
        if (collider is LSCircleCollider2D circle)
        {
            minimum = -Vector2d.One * circle.ScaledRadius;
            maximum = -minimum;
        }
        else if (collider is LSCapsuleCollider2D capsule)
        {
            // Admitted capsule reach bounds every local extent before world clipping.
            FixedBoundArea bounds = FixedBoundArea.FromCenteredRotatedCapsuleClippedToDomain(
                Vector2d.Zero, Fixed64.Zero, capsule.AxisLength, capsule.ScaledRadius);
            minimum = bounds.Min;
            maximum = bounds.Max;
        }
        else
        {
            rotation = collider.ConvexRotation;
            minimum = maximum = collider.GetScaledLocalVertexUnchecked(0);
            for (int i = 1; i < collider.VertexCount; i++)
            {
                Vector2d vertex = collider.GetScaledLocalVertexUnchecked(i);
                minimum = new Vector2d(FixedMath.Min(minimum.X, vertex.X), FixedMath.Min(minimum.Y, vertex.Y));
                maximum = new Vector2d(FixedMath.Max(maximum.X, vertex.X), FixedMath.Max(maximum.Y, vertex.Y));
            }
        }
        // Keep the enclosure centered on the canonical origin, avoiding another
        // rounded world-center transform for asymmetric authored polygons.
        Vector2d negativeMinimum = -minimum;
        Vector3d origin = new(collider.Center.X, collider.MixedSlabCenterY, collider.Center.Y);
        FixedQuaternion orientation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation);
        orientedBounds = new FixedOrientedBox(origin, orientation, new Vector3d(
            FixedMath.Max(negativeMinimum.X, maximum.X), collider.MixedHalfThickness,
            FixedMath.Max(negativeMinimum.Y, maximum.Y)));
        relativeBounds = FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
            origin, orientation,
            new Vector3d(minimum.X, -collider.MixedHalfThickness, minimum.Y),
            new Vector3d(maximum.X, collider.MixedHalfThickness, maximum.Y), referenceOrigin, referenceRotation);
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

    internal static Fixed64 GetCompoundCenteredProxyRadius(LSCompoundCollider2D compound)
    {
        Fixed64 radius = Fixed64.Zero;
        for (int index = 0; index < compound.PartCount; index++)
        {
            var part = compound.GetPartCollider(index);
            // Published centers include scalar transform rounding. Admission uses
            // exact authored attachment reach; the CCD proxy must enclose the
            // actual committed centers instead of reusing that admission metric.
            Vector2d offset = part.CanonicalCenter - compound.CanonicalCenter;
            radius = FixedMath.Max(radius, ColliderGeometryPolicy.Add(
                ColliderGeometryPolicy.Magnitude(offset), part.CanonicalCenteredProxyRadius));
        }
        return radius;
    }

    internal static Fixed64 GetCurrentCompoundCenteredProxyRadius(
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
                GetShapeCenteredProxyRadius(part),
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
            GetCompoundRelativeBounds(compound, compound.CanonicalCenter,
                out Vector2d minimum, out Vector2d maximum);
            return GetMinimumAbsoluteExtent(minimum, maximum);
        }
        if (collider.VertexCount == 0)
            return Fixed64.Zero;

        GetConvexBoundsFromRelativeCenter(collider, Vector2d.Zero,
            out Vector2d convexMinimum, out Vector2d convexMaximum);
        return GetMinimumAbsoluteExtent(convexMinimum, convexMaximum);
    }

    private static void GetCompoundRelativeBounds(LSCompoundCollider2D compound,
        Vector2d referenceCenter, out Vector2d minimum, out Vector2d maximum)
    {
        minimum = maximum = default;
        for (int index = 0; index < compound.PartCount; index++)
        {
            GetRelativeBounds(compound.GetPartCollider(index), referenceCenter,
                out Vector2d partMinimum, out Vector2d partMaximum);
            if (index == 0)
            {
                minimum = partMinimum;
                maximum = partMaximum;
            }
            else
            {
                minimum = new Vector2d(FixedMath.Min(minimum.X, partMinimum.X), FixedMath.Min(minimum.Y, partMinimum.Y));
                maximum = new Vector2d(FixedMath.Max(maximum.X, partMaximum.X), FixedMath.Max(maximum.Y, partMaximum.Y));
            }
        }
    }

    private static void GetRelativeBounds(LSCollider2D collider, Vector2d referenceCenter,
        out Vector2d minimum, out Vector2d maximum)
    {
        // Called only during publication. Admitted part spacing and local reach
        // bound these relative coordinates, even at an absolute scalar face.
        Vector2d relativeCenter = collider.CanonicalCenter - referenceCenter;
        if (collider is LSCircleCollider2D circle)
        {
            GetRoundBounds(relativeCenter, circle.ScaledRadius, circle.ScaledRadius, out minimum, out maximum);
            return;
        }
        if (collider is LSCapsuleCollider2D capsule)
        {
            FixedBoundArea centeredBounds = FixedBoundArea.FromCenteredRotatedCapsuleClippedToDomain(
                Vector2d.Zero, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius);
            GetRoundBounds(relativeCenter, centeredBounds.Max.X, centeredBounds.Max.Y, out minimum, out maximum);
            return;
        }
        GetConvexBoundsFromRelativeCenter(collider, relativeCenter, out minimum, out maximum);
    }

    private static void GetRoundBounds(Vector2d center, Fixed64 extentX, Fixed64 extentY,
        out Vector2d minimum, out Vector2d maximum)
    {
        Vector2d extents = new(extentX, extentY);
        minimum = center - extents;
        maximum = center + extents;
    }

    private static void GetConvexBoundsFromRelativeCenter(LSCollider2D collider, Vector2d relativeCenter,
        out Vector2d minimum, out Vector2d maximum)
    {
        // Admission bounds the local rotated sums; retain the existing fused
        // transform rounding rather than project products independently.
        _ = Vector2d.TryTransformPoint(relativeCenter, collider.GetScaledLocalVertexUnchecked(0),
            collider.ConvexRotation, out minimum);
        maximum = minimum;
        for (int index = 1; index < collider.VertexCount; index++)
        {
            _ = Vector2d.TryTransformPoint(relativeCenter, collider.GetScaledLocalVertexUnchecked(index),
                collider.ConvexRotation, out Vector2d point);
            minimum = new Vector2d(FixedMath.Min(minimum.X, point.X), FixedMath.Min(minimum.Y, point.Y));
            maximum = new Vector2d(FixedMath.Max(maximum.X, point.X), FixedMath.Max(maximum.Y, point.Y));
        }
    }

    private static Fixed64 GetMinimumAbsoluteExtent(
        Vector2d minimum,
        Vector2d maximum) =>
        FixedMath.Min(
            FixedMath.Max(minimum.X.Abs(), maximum.X.Abs()),
            FixedMath.Max(minimum.Y.Abs(), maximum.Y.Abs()));
}
