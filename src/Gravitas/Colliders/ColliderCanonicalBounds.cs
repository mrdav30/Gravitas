//=======================================================================
// ColliderCanonicalBounds.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>
/// Contains methods for computing canonical bounds of colliders, 
/// including proxy radii and relative bounds, based on collider type and properties.
/// </summary>
internal static class ColliderCanonicalBounds
{
    internal static Fixed64 GetCurrentCenteredProxyRadius(
        LSCollider collider)
    {
        switch (collider)
        {
            case LSSphereCollider:
                return collider.GetCurrentScaledRadius();
            case LSCapsuleCollider capsule:
                return GetCurrentCapsuleCenteredProxyRadius(capsule);
            case LSCuboidCollider cuboid:
                return GetCurrentCuboidCenteredProxyRadius(cuboid);
            case LSCylinderCollider cylinder:
                return GetCurrentFiniteAxisCenteredProxyRadius(
                    cylinder,
                    cylinder.Size.Y);
            case LSConeCollider cone:
                return GetCurrentFiniteAxisCenteredProxyRadius(
                    cone,
                    cone.Size.Y);
            case LSMeshCollider mesh:
                {
                    mesh.GetCurrentShapeScales(
                        out Vector3d ownerScale,
                        out Vector3d partScale);
                    return mesh.Mesh.GetScaledLocalRadius(
                        ownerScale,
                        partScale);
                }
            case LSCompoundCollider compound:
                return GetCurrentCompoundCenteredProxyRadius(
                    compound);
            default:
                return Fixed64.MaxValue;
        }
    }

    internal static FixedBoundBox GetRelativeBounds(
        LSCollider collider,
        Vector3d referenceOrigin,
        FixedQuaternion referenceRotation = default)
    {
        if (referenceRotation == default)
            referenceRotation = FixedQuaternion.Identity;
        if (collider is LSCompoundCollider compound)
        {
            return GetCompoundRelativeBounds(
                compound,
                referenceOrigin,
                referenceRotation);
        }
        if (!TryGetLocalBounds(
            collider,
            out Vector3d localMin,
            out Vector3d localMax))
        {
            return FixedBoundBox.FromMinMax(
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.MinValue,
                    Fixed64.MinValue),
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    Fixed64.MaxValue));
        }
        Vector3d sourceOrigin = collider is LSMeshCollider mesh
            ? mesh.Mesh.Origin
            : collider.CanonicalCenter;
        FixedQuaternion sourceRotation = collider is LSMeshCollider meshCollider
            ? meshCollider.Mesh.Rotation
            : collider.CanonicalRotation;
        return FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
            sourceOrigin,
            sourceRotation,
            localMin,
            localMax,
            referenceOrigin,
            referenceRotation);
    }

    internal static Fixed64 GetCenteredProxyRadius(
        LSCollider collider)
    {
        switch (collider)
        {
            case LSSphereCollider sphere:
                return sphere.ScaledRadius;
            case LSCapsuleCollider capsule:
                {
                    FixedBoundBox localBounds =
                        FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                            Vector3d.Zero,
                            Vector3d.Up,
                            capsule.AxisLength,
                            capsule.ScaledRadius);
                    return localBounds.Max.Y;
                }
            case LSCuboidCollider cuboid:
                return cuboid.OrientedBox.HalfExtents
                    .TryGetMagnitudeCeiling(
                        out Fixed64 cuboidRadius)
                    ? cuboidRadius
                    : Fixed64.MaxValue;
            case LSCylinderCollider cylinder:
                return GetFiniteAxisLocalRadius(
                    FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                        Vector3d.Zero,
                        Vector3d.Up,
                        cylinder.Height,
                        cylinder.ScaledRadius));
            case LSConeCollider cone:
                return GetFiniteAxisLocalRadius(
                    FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                        Vector3d.Zero,
                        Vector3d.Up,
                        cone.Height,
                        cone.ScaledRadius));
            case LSMeshCollider mesh:
                return mesh.Mesh.ScaledLocalRadius;
            case LSCompoundCollider compound:
                return GetCompoundCenteredProxyRadius(compound);
            default:
                return Fixed64.MaxValue;
        }
    }

    internal static Fixed64 GetGroundProbeRadius(
        LSCollider collider)
    {
        if (collider is LSSphereCollider sphere)
            return sphere.ScaledRadius;
        if (collider is LSCapsuleCollider capsule)
            return capsule.ScaledRadius;
        if (collider is LSCylinderCollider cylinder)
            return cylinder.ScaledRadius;
        if (collider is not (LSCuboidCollider or LSCompoundCollider))
            return Fixed64.Zero;

        Vector3d extents = GetMaximumAbsoluteExtents(
            collider,
            collider.CanonicalCenter);
        return FixedMath.Min(extents.X, extents.Z);
    }

    internal static Vector3d GetMaximumAbsoluteExtents(
        LSCollider collider,
        Vector3d referenceOrigin)
    {
        FixedBoundBox bounds = GetRelativeBounds(
            collider,
            referenceOrigin);
        return new Vector3d(
            FixedMath.Max(bounds.Min.X.Abs(), bounds.Max.X.Abs()),
            FixedMath.Max(bounds.Min.Y.Abs(), bounds.Max.Y.Abs()),
            FixedMath.Max(bounds.Min.Z.Abs(), bounds.Max.Z.Abs()));
    }

    private static FixedBoundBox GetCompoundRelativeBounds(
        LSCompoundCollider compound,
        Vector3d referenceOrigin,
        FixedQuaternion referenceRotation)
    {
        FixedBoundBox bounds = GetRelativeBounds(
            compound.GetPartCollider(0),
            referenceOrigin,
            referenceRotation);
        Vector3d minimum = bounds.Min;
        Vector3d maximum = bounds.Max;
        for (int index = 1; index < compound.PartCount; index++)
        {
            bounds = GetRelativeBounds(
                compound.GetPartCollider(index),
                referenceOrigin,
                referenceRotation);
            minimum = Vector3d.Min(minimum, bounds.Min);
            maximum = Vector3d.Max(maximum, bounds.Max);
        }

        return FixedBoundBox.FromMinMax(minimum, maximum);
    }

    private static bool TryGetLocalBounds(
        LSCollider collider,
        out Vector3d minimum,
        out Vector3d maximum)
    {
        Vector3d halfExtents;
        switch (collider)
        {
            case LSSphereCollider sphere:
                halfExtents = Vector3d.One * sphere.ScaledRadius;
                break;
            case LSCapsuleCollider capsule:
                {
                    FixedBoundBox bounds =
                        FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                            Vector3d.Zero,
                            Vector3d.Up,
                            capsule.AxisLength,
                            capsule.ScaledRadius);
                    minimum = bounds.Min;
                    maximum = bounds.Max;
                    return true;
                }
            case LSCuboidCollider cuboid:
                halfExtents = cuboid.OrientedBox.HalfExtents;
                break;
            case LSCylinderCollider cylinder:
                {
                    FixedBoundBox bounds =
                        FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                            Vector3d.Zero,
                            Vector3d.Up,
                            cylinder.Height,
                            cylinder.ScaledRadius);
                    minimum = bounds.Min;
                    maximum = bounds.Max;
                    return true;
                }
            case LSConeCollider cone:
                {
                    FixedBoundBox bounds =
                        FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                            Vector3d.Zero,
                            Vector3d.Up,
                            cone.Height,
                            cone.ScaledRadius);
                    minimum = bounds.Min;
                    maximum = bounds.Max;
                    return true;
                }
            case LSMeshCollider mesh:
                minimum = mesh.Mesh.ScaledLocalBounds.Min;
                maximum = mesh.Mesh.ScaledLocalBounds.Max;
                return true;
            default:
                minimum = default;
                maximum = default;
                return false;
        }

        minimum = -halfExtents;
        maximum = halfExtents;
        return true;
    }

    private static Fixed64 GetFiniteAxisLocalRadius(
        FixedBoundBox localBounds)
    {
        Vector3d extents = new(
            FixedMath.Max(localBounds.Min.X.Abs(), localBounds.Max.X.Abs()),
            FixedMath.Max(localBounds.Min.Y.Abs(), localBounds.Max.Y.Abs()),
            FixedMath.Max(localBounds.Min.Z.Abs(), localBounds.Max.Z.Abs()));
        return extents.TryGetMagnitudeCeiling(
            out Fixed64 radius)
            ? radius
            : Fixed64.MaxValue;
    }

    private static Fixed64 GetCurrentCuboidCenteredProxyRadius(
        LSCuboidCollider cuboid)
    {
        cuboid.GetCurrentShapeScales(
            out Vector3d ownerScale,
            out Vector3d partScale);
        Vector3d halfExtents =
            ColliderScalePolicy.ScalePositive(
                cuboid.Size,
                ownerScale,
                partScale,
                Fixed64.Two);
        return halfExtents.TryGetMagnitudeCeiling(
            out Fixed64 radius)
            ? radius
            : Fixed64.MaxValue;
    }

    private static Fixed64 GetCurrentCapsuleCenteredProxyRadius(
        LSCapsuleCollider capsule)
    {
        capsule.GetCurrentShapeScales(
            out Vector3d ownerScale,
            out Vector3d partScale);
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            capsule.Radius,
            ownerScale.X,
            partScale.X);
        Fixed64 radiusZ = ColliderScalePolicy.ScalePositive(
            capsule.Radius,
            ownerScale.Z,
            partScale.Z);
        if (!Fixed64.TryMultiplySubtractClamped(
                capsule.Size.Y,
                ownerScale.Y,
                partScale.Y,
                Fixed64.One,
                capsule.Radius,
                Fixed64.Two,
                ownerScale.X,
                partScale.X,
                out Fixed64 axisLengthX)
            || !Fixed64.TryMultiplySubtractClamped(
                capsule.Size.Y,
                ownerScale.Y,
                partScale.Y,
                Fixed64.One,
                capsule.Radius,
                Fixed64.Two,
                ownerScale.Z,
                partScale.Z,
                out Fixed64 axisLengthZ))
        {
            return Fixed64.MaxValue;
        }

        FixedBoundBox localBounds =
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero,
                Vector3d.Up,
                FixedMath.Min(axisLengthX, axisLengthZ),
                FixedMath.Max(radiusX, radiusZ));
        return localBounds.Max.Y;
    }

    private static Fixed64 GetCurrentFiniteAxisCenteredProxyRadius(
        LSCollider collider,
        Fixed64 authoredHeight)
    {
        collider.GetCurrentShapeScales(
            out Vector3d ownerScale,
            out Vector3d partScale);
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            collider.Radius,
            ownerScale.X,
            partScale.X);
        Fixed64 radiusZ = ColliderScalePolicy.ScalePositive(
            collider.Radius,
            ownerScale.Z,
            partScale.Z);
        Fixed64 height = ColliderScalePolicy.ScalePositive(
            authoredHeight,
            ownerScale.Y,
            partScale.Y);
        return GetFiniteAxisLocalRadius(
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                Vector3d.Zero,
                Vector3d.Up,
                height,
                FixedMath.Max(radiusX, radiusZ)));
    }

    private static Fixed64 GetCompoundCenteredProxyRadius(
        LSCompoundCollider compound)
    {
        Fixed64 bestRadius = Fixed64.Zero;
        for (int index = 0; index < compound.PartCount; index++)
        {
            LSCollider part = compound.GetPartCollider(index);
            bool offsetResolved = Vector3d.TrySubtract(
                part.CanonicalCenter,
                compound.CanonicalCenter,
                out Vector3d offset);
            bool distanceResolved =
                offset.TryGetMagnitudeCeiling(
                    out Fixed64 distance);
            bool radiusResolved = Fixed64.TryAdd(
                distance,
                part.CanonicalCenteredProxyRadius,
                out Fixed64 radius);
            if (!(offsetResolved & distanceResolved & radiusResolved))
                return Fixed64.MaxValue;

            bestRadius = FixedMath.Max(bestRadius, radius);
        }

        return bestRadius;
    }

    private static Fixed64 GetCurrentCompoundCenteredProxyRadius(
        LSCompoundCollider compound)
    {
        Fixed64 bestRadius = Fixed64.Zero;
        for (int index = 0; index < compound.PartCount; index++)
        {
            LSCollider part = compound.GetPartCollider(index);
            bool distanceResolved =
                part.TryGetCurrentScaledOffset(
                    out Vector3d centerOffset)
                & centerOffset.TryGetMagnitudeCeiling(
                    out Fixed64 distance);
            bool radiusResolved = Fixed64.TryAdd(
                distance,
                GetCurrentCenteredProxyRadius(part),
                out Fixed64 radius);
            if (!(distanceResolved
                & radiusResolved))
            {
                return Fixed64.MaxValue;
            }

            bestRadius = FixedMath.Max(bestRadius, radius);
        }

        return bestRadius;
    }
}
