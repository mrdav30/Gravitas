//=======================================================================
// LSCapsuleCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Colliders;

public class LSCapsuleCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedAxisLength;
    private Vector3d _preparedAxis;
    private Fixed64 _preparedArea;

    public LSCapsuleCollider() { }

    public LSCapsuleCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Capsule);
        Material = definition.Material;
        Radius = definition.Radius;
        Size = definition.Size;
    }

    public override ColliderType Shape => ColliderType.Capsule;
    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius => _scaledRadius;

    /// <summary>
    /// Gets the full physical distance between the capsule's hemisphere centers.
    /// </summary>
    public Fixed64 AxisLength { get; private set; }

    /// <summary>
    /// Gets the derived normalized world-space direction of the capsule's
    /// conceptual center axis. Exact geometry remains authoritative in the
    /// collider's rigid frame.
    /// </summary>
    public Vector3d WorldAxis { get; private set; } = Vector3d.Up;

    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, _size.Y, diameter);
    }

    protected override Vector3d NormalizeSize(Vector3d value) =>
        new(_radius * 2, value.Y, _radius * 2);

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.X,
            snapshot.PartScale.X);
        Fixed64 radiusZ = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.Z,
            snapshot.PartScale.Z);
        _preparedRadius = FixedMath.Max(radiusX, radiusZ);
        SwiftThrowHelper.ThrowIfArgument(
            !HasValidScaledDimensions(snapshot),
            nameof(snapshot),
            "Scaled capsule height must be at least the capsule diameter.");
        SwiftThrowHelper.ThrowIfArgument(
            !Fixed64.TryMultiplySubtractClamped(
                snapshot.Size.Y,
                snapshot.OwnerScale.Y,
                snapshot.PartScale.Y,
                Fixed64.One,
                snapshot.Radius,
                Fixed64.Two,
                snapshot.OwnerScale.X,
                snapshot.PartScale.X,
                out Fixed64 axisLengthX)
            | !Fixed64.TryMultiplySubtractClamped(
                snapshot.Size.Y,
                snapshot.OwnerScale.Y,
                snapshot.PartScale.Y,
                Fixed64.One,
                snapshot.Radius,
                Fixed64.Two,
                snapshot.OwnerScale.Z,
                snapshot.PartScale.Z,
                out Fixed64 axisLengthZ),
            nameof(snapshot),
            "Scaled capsule center-axis length must be representable.");
        _preparedAxisLength = FixedMath.Min(axisLengthX, axisLengthZ);
        _preparedAxis = (snapshot.Rotation * Vector3d.Up).Normalized;
        _preparedArea = Fixed64.Two * Fixed64.Pi * _preparedRadius * _preparedAxisLength
            + Fixed64.Two * Fixed64.Pi * _preparedRadius * _preparedRadius;
        SetPreparedBounds(FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            snapshot.Center,
            snapshot.Rotation,
            Vector3d.Up,
            _preparedAxisLength,
            _preparedRadius));
    }

    private static bool HasValidScaledDimensions(
        in ColliderShapeSnapshot snapshot) =>
        Fixed64.CompareProducts(
            snapshot.Size.Y,
            snapshot.OwnerScale.Y,
            snapshot.PartScale.Y,
            Fixed64.One,
            snapshot.Radius,
            Fixed64.Two,
            snapshot.OwnerScale.X,
            snapshot.PartScale.X) >= 0
        & Fixed64.CompareProducts(
            snapshot.Size.Y,
            snapshot.OwnerScale.Y,
            snapshot.PartScale.Y,
            Fixed64.One,
            snapshot.Radius,
            Fixed64.Two,
            snapshot.OwnerScale.Z,
            snapshot.PartScale.Z) >= 0;

    private protected override void PublishShape()
    {
        _scaledRadius = _preparedRadius;
        AxisLength = _preparedAxisLength;
        WorldAxis = _preparedAxis;
        Area = _preparedArea;
    }

    protected internal override Fixed64 CalculateMassPropertyWeight() =>
        Fixed64.Pi * ScaledRadiusSqr * AxisLength
        + Fixed64.FromFraction(4, 3) * Fixed64.Pi * ScaledRadiusSqr * ScaledRadius;

    // The capsule is split into a cylinder and a pair of solid hemispheres with masses
    // proportional to their volumes. Each hemisphere's centroid lies 3r/8 outward from
    // its sphere center, which contributes the 3dr/4 cross term to transverse cap inertia.
    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        if (AxisLength <= Fixed64.Epsilon)
        {
            Fixed64 sphereDiagonal = Fixed64.FromFraction(2, 5) * mass * ScaledRadiusSqr;
            Fixed3x3 sphereTensor = new(
                sphereDiagonal, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, sphereDiagonal, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, sphereDiagonal
            );
            return ShiftInertiaTensorFromLocalCenterOfMass(sphereTensor, mass, localCenterOfMassOffset);
        }

        // Masses of the cylinder and spheres (proportional to their volumes)
        Fixed64 cylinderVolume = Fixed64.Pi * ScaledRadiusSqr * AxisLength;
        Fixed64 sphereVolume = Fixed64.FromFraction(4, 3) * Fixed64.Pi * ScaledRadiusSqr * ScaledRadius;
        Fixed64 totalVolume = cylinderVolume + sphereVolume;
        if (totalVolume <= Fixed64.Zero)
        {
            // Fixed-point scaling can quantize a positive radius and both volumes to zero.
            // The remaining shape is the zero-radius limit: a thin rod along local Y.
            Fixed64 rodInertiaXZ = Fixed64.FromFraction(1, 12) * mass * AxisLength * AxisLength;
            Fixed3x3 rodTensor = new(
                rodInertiaXZ, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, rodInertiaXZ
            );
            return ShiftInertiaTensorFromLocalCenterOfMass(rodTensor, mass, localCenterOfMassOffset);
        }

        Fixed64 cylinderMass = mass * (cylinderVolume / totalVolume);
        Fixed64 sphereMass = mass - cylinderMass;

        // Distance from the center of the hemisphere to the center of the capsule
        Fixed64 d = AxisLength / 2;

        // Calculating the inertia tensors for the cylinder and the spheres
        Fixed64 cylinderInertiaY = Fixed64.FromFraction(1, 2) * cylinderMass * ScaledRadiusSqr;
        Fixed64 cylinderInertiaXZ = Fixed64.FromFraction(1, 12) * cylinderMass * ((3 * ScaledRadiusSqr) + (AxisLength * AxisLength));
        Fixed64 sphereInertiaXZ = sphereMass
            * (Fixed64.FromFraction(2, 5) * ScaledRadiusSqr
                + d * d
                + Fixed64.FromFraction(3, 4) * d * ScaledRadius);
        Fixed64 sphereInertiaY = Fixed64.FromFraction(2, 5) * sphereMass * ScaledRadiusSqr;

        // The total inertia tensor for the capsule
        Fixed64 totalInertia_xz = cylinderInertiaXZ + sphereInertiaXZ;
        Fixed64 totalInertia_y = cylinderInertiaY + sphereInertiaY;

        Fixed3x3 tensor = new(
            totalInertia_xz, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, totalInertia_y, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, totalInertia_xz
        );
        return ShiftInertiaTensorFromLocalCenterOfMass(tensor, mass, localCenterOfMassOffset);
    }

    // If the capsule is moving in the direction of its main axis,
    // the frontal area would be a circle (the end cap of the capsule).
    // Therefore, the frontal area would be πr^2, where r is the radius of the capsule.
    // If it's moving perpendicular to its main axis,
    // then the frontal area would be a rectangle with a semicircle on either end,
    // which would be (2r)*h + πr^2, where h is the height of the cylindrical part of the capsule.
    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 directionMagnitude = direction.Magnitude;
        if (directionMagnitude <= Fixed64.Epsilon)
            return Area;

        Vector3d normalizedDirection = direction / directionMagnitude;
        Fixed64 axial = Rotation.Inverse()
            .Rotate(normalizedDirection).Y.Abs();
        Fixed64 radialFactorSqr = Fixed64.One - axial * axial;
        Fixed64 radialFactor = radialFactorSqr <= Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Sqrt(radialFactorSqr);

        Fixed64 capArea = Fixed64.Pi * ScaledRadiusSqr;
        Fixed64 sideProfile = 2 * ScaledRadius * AxisLength;
        return capArea + radialFactor * sideProfile;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        if (!FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                other,
                Center,
                Rotation,
                Vector3d.Up,
                AxisLength,
                ScaledRadius,
                Vector3d.Right,
                out FixedPointAnchor surfaceAnchor,
                out _,
                out _)
            || !surfaceAnchor.TryGetPoint(out Vector3d surfacePoint))
        {
            throw new InvalidOperationException(
                "The closest capsule surface point is outside the representable coordinate domain.");
        }

        return surfacePoint;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        if (!FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                point,
                Center,
                Rotation,
                Vector3d.Up,
                AxisLength,
                ScaledRadius,
                Vector3d.Right,
                out _,
                out Vector3d outwardNormal,
                out _))
        {
            throw new InvalidOperationException(
                "The closest capsule surface normal is outside the representable coordinate domain.");
        }

        return outwardNormal;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckCapsuleOverlaps(this, ref outputIntersectionPoints);
}
