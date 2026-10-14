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

/// <summary>Represents a runtime 3D capsule collider whose local axis follows Y.</summary>
public sealed class LSCapsuleCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedAxisLength;
    private Vector3d _preparedAxis;
    private Fixed64 _preparedArea;

    /// <summary>Creates a capsule collider with default dimensions.</summary>
    public LSCapsuleCollider() { }

    /// <summary>Creates a runtime capsule from an authored shape definition.</summary>
    public LSCapsuleCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Capsule);
        Material = definition.Material;
        Radius = definition.Radius;
        Size = definition.Size;
    }

    /// <inheritdoc/>
    public override ColliderType Shape => ColliderType.Capsule;
    /// <inheritdoc/>
    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <inheritdoc/>
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

    /// <inheritdoc/>
    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, _size.Y, diameter);
    }

    /// <inheritdoc/>
    protected override Vector3d NormalizeSize(Vector3d value) =>
        new(_radius * 2, value.Y, _radius * 2);

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        GetScaledGeometry(snapshot, out _preparedRadius, out _preparedAxisLength);
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

    private static void GetScaledGeometry(
        in ColliderShapeSnapshot snapshot,
        out Fixed64 radius,
        out Fixed64 axisLength)
    {
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.X,
            snapshot.PartScale.X);
        Fixed64 radiusZ = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.Z,
            snapshot.PartScale.Z);
        radius = FixedMath.Max(radiusX, radiusZ);
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
        axisLength = FixedMath.Min(axisLengthX, axisLengthZ);
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

    internal override ExactMassWeight CalculateMassPropertyWeight()
    {
        GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 axisLength);
        return GetCylinderWeight(radius, axisLength).Add(GetCapWeight(radius));
    }

    internal void GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 axisLength)
    {
        if (HasCommittedShape)
        {
            radius = _scaledRadius;
            axisLength = AxisLength;
            return;
        }

        GetCurrentShapeScales(out Vector3d ownerScale, out Vector3d partScale);
        var snapshot = new ColliderShapeSnapshot(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            ownerScale,
            partScale,
            LocalOffset,
            Size,
            Radius);
        GetScaledGeometry(snapshot, out radius, out axisLength);
    }

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight() =>
        GetCylinderWeight(_preparedRadius, _preparedAxisLength)
            .Add(GetCapWeight(_preparedRadius));

    // The capsule is split into a cylinder and a pair of solid hemispheres with masses
    // proportional to their volumes. Each hemisphere's centroid lies 3r/8 outward from
    // its sphere center, which contributes the 3dr/4 cross term to transverse cap inertia.
    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 axisLength);
        Signed192 sphereAxial = ExactMassProperties.InertiaGeometryTerm(
            Fixed64.FromFraction(2, 5), radius, radius);
        if (axisLength <= Fixed64.Epsilon)
        {
            Fixed64 sphereDiagonal = ExactMassProperties.InertiaMoment(mass, sphereAxial);
            return new Fixed3x3(
                sphereDiagonal, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, sphereDiagonal, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, sphereDiagonal);
        }

        // Cancelling the shared r^2 retains the existing volume coefficients
        // while preserving the exact internal cylinder/hemisphere mass ratio.
        Signed192 cylinderWeight = Fixed64.GetExactRawProduct(Fixed64.Pi, axisLength);
        Signed192 capWeight = Fixed64.GetExactRawProduct(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi, radius);
        GetComponentInertiaGeometry(radius, axisLength, sphereAxial,
            out Signed192 cylinderTransverse, out Signed192 sphereTransverse);
        Fixed64 totalInertia_xz = ExactMassProperties.InertiaWeightedMoment(
            mass, cylinderTransverse, cylinderWeight, sphereTransverse, capWeight);
        Fixed64 totalInertia_y = ExactMassProperties.InertiaWeightedMoment(
            mass, ExactMassProperties.InertiaGeometryTerm(Fixed64.Half, radius, radius), cylinderWeight,
            sphereAxial, capWeight);

        Fixed3x3 tensor = new(
            totalInertia_xz, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, totalInertia_y, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, totalInertia_xz
        );
        return tensor;
    }

    internal static void GetComponentInertiaGeometry(
        Fixed64 radius, Fixed64 axisLength, Signed192 sphereAxial,
        out Signed192 cylinderTransverse, out Signed192 sphereTransverse)
    {
        Fixed64 twelfth = Fixed64.FromFraction(1, 12);
        cylinderTransverse = WideArithmetic.AddSigned192(
            ExactMassProperties.InertiaGeometryTerm(twelfth * (Fixed64)3, radius, radius),
            ExactMassProperties.InertiaGeometryTerm(twelfth, axisLength, axisLength));
        sphereTransverse = WideArithmetic.AddSigned192(sphereAxial,
            WideArithmetic.AddSigned192(
                ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(1, 4), axisLength, axisLength),
                ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(3, 8), axisLength, radius)));
    }

    internal static ExactMassWeight GetCylinderWeight(
        Fixed64 radius,
        Fixed64 axisLength) =>
        ExactMassWeight.FromProduct(
            Fixed64.Pi,
            radius,
            radius,
            axisLength);

    internal static ExactMassWeight GetCapWeight(Fixed64 radius) =>
        ExactMassWeight.FromProduct(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi,
            radius,
            radius,
            radius);

    // If the capsule is moving in the direction of its main axis,
    // the frontal area would be a circle (the end cap of the capsule).
    // Therefore, the frontal area would be πr^2, where r is the radius of the capsule.
    // If it's moving perpendicular to its main axis,
    // then the frontal area would be a rectangle with a semicircle on either end,
    // which would be (2r)*h + πr^2, where h is the height of the cylindrical part of the capsule.
    /// <inheritdoc/>
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

    /// <inheritdoc/>
    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        FixedPointAnchor surfaceAnchor =
            GetClosestSurfaceAnchor(other, out _);
        if (!surfaceAnchor.TryGetPoint(out Vector3d surfacePoint))
        {
            throw new InvalidOperationException(
                "The closest capsule surface point is outside the representable coordinate domain.");
        }

        return surfacePoint;
    }

    /// <inheritdoc/>
    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        _ = GetClosestSurfaceAnchor(
            point,
            out Vector3d outwardNormal);
        return outwardNormal;
    }

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point,
        out Vector3d normal) =>
        GetClosestSurfaceAnchor(point, out normal, out _);

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point, out Vector3d normal, out bool contained) =>
        WideFiniteAxisIntersection
            .GetClosestCenteredCapsuleSurfaceAnchor(
                point,
                Center,
                Rotation,
                Vector3d.Up,
                AxisLength,
                ScaledRadius,
                Vector3d.Right,
                out normal,
                out contained);

    /// <inheritdoc/>
    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckCapsuleOverlaps(this, ref outputIntersectionPoints);
}
