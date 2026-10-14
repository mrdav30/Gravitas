//=======================================================================
// LSCylinderCollider.cs
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

/// <summary>Represents a finite runtime 3D cylinder whose local axis follows Y.</summary>
public sealed class LSCylinderCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedHeight;
    private Vector3d _preparedAxis;
    private Fixed64 _preparedArea;

    /// <summary>Creates a cylinder collider with default dimensions.</summary>
    public LSCylinderCollider() { }

    /// <summary>Creates a runtime cylinder from an authored shape definition.</summary>
    public LSCylinderCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cylinder);
        Material = definition.Material;
        Radius = definition.Radius;
        Size = definition.Size;
    }

    /// <inheritdoc/>
    public override ColliderType Shape => ColliderType.Cylinder;
    /// <inheritdoc/>
    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <inheritdoc/>
    public override Fixed64 ScaledRadius => _scaledRadius;

    /// <summary>
    /// Gets the cylinder's full physical length along <see cref="WorldAxis"/>.
    /// </summary>
    public Fixed64 Height { get; private set; }

    /// <summary>
    /// Gets the derived normalized world-space cylinder axis. Exact geometry
    /// remains authoritative in the collider's rigid frame.
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

    internal override ExactMassWeight CalculateMassPropertyWeight()
    {
        GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 height);
        return ExactMassWeight.FromProduct(
            Fixed64.Pi,
            radius,
            radius,
            height);
    }

    private void GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 height)
    {
        if (HasCommittedShape)
        {
            radius = _scaledRadius;
            height = Height;
            return;
        }

        GetCurrentShapeScales(out Vector3d ownerScale, out Vector3d partScale);
        radius = FixedMath.Max(
            ColliderScalePolicy.ScalePositive(Radius, ownerScale.X, partScale.X),
            ColliderScalePolicy.ScalePositive(Radius, ownerScale.Z, partScale.Z));
        height = ColliderScalePolicy.ScalePositive(Size.Y, ownerScale.Y, partScale.Y);
    }

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight() =>
        ExactMassWeight.FromProduct(
            Fixed64.Pi,
            _preparedRadius,
            _preparedRadius,
            _preparedHeight);

    internal void GetInertiaGeometry(out Signed192 transverse, out Signed192 axial)
    {
        GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 height);
        Fixed64 twelfth = Fixed64.FromFraction(1, 12);
        transverse = WideArithmetic.AddSigned192(
            ExactMassProperties.InertiaGeometryTerm(twelfth * (Fixed64)3, radius, radius),
            ExactMassProperties.InertiaGeometryTerm(twelfth, height, height));
        axial = ExactMassProperties.InertiaGeometryTerm(Fixed64.Half, radius, radius);
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        GetInertiaGeometry(out Signed192 transverse, out Signed192 axial);
        Fixed64 inertiaXZ = ExactMassProperties.InertiaMoment(mass, transverse);
        Fixed64 inertiaY = ExactMassProperties.InertiaMoment(mass, axial);

        Fixed3x3 tensor = new(
            inertiaXZ, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, inertiaY, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, inertiaXZ
        );
        return tensor;
    }

    /// <inheritdoc/>
    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        return worker.CheckCylinderOverlaps(this, ref outputIntersectionPoints);
    }

    /// <inheritdoc/>
    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        FixedPointAnchor surfaceAnchor =
            GetClosestSurfaceAnchor(other, out _);
        if (!surfaceAnchor.TryGetPoint(out Vector3d surfacePoint))
        {
            throw new InvalidOperationException(
                "The closest cylinder surface point is outside the representable coordinate domain.");
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
            .GetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                Center,
                Rotation,
                Vector3d.Up,
                Height,
                ScaledRadius,
                Vector3d.Right,
                out normal,
                out contained);

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
        _preparedHeight = ColliderScalePolicy.ScalePositive(
            snapshot.Size.Y,
            snapshot.OwnerScale.Y,
            snapshot.PartScale.Y);
        _preparedAxis = (snapshot.Rotation * Vector3d.Up).Normalized;
        _preparedArea = Fixed64.Two * Fixed64.Pi * _preparedRadius
            * (_preparedHeight + _preparedRadius);
        SetPreparedBounds(FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
            snapshot.Center,
            snapshot.Rotation,
            Vector3d.Up,
            _preparedHeight,
            _preparedRadius));
    }

    private protected override void PublishShape()
    {
        _scaledRadius = _preparedRadius;
        Height = _preparedHeight;
        WorldAxis = _preparedAxis;
        Area = _preparedArea;
    }

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
        Fixed64 radialFactor = radialFactorSqr <= Fixed64.Zero ? Fixed64.Zero : FixedMath.Sqrt(radialFactorSqr);

        return axial * Fixed64.Pi * ScaledRadiusSqr
            + radialFactor * 2 * ScaledRadius * Height;
    }

}
