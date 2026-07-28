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

public class LSCylinderCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedHeight;
    private Vector3d _preparedAxis;
    private Fixed64 _preparedArea;

    public LSCylinderCollider() { }

    public LSCylinderCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cylinder);
        Material = definition.Material;
        Radius = definition.Radius;
        Size = definition.Size;
    }

    public override ColliderType Shape => ColliderType.Cylinder;
    public override int Priority => ColliderSettings.GetPriority(Shape);

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

    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, _size.Y, diameter);
    }

    protected override Vector3d NormalizeSize(Vector3d value) =>
        new(_radius * 2, value.Y, _radius * 2);

    protected internal override FixedMassWeight CalculateMassPropertyWeight() =>
        FixedMassWeight.FromProduct(
            Fixed64.Pi,
            ScaledRadius,
            ScaledRadius,
            Height);

    internal override FixedMassWeight CalculatePreparedMassPropertyWeight() =>
        FixedMassWeight.FromProduct(
            Fixed64.Pi,
            _preparedRadius,
            _preparedRadius,
            _preparedHeight);

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        Fixed64 radiusSqr = ScaledRadiusSqr;
        Fixed64 heightSqr = Height * Height;
        Fixed64 inertiaXZ = Fixed64.FromFraction(1, 12) * mass * ((3 * radiusSqr) + heightSqr);
        Fixed64 inertiaY = Fixed64.Half * mass * radiusSqr;

        Fixed3x3 tensor = new(
            inertiaXZ, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, inertiaY, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, inertiaXZ
        );
        return tensor;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        return worker.CheckCylinderOverlaps(this, ref outputIntersectionPoints);
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        if (!FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                other,
                Center,
                Rotation,
                Vector3d.Up,
                Height,
                ScaledRadius,
                Vector3d.Right,
                out FixedPointAnchor surfaceAnchor,
                out _,
                out _)
            || !surfaceAnchor.TryGetPoint(out Vector3d surfacePoint))
        {
            throw new InvalidOperationException(
                "The closest cylinder surface point is outside the representable coordinate domain.");
        }

        return surfacePoint;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        if (!FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                Center,
                Rotation,
                Vector3d.Up,
                Height,
                ScaledRadius,
                Vector3d.Right,
                out _,
                out Vector3d outwardNormal,
                out _))
        {
            throw new InvalidOperationException(
                "The closest cylinder surface normal is outside the representable coordinate domain.");
        }

        return outwardNormal;
    }

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
