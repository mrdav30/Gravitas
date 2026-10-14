//=======================================================================
// LSConeCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Colliders;

/// <summary>
/// Represents a finite circular cone whose local Y axis runs from base plane
/// to apex and whose local origin is the cone's bounding center.
/// </summary>
public sealed class LSConeCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedHeight;
    private Vector3d _preparedAxis;
    private Fixed64 _preparedVolume;

    /// <summary>Creates a cone collider with default dimensions.</summary>
    public LSConeCollider() { }

    /// <summary>Creates a runtime cone from an authored shape definition.</summary>
    public LSConeCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cone);
        Material = definition.Material;
        Radius = definition.Radius;
        Size = definition.Size;
    }

    /// <inheritdoc/>
    public override ColliderType Shape => ColliderType.Cone;

    /// <inheritdoc/>
    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <inheritdoc/>
    public override Fixed64 ScaledRadius => _scaledRadius;

    /// <summary>
    /// Gets the derived normalized world-space base-to-apex axis. Exact
    /// geometry remains authoritative in the collider's rigid frame.
    /// </summary>
    public Vector3d WorldAxis { get; private set; } = Vector3d.Up;

    /// <summary>
    /// Gets the full physical distance from the cone's base plane to its apex.
    /// </summary>
    public Fixed64 Height { get; private set; }

    /// <summary>
    /// Gets the solid cone volume, rounded once after final division and
    /// saturated to <see cref="Fixed64.MaxValue"/> when unrepresentable.
    /// Compound mass weighting retains its separate unnarrowed measure.
    /// </summary>
    public Fixed64 Volume { get; private set; }

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
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.X,
            snapshot.PartScale.X);
        Fixed64 radiusZ = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.Z,
            snapshot.PartScale.Z);
        Fixed64 radius = FixedMath.Max(radiusX, radiusZ);
        Fixed64 height = ColliderScalePolicy.ScalePositive(
            snapshot.Size.Y,
            snapshot.OwnerScale.Y,
            snapshot.PartScale.Y);
        Fixed64 volume;
        if (HasCommittedShape && radius == ScaledRadius && height == Height)
        {
            volume = Volume;
        }
        else if (!Fixed64.TryMultiplyDivideBySum(
            Fixed64.Pi, radius, radius, height,
            (Fixed64)3, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            out volume))
        {
            // Saturate only the final diagnostic measure; valid geometry and
            // exact compound mass weights need not fit a scalar volume.
            volume = Fixed64.MaxValue;
        }
        _preparedRadius = radius;
        _preparedHeight = height;
        _preparedVolume = volume;
        _preparedAxis = HasCommittedShape && snapshot.Rotation == CanonicalRotation
            ? WorldAxis
            : (snapshot.Rotation * Vector3d.Up).Normalized;
        // Body-bound shapes are checked by the shared shape transaction.
        if (CompoundOwner == null
            && Body == null
            && !CalculatePreparedLocalMassPoint().TryGetPoint(out _))
        {
            throw new InvalidOperationException(
                "Prepared collider mass-property point is outside the Fixed64 coordinate domain.");
        }
        SetPreparedBounds(FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
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
        Volume = _preparedVolume;
        Area = _preparedVolume;
    }

    internal override ExactMassWeight CalculateMassPropertyWeight()
    {
        GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 height);
        return ExactMassWeight.FromProduct(
            Fixed64.Pi / (Fixed64)3,
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
            Fixed64.Pi / (Fixed64)3,
            _preparedRadius,
            _preparedRadius,
            _preparedHeight);

    internal override ExactMassPoint3D CalculateLocalMassPoint() =>
        TransformRelativeMassPropertyPointExact(
            GetLocalCenterOfMass(
                HasCommittedShape
                    ? Height
                    : GetCurrentHeight()));

    internal override ExactMassPoint3D CalculatePreparedLocalMassPoint() =>
        TransformPreparedRelativeMassPropertyPointExact(
            GetLocalCenterOfMass(_preparedHeight));

    private Fixed64 GetCurrentHeight()
    {
        GetCurrentShapeScales(
            out Vector3d ownerScale,
            out Vector3d partScale);
        return ColliderScalePolicy.ScalePositive(
            Size.Y,
            ownerScale.Y,
            partScale.Y);
    }

    private static Vector3d GetLocalCenterOfMass(Fixed64 height) =>
        new(
            Fixed64.Zero,
            -height * Fixed64.FromFraction(1, 4),
            Fixed64.Zero);

    internal void GetInertiaGeometry(out Signed192 transverse, out Signed192 axial)
    {
        GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 height);
        transverse = WideArithmetic.AddSigned192(
            ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(3, 20), radius, radius),
            ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(3, 80), height, height));
        axial = ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(3, 10), radius, radius);
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        GetInertiaGeometry(out Signed192 transverse, out Signed192 axial);
        Fixed64 inertiaXZ = ExactMassProperties.InertiaMoment(mass, transverse);
        Fixed64 inertiaY = ExactMassProperties.InertiaMoment(mass, axial);

        Fixed3x3 tensor = new(
            inertiaXZ, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, inertiaY, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, inertiaXZ);
        return tensor;
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

        Fixed64 baseArea = Fixed64.Pi * ScaledRadiusSqr;
        Fixed64 triangularProfile = ScaledRadius * Height;
        return axial * baseArea + radialFactor * triangularProfile;
    }

    /// <inheritdoc/>
    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        FixedPointAnchor surfaceAnchor =
            GetClosestSurfaceAnchor(other, out _);
        if (!surfaceAnchor.TryGetPoint(out Vector3d surfacePoint))
        {
            throw new InvalidOperationException(
                "The closest cone surface point is outside the representable coordinate domain.");
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
            .GetClosestCenteredFiniteConeSurfaceAnchor(
                point,
                Center,
                Rotation,
                Vector3d.Up,
                Height,
                ScaledRadius,
                Vector3d.Right,
                out normal,
                out contained);

    /// <inheritdoc/>
    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckConeOverlaps(this, ref outputIntersectionPoints);

    internal bool ContainsWorldPoint(
        Vector3d point,
        Fixed64 tolerance = default) =>
        FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
            point,
            Center,
            Rotation,
            Vector3d.Up,
            Height,
            ScaledRadius,
            Vector3d.Right,
            out _,
            out _,
            out Fixed64 signedDistance)
        && signedDistance <= tolerance;
}
