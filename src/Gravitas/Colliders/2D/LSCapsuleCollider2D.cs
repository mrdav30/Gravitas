//=======================================================================
// LSCapsuleCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D capsule collider whose local center segment follows local 2D Y.
/// </summary>
public sealed class LSCapsuleCollider2D : LSCollider2D
{
    private Fixed64 _radius;
    private Fixed64 _height;
    private Fixed64 _scaledRadius;
    private Fixed64 _axisLength;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedAxisLength;
    private Vector2d _preparedAxis;

    public LSCapsuleCollider2D(Fixed64 radius, Fixed64 height)
    {
        ValidateDimensions(radius, height);
        _radius = radius;
        _height = height;
        MarkShapeDirty();
    }

    public LSCapsuleCollider2D(ColliderShapeDefinition2D definition)
    {
        definition.EnsureKind(ColliderShapeDefinition2DKind.Capsule);
        Material = definition.Material;
        Fixed64 radius = definition.Radius;
        Fixed64 height = definition.Size.Y;
        ValidateDimensions(radius, height);
        _radius = radius;
        _height = height;
        MarkShapeDirty();
    }

    public override ColliderType2D Shape => ColliderType2D.Capsule;

    /// <summary>
    /// Gets or sets the unscaled semicircle radius.
    /// </summary>
    public Fixed64 Radius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _radius;
        set
        {
            ValidateDimensions(value, _height);
            if (_radius == value)
                return;

            _radius = value;
            MarkShapeDirty();
        }
    }

    /// <summary>
    /// Gets or sets the unscaled full end-to-end capsule height.
    /// </summary>
    public Fixed64 Height
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _height;
        set
        {
            ValidateDimensions(_radius, value);
            if (_height == value)
                return;

            _height = value;
            MarkShapeDirty();
        }
    }

    /// <summary>
    /// Gets the radius after compound/local scaling.
    /// </summary>
    public Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            GetMassPropertyGeometry(
                out Fixed64 radius,
                out _);
            return radius;
        }
    }

    /// <summary>
    /// Gets the derived normalized world-space direction of the capsule's
    /// conceptual center axis. Exact geometry remains authoritative in the
    /// collider's planar rigid frame.
    /// </summary>
    public Vector2d WorldAxis { get; private set; } = Vector2d.Forward;

    /// <summary>
    /// Gets the full length of the conceptual center axis.
    /// </summary>
    public Fixed64 AxisLength
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            if (HasCommittedShape)
                return _axisLength;

            GetMassPropertyGeometry(
                out _,
                out Fixed64 axisLength);
            return axisLength;
        }
    }

    public override bool ContainsPoint(Vector2d point)
        => FixedSegment2d.ContainsPointInCenteredCapsule(
            point,
            Center,
            Rotation,
            AxisLength,
            ScaledRadius,
            Fixed64.Zero,
            strict: false);

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        Vector2d direction = FixedSegment2d.GetDirectionFromCenteredAxis(
            point,
            Center,
            Rotation,
            AxisLength);
        if (direction == Vector2d.Zero)
            direction = Rotate(Vector2d.Right, Rotation).Normalized;
        if (FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
                point,
                Center,
                Rotation,
                AxisLength,
                ScaledRadius,
                direction,
                out Vector2d surfacePoint))
            return surfacePoint;

        throw new InvalidOperationException("The closest capsule surface point is outside the Fixed64 coordinate domain.");
    }

    internal Vector2d GetNormalFromCenteredAxis(Vector2d point)
    {
        Vector2d direction = FixedSegment2d.GetDirectionFromCenteredAxis(
            point,
            Center,
            Rotation,
            AxisLength);
        return direction != Vector2d.Zero
            ? direction
            : Rotate(Vector2d.Right, Rotation).Normalized;
    }

    internal bool TryGetSurfacePointFromCenteredAxis(
        Vector2d point,
        Vector2d normal,
        out Vector2d surfacePoint) =>
        TryGetSurfacePoint(point, normal, out surfacePoint);

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        if (TryGetSupportPoint(direction, out Vector2d support))
            return support;

        throw new InvalidOperationException("The capsule support point is outside the Fixed64 coordinate domain.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetSupportPoint(Vector2d direction, out Vector2d support)
    {
        Vector2d supportDirection = direction == Vector2d.Zero
            ? Rotate(Vector2d.Right, Rotation)
            : direction;
        return FixedSegment2d.TryGetCenteredCapsuleSupport(
            Center,
            Rotation,
            AxisLength,
            ScaledRadius,
            supportDirection,
            out support);
    }

    private bool TryGetSurfacePoint(
        Vector2d point,
        Vector2d worldNormal,
        out Vector2d surfacePoint) =>
        FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            point,
            Center,
            Rotation,
            AxisLength,
            ScaledRadius,
            worldNormal,
            out surfacePoint);

    internal override Fixed64 CalculateAreaForMassProperties()
    {
        GetMassPropertyGeometry(
            out Fixed64 radius,
            out Fixed64 cylinderLength);
        return cylinderLength * radius * (Fixed64)2 + Fixed64.Pi * radius * radius;
    }

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        GetMassPropertyGeometry(
            out Fixed64 radius,
            out Fixed64 cylinderLength);
        Vector2d centerOfMass = CalculateLocalCenterOfMassOffset();
        Fixed64 momentAboutCenterOfMass = CalculateCenteredMoment(mass, radius, cylinderLength);
        return ApplyParallelAxis(momentAboutCenterOfMass, mass, centerOfMass, localReferencePoint);
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            _radius,
            snapshot.OwnerScale.X,
            snapshot.PartScale.X);
        Fixed64 radiusY = ColliderScalePolicy.ScalePositive(
            _radius,
            snapshot.OwnerScale.Y,
            snapshot.PartScale.Y);
        _preparedRadius = FixedMath.Max(radiusX, radiusY);
        SwiftThrowHelper.ThrowIfArgument(
            !HasValidScaledDimensions(
                _radius,
                _height,
                snapshot.OwnerScale,
                snapshot.PartScale),
            nameof(snapshot),
            "Scaled 2D capsule height must be at least the capsule diameter.");
        SwiftThrowHelper.ThrowIfArgument(
            !Fixed64.TryMultiplySubtractClamped(
                _height,
                snapshot.OwnerScale.Y,
                snapshot.PartScale.Y,
                Fixed64.One,
                _radius,
                Fixed64.Two,
                snapshot.OwnerScale.X,
                snapshot.PartScale.X,
                out Fixed64 axisLengthX)
            | !Fixed64.TryMultiplySubtractClamped(
                _height,
                snapshot.OwnerScale.Y,
                snapshot.PartScale.Y,
                Fixed64.One,
                _radius,
                Fixed64.Two,
                snapshot.OwnerScale.Y,
                snapshot.PartScale.Y,
                out Fixed64 axisLengthY),
            nameof(snapshot),
            "Scaled 2D capsule center-axis length must be representable.");
        _preparedAxisLength = FixedMath.Min(axisLengthX, axisLengthY);
        _preparedAxis = Rotate(Vector2d.Forward, snapshot.Rotation).Normalized;
        SetPreparedBounds(FixedBoundArea.FromCenteredRotatedCapsuleClippedToDomain(
            snapshot.Center,
            snapshot.Rotation,
            _preparedAxisLength,
            _preparedRadius));
    }

    private protected override void PublishShape()
    {
        _scaledRadius = _preparedRadius;
        _axisLength = _preparedAxisLength;
        WorldAxis = _preparedAxis;
    }

    protected override void RecordShapeData(IChronicler chronicler)
    {
        Fixed64 radius = _radius;
        Fixed64 height = _height;
        RecordValues.Look(chronicler, ref radius, "Radius", Fixed64.Half);
        RecordValues.Look(chronicler, ref height, "Height", Fixed64.One);
        if (chronicler.Mode == SerializationMode.Loading)
        {
            ValidateDimensions(radius, height);
            _radius = radius;
            _height = height;
        }
    }

    private void GetMassPropertyGeometry(
        out Fixed64 radius,
        out Fixed64 axisLength)
    {
        if (HasCommittedShape)
        {
            radius = _scaledRadius;
            axisLength = _axisLength;
            return;
        }

        GetCurrentScaleFactors(
            out Vector2d ownerScale,
            out Vector2d partScale);
        Fixed64 radiusX = ColliderScalePolicy.Scale(
            _radius,
            ownerScale.X,
            partScale.X);
        Fixed64 radiusY = ColliderScalePolicy.Scale(
            _radius,
            ownerScale.Y,
            partScale.Y);
        radius = FixedMath.Max(radiusX, radiusY);
        SwiftThrowHelper.ThrowIfTrue(
            !HasValidScaledDimensions(
                _radius,
                _height,
                ownerScale,
                partScale),
            nameof(CalculateAreaForMassProperties),
            "Scaled 2D capsule height must be at least the capsule diameter.");
        bool representable = Fixed64.TryMultiplySubtractClamped(
                _height,
                ownerScale.Y,
                partScale.Y,
                Fixed64.One,
                _radius,
                Fixed64.Two,
                ownerScale.X,
                partScale.X,
                out Fixed64 axisLengthX)
            & Fixed64.TryMultiplySubtractClamped(
                _height,
                ownerScale.Y,
                partScale.Y,
                Fixed64.One,
                _radius,
                Fixed64.Two,
                ownerScale.Y,
                partScale.Y,
                out Fixed64 axisLengthY);
        SwiftThrowHelper.ThrowIfTrue(
            !representable,
            nameof(CalculateAreaForMassProperties),
            "Scaled 2D capsule center-axis length must be representable.");
        axisLength = FixedMath.Min(axisLengthX, axisLengthY);
    }

    private static Fixed64 CalculateCenteredMoment(Fixed64 mass, Fixed64 radius, Fixed64 cylinderLength)
    {
        if (cylinderLength <= Fixed64.Epsilon)
            return mass * radius * radius * Fixed64.Half;

        Fixed64 rectangleArea = cylinderLength * radius * (Fixed64)2;
        Fixed64 circleArea = Fixed64.Pi * radius * radius;
        Fixed64 totalArea = rectangleArea + circleArea;
        if (totalArea <= Fixed64.Zero)
            return mass * cylinderLength * cylinderLength / (Fixed64)12;

        Fixed64 rectangleMass = mass * (rectangleArea / totalArea);
        Fixed64 capMass = mass * ((circleArea * Fixed64.Half) / totalArea);
        Fixed64 rectangleMoment =
            rectangleMass * ((radius * (Fixed64)2 * radius * (Fixed64)2) + (cylinderLength * cylinderLength)) / (Fixed64)12;

        // Semicircle centroid distance from its flat edge is 4r / (3pi).
        Fixed64 capCentroidOffset = (Fixed64)4 * radius / ((Fixed64)3 * Fixed64.Pi);
        Fixed64 capCenterDistance = cylinderLength * Fixed64.Half + capCentroidOffset;
        Fixed64 capCenteredFactor =
            Fixed64.One / (Fixed64)4
            - (Fixed64)16 / ((Fixed64)9 * Fixed64.Pi * Fixed64.Pi);
        Fixed64 capMomentAboutOwnCentroid = capMass * radius * radius * capCenteredFactor;
        Fixed64 capMomentAboutCapsuleCenter =
            capMomentAboutOwnCentroid + capMass * capCenterDistance * capCenterDistance;

        return rectangleMoment + capMomentAboutCapsuleCenter * (Fixed64)2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateDimensions(Fixed64 radius, Fixed64 height)
    {
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "2D capsule radius must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(
            Fixed64.CompareProducts(
                height,
                Fixed64.One,
                radius,
                Fixed64.Two) < 0,
            nameof(height),
            "2D capsule height must be at least the capsule diameter.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasValidScaledDimensions(
        Fixed64 radius,
        Fixed64 height,
        Vector2d ownerScale,
        Vector2d partScale) =>
        Fixed64.CompareProducts(
            height,
            ownerScale.Y,
            partScale.Y,
            Fixed64.One,
            radius,
            Fixed64.Two,
            ownerScale.X,
            partScale.X) >= 0
        & Fixed64.CompareProducts(
            height,
            ownerScale.Y,
            partScale.Y,
            Fixed64.One,
            radius,
            Fixed64.Two,
            ownerScale.Y,
            partScale.Y) >= 0;
}
