//=======================================================================
// LSCapsuleCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Bounds;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D capsule collider whose local center segment follows local 2D Y.
/// </summary>
public sealed class LSCapsuleCollider2D : LSCollider2D
{
    private Fixed64 _radius;
    private Fixed64 _height;

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
        get => _radius * FixedMath.Max(LocalScale.X, LocalScale.Y);
    }

    /// <summary>
    /// Gets the full end-to-end capsule height after local-axis scaling.
    /// </summary>
    public Fixed64 ScaledHeight
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            Fixed64 radius = ScaledRadius;
            Fixed64 scaledHeight = _height * LocalScale.Y;
            Fixed64 diameter = radius * (Fixed64)2;
            return scaledHeight < diameter ? diameter : scaledHeight;
        }
    }

    /// <summary>
    /// Gets the world-space center of the lower local-Y cap.
    /// </summary>
    public Vector2d SegmentStart
    {
        get
        {
            CalculateSegment(out Vector2d start, out _);
            return start;
        }
    }

    /// <summary>
    /// Gets the world-space center of the upper local-Y cap.
    /// </summary>
    public Vector2d SegmentEnd
    {
        get
        {
            CalculateSegment(out _, out Vector2d end);
            return end;
        }
    }

    public override bool ContainsPoint(Vector2d point)
    {
        CalculateSegment(out Vector2d start, out Vector2d end);
        Fixed64 radius = ScaledRadius;
        Vector2d closest = new FixedSegment2d(start, end).ClosestPoint(point);
        return Vector2d.DistanceSquared(point, closest) <= radius * radius;
    }

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        CalculateSegment(out Vector2d start, out Vector2d end);
        Vector2d segmentPoint = new FixedSegment2d(start, end).ClosestPoint(point);
        Vector2d direction = point - segmentPoint;
        Fixed64 directionLengthSquared = direction.MagnitudeSquared;
        if (directionLengthSquared <= Fixed64.Epsilon)
            direction = Rotate(Vector2d.Right, Rotation);
        else
            direction /= FixedMath.Sqrt(directionLengthSquared);

        return segmentPoint + direction * ScaledRadius;
    }

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        CalculateSegment(out Vector2d start, out Vector2d end);
        Vector2d normal = direction.MagnitudeSquared > Fixed64.Epsilon
            ? direction.Normalized
            : Rotate(Vector2d.Right, Rotation);
        Vector2d axis = end - start;
        Vector2d segmentPoint = Vector2d.Dot(axis, normal) >= Fixed64.Zero ? end : start;
        return segmentPoint + normal * ScaledRadius;
    }

    internal override Fixed64 CalculateAreaForMassProperties()
    {
        Fixed64 radius = ScaledRadius;
        Fixed64 cylinderLength = GetScaledCylinderLength(radius);
        return cylinderLength * radius * (Fixed64)2 + Fixed64.Pi * radius * radius;
    }

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 radius = ScaledRadius;
        Fixed64 cylinderLength = GetScaledCylinderLength(radius);
        Vector2d centerOfMass = CalculateLocalCenterOfMassOffset();
        Fixed64 momentAboutCenterOfMass = CalculateCenteredMoment(mass, radius, cylinderLength);
        return ApplyParallelAxis(momentAboutCenterOfMass, mass, centerOfMass, localReferencePoint);
    }

    protected override void RebuildShape()
    {
        CalculateSegment(out Vector2d start, out Vector2d end);
        Fixed64 radius = ScaledRadius;
        Vector2d min = new(
            FixedMath.Min(start.X, end.X) - radius,
            FixedMath.Min(start.Y, end.Y) - radius);
        Vector2d max = new(
            FixedMath.Max(start.X, end.X) + radius,
            FixedMath.Max(start.Y, end.Y) + radius);
        SetBoundsFromMinMax(min, max);
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

    private void CalculateSegment(out Vector2d start, out Vector2d end)
    {
        Fixed64 halfSegmentLength = GetScaledCylinderLength(ScaledRadius) * Fixed64.Half;
        Vector2d localAxis = new(Fixed64.Zero, halfSegmentLength);
        Vector2d worldAxis = Rotate(localAxis, Rotation);
        Vector2d center = Center;
        start = center - worldAxis;
        end = center + worldAxis;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 GetScaledCylinderLength(Fixed64 scaledRadius)
    {
        Fixed64 cylinderLength = ScaledHeight - scaledRadius * (Fixed64)2;
        return cylinderLength > Fixed64.Zero ? cylinderLength : Fixed64.Zero;
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
            height < radius * (Fixed64)2,
            nameof(height),
            "2D capsule height must be at least the capsule diameter.");
    }
}
