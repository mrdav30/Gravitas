//=======================================================================
// LSCircleCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D circle collider.
/// </summary>
public sealed class LSCircleCollider2D : LSCollider2D
{
    private Fixed64 _radius;
    private Fixed64 _scaledRadius;
    private Fixed64 _preparedRadius;

    public LSCircleCollider2D(Fixed64 radius)
    {
        Radius = radius;
    }

    public LSCircleCollider2D(ColliderShapeDefinition2D definition)
    {
        definition.EnsureKind(ColliderShapeDefinition2DKind.Circle);
        Material = definition.Material;
        Radius = definition.Radius;
    }

    public override ColliderType2D Shape => ColliderType2D.Circle;

    public Fixed64 Radius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _radius;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(value <= Fixed64.Zero, nameof(value), "2D circle radius must be greater than zero.");
            if (_radius == value)
                return;

            _radius = value;
            MarkShapeDirty();
        }
    }

    public Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetMassPropertyRadius();
    }

    public override bool ContainsPoint(Vector2d point) =>
        FixedSegment2d.ContainsPointInCenteredCapsule(
            point,
            Center,
            Vector2d.Forward,
            Fixed64.Zero,
            ScaledRadius,
            Fixed64.Zero);

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        if (TryGetClosestBoundaryAnchor(
                point,
                out FixedPointAnchor2d anchor)
            && anchor.TryGetPoint(out Vector2d closest))
        {
            return closest;
        }

        throw new System.InvalidOperationException(
            "The closest circle surface point is outside the Fixed64 coordinate domain.");
    }

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        if (direction == Vector2d.Zero)
            direction = Vector2d.Right;

        if (Vector2d.TryRotate(
                direction.Normalized,
                -Rotation,
                out Vector2d localDirection))
        {
            FixedPointAnchor2d anchor =
                FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Center,
                Rotation,
                Vector2d.Forward,
                Fixed64.Zero,
                ScaledRadius,
                localDirection);
            if (anchor.TryGetPoint(out Vector2d support))
                return support;
        }

        throw new System.InvalidOperationException(
            "The circle support point is outside the Fixed64 coordinate domain.");
    }

    internal override ExactMassWeight CalculateAreaForMassProperties()
    {
        Fixed64 radius = GetMassPropertyRadius();
        return ExactMassWeight.FromProduct(
            Fixed64.Pi,
            radius,
            radius);
    }

    internal override ExactMassWeight CalculatePreparedAreaForMassProperties() =>
        ExactMassWeight.FromProduct(
            Fixed64.Pi,
            _preparedRadius,
            _preparedRadius);

    internal override Fixed64 CalculateCenterOfMassMoment(Fixed64 mass)
    {
        Fixed64 radius = GetMassPropertyRadius();
        return Fixed64.TryMultiplyDivide(
            mass,
            radius,
            radius,
            Fixed64.Two,
            out Fixed64 moment)
            ? moment
            : Fixed64.MaxValue;
    }

    private Fixed64 GetMassPropertyRadius()
    {
        if (HasCommittedShape)
            return _scaledRadius;

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
        return FixedMath.Max(radiusX, radiusY);
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
        SetPreparedBounds(FixedBoundArea.FromCenterAndScopeClippedToDomain(
            snapshot.Center,
            new Vector2d(_preparedRadius, _preparedRadius)));
    }

    private protected override void PublishShape() =>
        _scaledRadius = _preparedRadius;

    protected override void RecordShapeData(IChronicler chronicler)
    {
        Fixed64 radius = _radius;
        RecordValues.Look(chronicler, ref radius, "Radius", Fixed64.Half);
        if (chronicler.Mode == SerializationMode.Loading)
        {
            SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "2D circle radius must be greater than zero.");
            _radius = radius;
        }
    }
}
