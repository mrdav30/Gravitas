//=======================================================================
// LSAABBoxCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D axis-aligned box collider.
/// </summary>
public sealed class LSAABBoxCollider2D : LSCollider2D, IConvexVertexSource2D
{
    private Vector2d _size;
    private Vector2d _halfExtents;
    private Vector2d _scaledHalfExtents;
    private Vector2d _preparedHalfExtents;

    public LSAABBoxCollider2D(Vector2d size)
    {
        Size = size;
    }

    public LSAABBoxCollider2D(ColliderShapeDefinition2D definition)
    {
        definition.EnsureKind(ColliderShapeDefinition2DKind.AABBox);
        Material = definition.Material;
        Size = definition.Size;
    }

    public override ColliderType2D Shape => ColliderType2D.AABox;

    public Vector2d Size
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _size;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value.X <= Fixed64.Zero || value.Y <= Fixed64.Zero,
                nameof(value),
                "2D AABB size components must be greater than zero.");
            if (_size == value)
                return;

            _size = value;
            _halfExtents = value * Fixed64.Half;
            MarkShapeDirty();
        }
    }

    public Vector2d HalfExtents => _halfExtents;

    public Vector2d ScaledHalfExtents
    {
        get
        {
            if (HasCommittedShape)
                return _scaledHalfExtents;

            GetCurrentScaleFactors(
                out Vector2d ownerScale,
                out Vector2d partScale);
            return ColliderScalePolicy.Scale(
                _size,
                ownerScale,
                partScale,
                Fixed64.Two);
        }
    }

    int IConvexVertexSource2D.VertexCount => 4;

    Fixed64 IConvexVertexSource2D.Rotation => Fixed64.Zero;

    public override bool ContainsPoint(Vector2d point)
    {
        Span<Vector2d> offsets = stackalloc Vector2d[4];
        GetCenterRelativeVertices(offsets);
        return FixedConvex2dRelations.ContainsPoint(point, Center, offsets);
    }

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        Span<Vector2d> offsets = stackalloc Vector2d[4];
        GetCenterRelativeVertices(offsets);
        if (FixedConvex2dRelations.ContainsPoint(point, Center, offsets))
            return point;

        Vector2d offset =
            FixedConvex2dRelations.GetClosestPointOffset(
                point,
                Center,
                offsets);
        // An exterior closest point lies on the segment from the representable
        // query to the representable box center, so its final sum is in range.
        _ = Vector2d.TryAdd(Center, offset, out Vector2d closest);
        return closest;
    }

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        Vector2d halfExtents = ScaledHalfExtents;
        Vector2d offset = new(
            direction.X >= Fixed64.Zero
                ? halfExtents.X
                : -halfExtents.X,
            direction.Y >= Fixed64.Zero
                ? halfExtents.Y
                : -halfExtents.Y);
        if (Vector2d.TryAdd(Center, offset, out Vector2d support))
            return support;

        throw new System.InvalidOperationException(
            "The box support point is outside the Fixed64 coordinate domain.");
    }

    Vector2d IConvexVertexSource2D.GetScaledLocalVertexUnchecked(int index)
    {
        Vector2d halfExtents = ScaledHalfExtents;
        return index switch
        {
            0 => new Vector2d(-halfExtents.X, -halfExtents.Y),
            1 => new Vector2d(halfExtents.X, -halfExtents.Y),
            2 => new Vector2d(halfExtents.X, halfExtents.Y),
            _ => new Vector2d(-halfExtents.X, halfExtents.Y)
        };
    }

    FixedPointAnchor2d IConvexVertexSource2D.GetSupportAnchor(Vector2d direction)
    {
        Vector2d halfExtents = ScaledHalfExtents;
        return new FixedPointAnchor2d(
            Center,
            Fixed64.Zero,
            new Vector2d(
                direction.X >= Fixed64.Zero
                    ? halfExtents.X
                    : -halfExtents.X,
                direction.Y >= Fixed64.Zero
                    ? halfExtents.Y
                    : -halfExtents.Y));
    }

    private void GetCenterRelativeVertices(Span<Vector2d> offsets)
    {
        Vector2d halfExtents = ScaledHalfExtents;
        offsets[0] = new Vector2d(-halfExtents.X, -halfExtents.Y);
        offsets[1] = new Vector2d(halfExtents.X, -halfExtents.Y);
        offsets[2] = new Vector2d(halfExtents.X, halfExtents.Y);
        offsets[3] = new Vector2d(-halfExtents.X, halfExtents.Y);
    }

    internal override Fixed64 CalculateAreaForMassProperties()
    {
        Vector2d halfExtents = ScaledHalfExtents;
        return Fixed64.TryMultiplyDivide(
            halfExtents.X,
            halfExtents.Y,
            (Fixed64)4,
            Fixed64.One,
            out Fixed64 area)
            ? area
            : Fixed64.MaxValue;
    }

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        Vector2d halfExtents = ScaledHalfExtents;
        bool representable = Fixed64.TryMultiplyDivide(
                mass,
                halfExtents.X,
                halfExtents.X,
                (Fixed64)3,
                out Fixed64 xMoment)
            & Fixed64.TryMultiplyDivide(
                mass,
                halfExtents.Y,
                halfExtents.Y,
                (Fixed64)3,
                out Fixed64 yMoment)
            & Fixed64.TryAdd(xMoment, yMoment, out Fixed64 momentAboutCenterOfMass);
        if (!representable)
            momentAboutCenterOfMass = Fixed64.MaxValue;

        return ApplyParallelAxis(
            momentAboutCenterOfMass,
            mass,
            CalculateLocalCenterOfMassOffset(),
            localReferencePoint);
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        _preparedHalfExtents = ColliderScalePolicy.ScalePositive(
            _size,
            snapshot.OwnerScale,
            snapshot.PartScale,
            Fixed64.Two);
        SetPreparedBounds(FixedBoundArea.FromCenterAndScopeClippedToDomain(
            snapshot.Center,
            _preparedHalfExtents));
    }

    private protected override void PublishShape() =>
        _scaledHalfExtents = _preparedHalfExtents;

    protected override void RecordShapeData(IChronicler chronicler)
    {
        Vector2d size = _size;
        RecordValues.Look(chronicler, ref size, "Size", Vector2d.One);
        if (chronicler.Mode == SerializationMode.Loading)
        {
            SwiftThrowHelper.ThrowIfArgument(
                size.X <= Fixed64.Zero || size.Y <= Fixed64.Zero,
                nameof(size),
                "2D AABB size components must be greater than zero.");
            _size = size;
            _halfExtents = size * Fixed64.Half;
        }
    }
}
