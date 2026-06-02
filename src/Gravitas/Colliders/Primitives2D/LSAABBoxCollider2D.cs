using FixedMathSharp;
using Chronicler;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D axis-aligned box collider.
/// </summary>
public sealed class LSAABBoxCollider2D : LSCollider2D
{
    private Vector2d _size;
    private Vector2d _halfExtents;

    public LSAABBoxCollider2D(Vector2d size)
    {
        Size = size;
    }

    public override ColliderType2D Shape => ColliderType2D.AABox;

    public Vector2d Size
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _size;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value.x <= Fixed64.Zero || value.y <= Fixed64.Zero,
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

    internal override int VertexCount => 4;

    public override bool ContainsPoint(Vector2d point) =>
        point.x >= MinX && point.x <= MaxX
        && point.y >= MinY && point.y <= MaxY;

    public override Vector2d GetClosestPoint(Vector2d point) =>
        new(
            ClampAxis(point.x, MinX, MaxX),
            ClampAxis(point.y, MinY, MaxY));

    public override Vector2d GetSupportPoint(Vector2d direction) =>
        new(
            direction.x >= Fixed64.Zero ? MaxX : MinX,
            direction.y >= Fixed64.Zero ? MaxY : MinY);

    internal override Vector2d GetVertexUnchecked(int index)
    {
        return index switch
        {
            0 => new Vector2d(MinX, MinY),
            1 => new Vector2d(MaxX, MinY),
            2 => new Vector2d(MaxX, MaxY),
            _ => new Vector2d(MinX, MaxY)
        };
    }

    protected override void RebuildShape()
    {
        SetBoundsFromMinMax(Center - _halfExtents, Center + _halfExtents);
    }

    protected override void RecordShapeData(IChronicler chronicler)
    {
        Vector2d size = _size;
        RecordValues.Look(chronicler, ref size, "Size", Vector2d.One);
        if (chronicler.Mode == SerializationMode.Loading)
        {
            SwiftThrowHelper.ThrowIfArgument(
                size.x <= Fixed64.Zero || size.y <= Fixed64.Zero,
                nameof(size),
                "2D AABB size components must be greater than zero.");
            _size = size;
            _halfExtents = size * Fixed64.Half;
        }
    }
}
