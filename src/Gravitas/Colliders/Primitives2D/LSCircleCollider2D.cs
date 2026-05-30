using FixedMathSharp;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D circle collider.
/// </summary>
public sealed class LSCircleCollider2D : LSCollider2D
{
    private Fixed64 _radius;

    public LSCircleCollider2D(Fixed64 radius)
    {
        Radius = radius;
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

    internal override int VertexCount => 0;

    public override bool ContainsPoint(Vector2d point) =>
        Vector2d.SqrDistance(point, Center) <= Radius * Radius;

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        Vector2d direction = point - Center;
        if (direction.SqrMagnitude <= Fixed64.Epsilon)
            return Center + Vector2d.Right * Radius;

        return Center + direction.Normal * Radius;
    }

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        if (direction.SqrMagnitude <= Fixed64.Epsilon)
            return Center + Vector2d.Right * Radius;

        return Center + direction.Normal * Radius;
    }

    internal override Vector2d GetVertexUnchecked(int index) => Center;

    protected override void RebuildShape()
    {
        Vector2d extents = new(Radius, Radius);
        SetBoundsFromMinMax(Center - extents, Center + extents);
    }
}
