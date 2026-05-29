using FixedMathSharp;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D convex polygon collider with deterministic vertex ordering.
/// </summary>
public sealed class LSPolygonCollider2D : LSCollider2D
{
    private readonly Vector2d[] _localVertices;
    private readonly Vector2d[] _worldVertices;

    public LSPolygonCollider2D(params Vector2d[] vertices)
    {
        SwiftThrowHelper.ThrowIfNull(vertices, nameof(vertices));
        SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon must contain at least three vertices.");
        ValidateConvex(vertices);

        _localVertices = new Vector2d[vertices.Length];
        _worldVertices = new Vector2d[vertices.Length];
        Array.Copy(vertices, _localVertices, vertices.Length);
        Rebuild();
    }

    public override Collider2DType Shape => Collider2DType.ConvexPolygon;

    public int Count => _worldVertices.Length;

    internal override int VertexCount => _worldVertices.Length;

    public Vector2d GetWorldVertex(int index)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _worldVertices.Length, nameof(index));
        return _worldVertices[index];
    }

    public override bool ContainsPoint(Vector2d point)
    {
        bool hasPositive = false;
        bool hasNegative = false;
        for (int i = 0; i < _worldVertices.Length; i++)
        {
            Vector2d a = _worldVertices[i];
            Vector2d b = _worldVertices[(i + 1) % _worldVertices.Length];
            Fixed64 cross = Vector2d.CrossProduct(b - a, point - a);
            if (cross > Fixed64.Epsilon)
                hasPositive = true;
            else if (cross < -Fixed64.Epsilon)
                hasNegative = true;

            if (hasPositive && hasNegative)
                return false;
        }

        return true;
    }

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        if (ContainsPoint(point))
            return point;

        Fixed64 bestDistance = Fixed64.MAX_VALUE;
        Vector2d bestPoint = _worldVertices[0];
        for (int i = 0; i < _worldVertices.Length; i++)
        {
            Vector2d a = _worldVertices[i];
            Vector2d b = _worldVertices[(i + 1) % _worldVertices.Length];
            Vector2d candidate = ClosestPointOnSegment(point, a, b);
            Fixed64 distance = Vector2d.SqrDistance(point, candidate);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bestPoint = candidate;
        }

        return bestPoint;
    }

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        int bestIndex = 0;
        Fixed64 best = Vector2d.Dot(_worldVertices[0], direction);
        for (int i = 1; i < _worldVertices.Length; i++)
        {
            Fixed64 projection = Vector2d.Dot(_worldVertices[i], direction);
            if (projection <= best)
                continue;

            best = projection;
            bestIndex = i;
        }

        return _worldVertices[bestIndex];
    }

    internal override Vector2d GetVertexUnchecked(int index) => _worldVertices[index];

    protected override void RebuildShape()
    {
        Vector2d min = Vector2d.Zero;
        Vector2d max = Vector2d.Zero;
        Fixed64 rotation = Rotation;
        Vector2d center = Center;
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d vertex = center + Rotate(_localVertices[i], rotation);
            _worldVertices[i] = vertex;
            if (i == 0)
            {
                min = vertex;
                max = vertex;
                continue;
            }

            min = new Vector2d(FixedMath.Min(min.x, vertex.x), FixedMath.Min(min.y, vertex.y));
            max = new Vector2d(FixedMath.Max(max.x, vertex.x), FixedMath.Max(max.y, vertex.y));
        }

        SetBoundsFromMinMax(min, max);
    }

    private static void ValidateConvex(Vector2d[] vertices)
    {
        int sign = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d a = vertices[i];
            Vector2d b = vertices[(i + 1) % vertices.Length];
            Vector2d c = vertices[(i + 2) % vertices.Length];
            Fixed64 cross = Vector2d.CrossProduct(b - a, c - b);
            SwiftThrowHelper.ThrowIfArgument(cross.Abs() <= Fixed64.Epsilon, nameof(vertices), "2D polygon vertices must not be collinear.");

            int currentSign = cross > Fixed64.Zero ? 1 : -1;
            if (sign == 0)
            {
                sign = currentSign;
                continue;
            }

            SwiftThrowHelper.ThrowIfArgument(currentSign != sign, nameof(vertices), "2D polygon must be convex.");
        }
    }
}
