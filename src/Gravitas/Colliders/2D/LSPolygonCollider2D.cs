//=======================================================================
// LSPolygonCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Bounds;
using System;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D convex polygon collider with deterministic vertex ordering.
/// </summary>
public sealed class LSPolygonCollider2D : LSCollider2D, IConvexVertexSource2D
{
    private Vector2d[] _localVertices;
    private Vector2d[] _worldVertices;

    public LSPolygonCollider2D(params Vector2d[] vertices)
    {
        _localVertices = Array.Empty<Vector2d>();
        _worldVertices = Array.Empty<Vector2d>();
        SetLocalVertices(vertices, markDirty: true);
    }

    public LSPolygonCollider2D(ColliderShapeDefinition2D definition)
    {
        definition.EnsureKind(ColliderShapeDefinition2DKind.ConvexPolygon);
        Material = definition.Material;
        _localVertices = Array.Empty<Vector2d>();
        _worldVertices = Array.Empty<Vector2d>();
        SetLocalVertices(definition.GetPolygonVerticesForRuntime(), markDirty: true);
    }

    public override ColliderType2D Shape => ColliderType2D.ConvexPolygon;

    public int Count => _worldVertices.Length;

    int IConvexVertexSource2D.VertexCount => _worldVertices.Length;

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
            int orientation = Vector2d.OrientationSign(a, b, point);
            if (orientation > 0)
                hasPositive = true;
            else if (orientation < 0)
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

        Vector2d bestPoint = _worldVertices[0];
        for (int i = 0; i < _worldVertices.Length; i++)
        {
            Vector2d a = _worldVertices[i];
            Vector2d b = _worldVertices[(i + 1) % _worldVertices.Length];
            Vector2d candidate = new FixedSegment2d(a, b).ClosestPoint(point);
            if (Vector2d.CompareDistanceSquared(point, candidate, point, bestPoint) >= 0)
                continue;

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

    Vector2d IConvexVertexSource2D.GetVertexUnchecked(int index) => _worldVertices[index];

    public override Vector2d CalculateLocalCenterOfMassOffset()
    {
        if (!TryCalculateSignedAreaAndCentroid(out _, out Vector2d centroid))
            return base.CalculateLocalCenterOfMassOffset();

        return centroid;
    }

    internal override Fixed64 CalculateAreaForMassProperties()
    {
        Fixed64 signedDoubleArea = CalculateSignedDoubleArea();
        return signedDoubleArea.Abs() * Fixed64.Half;
    }

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        if (!TryCalculateSignedAreaAndCentroid(out Fixed64 signedDoubleArea, out Vector2d centerOfMass))
            return ApplyParallelAxis(
                Fixed64.Zero,
                mass,
                base.CalculateLocalCenterOfMassOffset(),
                localReferencePoint);

        Fixed64 area = signedDoubleArea.Abs() * Fixed64.Half;
        Fixed64 density = mass / area;
        Fixed64 centeredIntegral = Fixed64.Zero;
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d a = GetMassPropertyVertex(i) - centerOfMass;
            Vector2d b = GetMassPropertyVertex((i + 1) % _localVertices.Length) - centerOfMass;
            Fixed64 cross = Vector2d.CrossProduct(a, b);
            Fixed64 term =
                a.MagnitudeSquared +
                Vector2d.Dot(a, b) +
                b.MagnitudeSquared;
            centeredIntegral += cross * term;
        }

        Fixed64 momentAboutCenterOfMass = (density * centeredIntegral).Abs() / (Fixed64)12;

        return ApplyParallelAxis(
            momentAboutCenterOfMass,
            mass,
            centerOfMass,
            localReferencePoint);
    }

    protected override void RebuildShape()
    {
        Vector2d min = Vector2d.Zero;
        Vector2d max = Vector2d.Zero;
        Fixed64 rotation = Rotation;
        Vector2d center = Center;
        Vector2d localScale = LocalScale;
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d vertex = center + Rotate(Vector2d.Multiply(_localVertices[i], localScale), rotation);
            _worldVertices[i] = vertex;
            if (i == 0)
            {
                min = vertex;
                max = vertex;
                continue;
            }

            min = new Vector2d(FixedMath.Min(min.X, vertex.X), FixedMath.Min(min.Y, vertex.Y));
            max = new Vector2d(FixedMath.Max(max.X, vertex.X), FixedMath.Max(max.Y, vertex.Y));
        }

        SetBoundsFromMinMax(min, max);
    }

    protected override void RecordShapeData(IChronicler chronicler)
    {
        Vector2d[] vertices = _localVertices;
        RecordValues.Look(chronicler, ref vertices, "Vertices", Array.Empty<Vector2d>());
        if (chronicler.Mode == SerializationMode.Loading && vertices.Length > 0)
            SetLocalVertices(vertices, markDirty: false);
    }

    private void SetLocalVertices(Vector2d[] vertices, bool markDirty)
    {
        SwiftThrowHelper.ThrowIfNull(vertices, nameof(vertices));
        SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon must contain at least three vertices.");
        ValidateConvexPolygon(vertices);

        if (_localVertices.Length != vertices.Length)
        {
            _localVertices = new Vector2d[vertices.Length];
            _worldVertices = new Vector2d[vertices.Length];
        }

        Array.Copy(vertices, _localVertices, vertices.Length);
        if (markDirty)
            MarkShapeDirty();
    }

    internal static void ValidateConvexPolygon(Vector2d[] vertices)
    {
        SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon must contain at least three vertices.");

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

    private Fixed64 CalculateSignedDoubleArea()
    {
        Fixed64 signedDoubleArea = Fixed64.Zero;
        Vector2d anchor = GetMassPropertyVertex(0);
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d a = GetMassPropertyVertex(i) - anchor;
            Vector2d b = GetMassPropertyVertex((i + 1) % _localVertices.Length) - anchor;
            signedDoubleArea += Vector2d.CrossProduct(a, b);
        }

        return signedDoubleArea;
    }

    private bool TryCalculateSignedAreaAndCentroid(out Fixed64 signedDoubleArea, out Vector2d centroid)
    {
        signedDoubleArea = Fixed64.Zero;
        Vector2d weightedCentroid = Vector2d.Zero;
        Vector2d anchor = GetMassPropertyVertex(0);
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d a = GetMassPropertyVertex(i) - anchor;
            Vector2d b = GetMassPropertyVertex((i + 1) % _localVertices.Length) - anchor;
            Fixed64 cross = Vector2d.CrossProduct(a, b);
            signedDoubleArea += cross;
            weightedCentroid += (a + b) * cross;
        }

        if (signedDoubleArea.Abs() <= Fixed64.Epsilon)
        {
            centroid = Vector2d.Zero;
            return false;
        }

        centroid = anchor + weightedCentroid / ((Fixed64)3 * signedDoubleArea);
        return true;
    }

    private Vector2d GetMassPropertyVertex(int index) =>
        TransformMassPropertyPoint(
            ScaledLocalOffset + Vector2d.Multiply(_localVertices[index], LocalScale));
}
