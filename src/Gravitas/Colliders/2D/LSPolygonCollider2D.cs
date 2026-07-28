//=======================================================================
// LSPolygonCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Colliders;

/// <summary>
/// Pure 2D convex polygon collider with deterministic vertex ordering.
/// </summary>
public sealed class LSPolygonCollider2D : LSCollider2D, IConvexVertexSource2D
{
    private Vector2d[] _localVertices;
    private Vector2d[] _scaledLocalVertices;
    private Vector2d[] _scaledLocalVerticesScratch;

    public LSPolygonCollider2D(params Vector2d[] vertices)
    {
        _localVertices = Array.Empty<Vector2d>();
        _scaledLocalVertices = Array.Empty<Vector2d>();
        _scaledLocalVerticesScratch = Array.Empty<Vector2d>();
        SetLocalVertices(vertices, markDirty: true);
    }

    public LSPolygonCollider2D(ColliderShapeDefinition2D definition)
    {
        definition.EnsureKind(ColliderShapeDefinition2DKind.ConvexPolygon);
        Material = definition.Material;
        _localVertices = Array.Empty<Vector2d>();
        _scaledLocalVertices = Array.Empty<Vector2d>();
        _scaledLocalVerticesScratch = Array.Empty<Vector2d>();
        SetLocalVertices(definition.GetPolygonVerticesForRuntime(), markDirty: true);
    }

    public override ColliderType2D Shape => ColliderType2D.ConvexPolygon;

    public int Count => _scaledLocalVertices.Length;

    /// <summary>
    /// Gets the committed vertices in the collider's scaled local frame.
    /// </summary>
    internal ReadOnlySpan<Vector2d> ScaledLocalVertices => _scaledLocalVertices;

    int IConvexVertexSource2D.VertexCount => _scaledLocalVertices.Length;

    Fixed64 IConvexVertexSource2D.Rotation => Rotation;

    /// <summary>
    /// Gets a world-space vertex when the conceptual point is representable.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// Thrown when the conceptual vertex lies outside the fixed-point scalar
    /// domain. Use <see cref="TryGetWorldVertex(int, out Vector2d)"/> when
    /// querying geometry near a scalar boundary.
    /// </exception>
    public Vector2d GetWorldVertex(int index)
    {
        if (TryGetWorldVertex(index, out Vector2d vertex))
            return vertex;

        throw new InvalidOperationException(
            "The polygon vertex is outside the representable coordinate range. Use TryGetWorldVertex.");
    }

    /// <summary>
    /// Attempts to materialize a committed polygon vertex in world space
    /// without saturation.
    /// </summary>
    public bool TryGetWorldVertex(int index, out Vector2d vertex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(
            index,
            _scaledLocalVertices.Length,
            nameof(index));
        return TryGetVertex(index, out vertex);
    }

    public override bool ContainsPoint(Vector2d point) =>
        FixedConvex2dRelations.ContainsPoint(
            point,
            Center,
            Rotation,
            _scaledLocalVertices);

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        if (ContainsPoint(point))
            return point;

        FixedPointAnchor2d anchor =
            FixedConvex2dRelations.GetClosestPointAnchor(
                point,
                Center,
                Rotation,
                _scaledLocalVertices);
        if (anchor.TryGetPoint(out Vector2d closest))
        {
            return closest;
        }

        throw new InvalidOperationException(
            "The closest polygon point is outside the Fixed64 coordinate domain.");
    }

    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        FixedPointAnchor2d anchor = FixedConvex2dRelations.GetSupportAnchor(
            Center,
            Rotation,
            _scaledLocalVertices,
            direction);
        if (anchor.TryGetPoint(out Vector2d support))
        {
            return support;
        }

        throw new InvalidOperationException(
            "The polygon support point is outside the Fixed64 coordinate domain.");
    }

    Vector2d IConvexVertexSource2D.GetScaledLocalVertexUnchecked(int index) =>
        _scaledLocalVertices[index];

    FixedPointAnchor2d IConvexVertexSource2D.GetSupportAnchor(Vector2d direction) =>
        FixedConvex2dRelations.GetSupportAnchor(
            Center,
            Rotation,
            _scaledLocalVertices,
            direction);

    internal override FixedMassPoint2d CalculateLocalMassPoint()
    {
        _ = TryCalculateIntrinsicSignedAreaAndCentroid(
            out _,
            out Vector2d intrinsicCentroid);
        return TransformRelativeMassPropertyPointExact(intrinsicCentroid);
    }

    internal override FixedMassPoint2d CalculatePreparedLocalMassPoint()
    {
        _ = FixedConvex2dRelations.TryGetMassWeightAndCentroid(
            _scaledLocalVerticesScratch,
            out _,
            out Vector2d intrinsicCentroid);
        return TransformPreparedRelativeMassPropertyPointExact(
            intrinsicCentroid);
    }

    internal override FixedMassWeight CalculateAreaForMassProperties()
    {
        ReadOnlySpan<Vector2d> vertices = GetMassPropertyVertices();
        _ = FixedConvex2dRelations.TryGetMassWeightAndCentroid(
            vertices,
            out FixedMassWeight weight,
            out _);
        return weight;
    }

    internal override FixedMassWeight CalculatePreparedAreaForMassProperties()
    {
        _ = FixedConvex2dRelations.TryGetMassWeightAndCentroid(
            _scaledLocalVerticesScratch,
            out FixedMassWeight weight,
            out _);
        return weight;
    }

    internal override Fixed64 CalculateCenterOfMassMoment(Fixed64 mass)
    {
        ReadOnlySpan<Vector2d> vertices = GetMassPropertyVertices();
        _ = FixedConvex2dRelations.TryGetAreaAndCentroid(
            vertices,
            out Fixed64 area,
            out Vector2d intrinsicCenterOfMass);
        if (area <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 density = mass / area;
        Fixed64 centeredIntegral = Fixed64.Zero;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d a = vertices[i] - intrinsicCenterOfMass;
            Vector2d b =
                vertices[(i + 1) % vertices.Length] - intrinsicCenterOfMass;
            Fixed64 cross = Vector2d.CrossProduct(a, b);
            Fixed64 term =
                a.MagnitudeSquared +
                Vector2d.Dot(a, b) +
                b.MagnitudeSquared;
            centeredIntegral += cross * term;
        }

        return (density * centeredIntegral).Abs() / (Fixed64)12;
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d scaledVertex = ColliderScalePolicy.Scale(
                _localVertices[i],
                snapshot.OwnerScale,
                snapshot.PartScale);
            _scaledLocalVerticesScratch[i] = scaledVertex;
        }

        SetPreparedBounds(FixedBoundArea.FromRotatedOffsetsClippedToDomain(
            snapshot.Center,
            snapshot.Rotation,
            _scaledLocalVerticesScratch));
    }

    private protected override void PublishShape()
    {
        Vector2d[] offsets = _scaledLocalVertices;
        _scaledLocalVertices = _scaledLocalVerticesScratch;
        _scaledLocalVerticesScratch = offsets;
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
            _scaledLocalVertices = new Vector2d[vertices.Length];
            _scaledLocalVerticesScratch = new Vector2d[vertices.Length];
        }

        Array.Copy(vertices, _localVertices, vertices.Length);
        if (markDirty)
            MarkShapeDirty();
    }

    internal static void ValidateConvexPolygon(Vector2d[] vertices)
    {
        SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon must contain at least three vertices.");
        SwiftThrowHelper.ThrowIfArgument(
            !FixedConvex2dRelations.IsStrictlyConvex(vertices),
            nameof(vertices),
            "2D polygon vertices must form a strictly convex boundary.");
    }

    private bool TryCalculateIntrinsicSignedAreaAndCentroid(
        out Fixed64 signedDoubleArea,
        out Vector2d centroid)
    {
        return FixedConvex2dRelations.TryGetAreaAndCentroid(
            GetMassPropertyVertices(),
            out signedDoubleArea,
            out centroid);
    }

    private ReadOnlySpan<Vector2d> GetMassPropertyVertices()
    {
        if (HasCommittedShape)
            return _scaledLocalVertices;

        GetCurrentScaleFactors(
            out Vector2d ownerScale,
            out Vector2d partScale);
        for (int i = 0; i < _localVertices.Length; i++)
        {
            _scaledLocalVerticesScratch[i] = ColliderScalePolicy.Scale(
                _localVertices[i],
                ownerScale,
                partScale);
        }

        return _scaledLocalVerticesScratch;
    }
}
