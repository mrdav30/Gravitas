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
    private PolygonMassProperties2D _massProperties;
    private PolygonMassProperties2D _preparedMassProperties;

    /// <summary>Creates a pure 2D convex polygon from authored local vertices.</summary>
    public LSPolygonCollider2D(params Vector2d[] vertices)
    {
        _localVertices = Array.Empty<Vector2d>();
        _scaledLocalVertices = Array.Empty<Vector2d>();
        _scaledLocalVerticesScratch = Array.Empty<Vector2d>();
        SetLocalVertices(vertices, markDirty: true);
    }

    /// <summary>Creates a runtime convex polygon from an authored shape definition.</summary>
    public LSPolygonCollider2D(ColliderShapeDefinition2D definition)
    {
        definition.EnsureKind(ColliderShapeDefinition2DKind.ConvexPolygon);
        Material = definition.Material;
        _localVertices = Array.Empty<Vector2d>();
        _scaledLocalVertices = Array.Empty<Vector2d>();
        _scaledLocalVerticesScratch = Array.Empty<Vector2d>();
        SetLocalVertices(definition.GetPolygonVerticesForRuntime(), markDirty: true);
    }

    /// <inheritdoc/>
    public override ColliderType2D Shape => ColliderType2D.ConvexPolygon;

    /// <summary>Gets the polygon vertex count.</summary>
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
        ChronicleGuard.ThrowIfArrayIndexInvalid(
            index,
            _scaledLocalVertices.Length,
            nameof(index));
        return TryGetVertex(index, out vertex);
    }

    /// <inheritdoc/>
    public override bool ContainsPoint(Vector2d point) =>
        FixedConvex2dRelations.ContainsPoint(
            point,
            Center,
            Rotation,
            _scaledLocalVertices);

    /// <inheritdoc/>
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

    /// <inheritdoc/>
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

    internal override ExactMassPoint2D CalculateLocalMassPoint()
    {
        return TransformRelativeMassPropertyPointExact(GetMassProperties().Centroid);
    }

    internal override ExactMassPoint2D CalculatePreparedLocalMassPoint()
    {
        return TransformPreparedPartMassPropertyPointExact(
            _preparedMassProperties.Centroid);
    }

    internal override ExactMassWeight CalculateAreaForMassProperties()
    {
        return GetMassProperties().Weight;
    }

    internal override ExactMassWeight CalculatePreparedAreaForMassProperties()
    {
        return _preparedMassProperties.Weight;
    }

    internal override Fixed64 CalculateCenterOfMassMoment(Fixed64 mass)
    {
        return GetMassProperties().CalculateMoment(mass);
    }

    /// <inheritdoc/>
    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        PolygonMassProperties2D properties = GetMassProperties();
        // Detached scaling may collapse area before shape admission. Retain the
        // base point-mass limit rather than dividing by a zero polygon weight.
        if (properties.Weight.IsZero)
            return base.CalculateMomentOfInertia(mass, localReferencePoint);

        ExactMassPoint2D point = TransformRelativeMassPropertyPointExact(properties.Centroid);
        ExactMassPoint2D reference = ExactMassPoint2D.FromPoint(localReferencePoint);
        Signed320 dx = WideArithmetic.SubtractSigned320(point.XNumerator, reference.XNumerator);
        Signed320 dy = WideArithmetic.SubtractSigned320(point.YNumerator, reference.YNumerator);
        if (dx.IsZero && dy.IsZero)
            return properties.CalculateMoment(mass);

        if (properties.TryCalculateMoment(mass, CompoundLocalRotation, dx, dy, out Fixed64 moment))
            return moment;

        if (!point.TryGetPoint(out _))
            throw new InvalidOperationException("The requested moment of inertia is outside the Fixed64 scalar domain.");
        return Fixed64.MaxValue;
    }

    internal Fixed64 GetCurrentGeometryReach()
    {
        GetCurrentScaleFactors(out Vector2d ownerScale, out Vector2d partScale);
        return GetCandidateGeometryReach(ownerScale, partScale);
    }

    internal Fixed64 GetCandidateGeometryReach(Vector2d ownerScale, Vector2d partScale)
    {
        Fixed64 reach = Fixed64.Zero;
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d vertex = _localVertices[i];
            // Detached metadata can precede a compensating host scale; do not
            // publish or reject runtime geometry while measuring its enclosure.
            if (!Fixed64.TryMultiplyDivide(vertex.X, ownerScale.X, partScale.X,
                    Fixed64.One, out Fixed64 x)
                || !Fixed64.TryMultiplyDivide(vertex.Y, ownerScale.Y, partScale.Y,
                    Fixed64.One, out Fixed64 y)
                || !new Vector2d(x, y).TryGetMagnitudeCeiling(out Fixed64 vertexReach))
                return Fixed64.MaxValue;
            reach = FixedMath.Max(reach, vertexReach);
        }
        return reach;
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        Fixed64 reach = Fixed64.Zero;
        bool verticesChanged = !HasCommittedShape
            || _scaledLocalVertices.Length != _localVertices.Length;
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector2d scaledVertex = ColliderScalePolicy.Scale(
                _localVertices[i],
                snapshot.OwnerScale,
                snapshot.PartScale);
            verticesChanged = verticesChanged || scaledVertex != _scaledLocalVertices[i];
            _scaledLocalVerticesScratch[i] = scaledVertex;
            reach = FixedMath.Max(reach, ColliderGeometryPolicy.Magnitude(scaledVertex));
        }

        // Positive scaling can collapse a small edge through rounding. Validate
        // before publication, but reuse admission when only the pose changes.
        if (verticesChanged)
        {
            ValidateConvexPolygon(_scaledLocalVerticesScratch);
            _preparedMassProperties = PolygonMassProperties2D.Create(_scaledLocalVerticesScratch);
        }
        else
        {
            // Intrinsic integrals depend on effective geometry, not world pose.
            _preparedMassProperties = _massProperties;
        }

        SetPreparedGeometryReach(reach);
        SetPreparedBounds(FixedBoundArea.FromRotatedOffsetsClippedToDomain(
            snapshot.Center,
            snapshot.Rotation,
            _scaledLocalVerticesScratch));
    }

    private protected override void PublishShape()
    {
        Vector2d[] offsets = _scaledLocalVertices;
        if (offsets.Length != _scaledLocalVerticesScratch.Length)
            offsets = new Vector2d[_scaledLocalVerticesScratch.Length];
        _scaledLocalVertices = _scaledLocalVerticesScratch;
        _scaledLocalVerticesScratch = offsets;
        _massProperties = _preparedMassProperties;
    }

    /// <inheritdoc/>
    protected override void RecordShapeData(IChronicler chronicler)
    {
        Vector2d[] vertices = _localVertices;
        RecordValues.Look(chronicler, ref vertices, "Vertices", Array.Empty<Vector2d>());
        if (chronicler.Mode == SerializationMode.Loading && vertices.Length > 0)
            SetLocalVertices(vertices, markDirty: false);
    }

    private void SetLocalVertices(Vector2d[] vertices, bool markDirty)
    {
        ChronicleGuard.ThrowIfNull(vertices, nameof(vertices));
        ChronicleGuard.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon must contain at least three vertices.");
        ValidateConvexPolygon(vertices);

        if (_localVertices.Length != vertices.Length)
        {
            _localVertices = new Vector2d[vertices.Length];
            // A load may change vertex count. Retain committed query geometry
            // until the replacement has passed scaled admission and publishes.
            if (!HasCommittedShape)
                _scaledLocalVertices = new Vector2d[vertices.Length];
            _scaledLocalVerticesScratch = new Vector2d[vertices.Length];
        }

        Array.Copy(vertices, _localVertices, vertices.Length);
        if (markDirty)
            MarkShapeDirty();
    }

    internal static void ValidateConvexPolygon(Vector2d[] vertices)
    {
        ChronicleGuard.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "2D polygon must contain at least three vertices.");
        ChronicleGuard.ThrowIfArgument(
            !FixedConvex2dRelations.IsStrictlyConvex(vertices),
            nameof(vertices),
            "2D polygon vertices must form a strictly convex boundary.");
    }

    internal PolygonMassProperties2D GetMassProperties() => HasCommittedShape
        ? _massProperties
        : PolygonMassProperties2D.Create(GetMassPropertyVertices());

    private ReadOnlySpan<Vector2d> GetMassPropertyVertices()
    {
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
