//=======================================================================
// PhysicsMesh.Scale.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using SwiftCollections.Query;
using System;

namespace Gravitas.Colliders;

/// <content>Prepares and atomically publishes scaled geometry and its derived caches.</content>
public partial class PhysicsMesh
{
    private Vector3d _position;
    private Vector3d _ownerScale = Vector3d.One;
    private Vector3d _partScale = Vector3d.One;
    private bool _scaleInitialized;
    private Fixed64[] _scaledFaceAreas;
    private Fixed64[] _preparedScaledFaceAreas;
    private Vector3d[] _scaledFaceNormals;
    private Vector3d[] _preparedScaledFaceNormals;
    private Fixed64 _scaledTotalArea;
    private ExactMassWeight _scaledTotalAreaWeight;
    private Fixed64 _scaledLocalRadius;
    private FixedBoundBox _scaledLocalBounds;
    private MeshSurfaceMassProperties _surfaceMassProperties;

    private Vector3d _preparedPosition;
    private Vector3d _preparedOwnerScale;
    private Vector3d _preparedPartScale;
    private FixedQuaternion _preparedRotation;
    private FixedBoundBox _preparedBounds;
    private FixedBoundBox _preparedScaledLocalBounds;
    private Fixed64 _preparedScaledTotalArea;
    private ExactMassWeight _preparedScaledTotalAreaWeight;
    private Fixed64 _preparedScaledLocalRadius;
    private MeshSurfaceMassProperties _preparedSurfaceMassProperties;
    private bool _preparedGeometryChanged;

    /// <summary>
    /// Gets the committed host or standalone scale factor.
    /// </summary>
    public Vector3d OwnerScale => _ownerScale;

    /// <summary>
    /// Gets the committed authored compound-part scale factor.
    /// Standalone meshes use <see cref="Vector3d.One"/>.
    /// </summary>
    public Vector3d PartScale => _partScale;

    /// <summary>
    /// Gets the current centered, scaled bounds in mesh-local coordinates.
    /// </summary>
    public FixedBoundBox ScaledLocalBounds => _scaledLocalBounds;

    internal Fixed64 ScaledLocalRadius => _scaledLocalRadius;

    internal Fixed64 PreparedScaledLocalRadius => _preparedScaledLocalRadius;

    internal bool HasRuntimeGeometry => _scaleInitialized;

    internal ExactMassWeight SurfaceMassWeight =>
        _scaledTotalAreaWeight;

    internal ExactMassWeight PreparedSurfaceMassWeight =>
        _preparedScaledTotalAreaWeight;

    internal Fixed64 GetScaledLocalRadius(
        Vector3d ownerScale,
        Vector3d partScale)
    {
        if (_scaleInitialized
            && ownerScale == _ownerScale
            && partScale == _partScale)
        {
            return _scaledLocalRadius;
        }

        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            ownerScale, FixedQuaternion.Identity, _localBounds.Center, partScale);
        Fixed64 radius = Fixed64.Zero;
        foreach (Vector3d vertex in _localVertices)
        {
            frame.GetScaledLocalPointNumerators(vertex, out Signed192 x, out Signed192 y, out Signed192 z);
            if (!WideGeometry.TryGetMagnitudeCeiling(x, y, z, new Signed192(0, 1, 0), out Fixed64 vertexRadius))
                return Fixed64.MaxValue;
            radius = FixedMath.Max(radius, vertexRadius);
        }

        return radius;
    }

    internal FixedBoundBox PreparedBounds => _preparedBounds;

    internal MeshSurfaceMassProperties PreparedSurfaceMassProperties =>
        _preparedSurfaceMassProperties;

    /// <summary>
    /// Gets cached uniform thin-shell mass properties for the committed geometry.
    /// </summary>
    public MeshSurfaceMassProperties SurfaceMassProperties => _surfaceMassProperties;

    internal void PrepareTransformation(
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d ownerScale,
        Vector3d partScale,
        MeshInertiaPolicy? inertiaPolicy)
    {
        var frame = new ScaledRigidFrame3d(position, FixedQuaternion.Identity,
            Vector3d.Zero, ownerScale, rotation, _localBounds.Center, partScale);
        PrepareTransformation(position, rotation, ownerScale, partScale, inertiaPolicy, frame);
    }

    internal void PrepareTransformation(
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d ownerScale,
        Vector3d partScale,
        MeshInertiaPolicy? inertiaPolicy,
        in ScaledRigidFrame3d sourceFrame)
    {
        ColliderScalePolicy.Validate(ownerScale);
        ColliderScalePolicy.Validate(partScale);
        _preparedSourceFrame = sourceFrame;
        _preparedPosition = position;
        _preparedRotation = rotation;

        bool geometryChanged = !_scaleInitialized
            || ownerScale != _ownerScale
            || partScale != _partScale
            || (_scaleIsExact && _geometryUsesSource != sourceFrame.HasLocalPlacement);
        _preparedGeometryChanged = geometryChanged;
        for (CompoundMassCache? cache = _compoundMassCache; cache != null; cache = cache.Alternate)
        {
            cache.PreparedValid = !geometryChanged && cache.CurrentValid;
            cache.Prepared = cache.Current;
        }
        if (geometryChanged)
            PrepareScaledGeometry(ownerScale, partScale);
        else
        {
            CopyCommittedGeometryCandidate();
            SelectPreparedGeometryFrame(position, rotation);
        }

        PrepareClosedVolumeMassProperties(
            inertiaPolicy,
            geometryChanged,
            ownerScale,
            partScale);

        _preparedPosition = position;
        _preparedRotation = rotation;
        _preparedOwnerScale = ownerScale;
        _preparedPartScale = partScale;
        _preparedBounds = _preparedGeometryUsesSource
            ? sourceFrame.GetWorldBounds(_localBounds.Min, _localBounds.Max)
            : FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                position, rotation, _preparedScaledLocalBounds.Min, _preparedScaledLocalBounds.Max,
                Vector3d.Zero, FixedQuaternion.Identity);
    }

    internal void PublishPreparedTransformation()
    {
        if (_preparedGeometryChanged)
        {
            (_scaledLocalVertices, _preparedScaledLocalVertices) =
                (_preparedScaledLocalVertices, _scaledLocalVertices);
            (_scaledFaceAreas, _preparedScaledFaceAreas) =
                (_preparedScaledFaceAreas, _scaledFaceAreas);
            (_scaledFaceNormals, _preparedScaledFaceNormals) =
                (_preparedScaledFaceNormals, _scaledFaceNormals);
            (_triangleBVH, _preparedTriangleBVH) =
                (_preparedTriangleBVH, _triangleBVH);
            if (_supportVertexIndices != null)
            {
                (_supportTreeNodes, _preparedSupportTreeNodes) =
                    (_preparedSupportTreeNodes, _supportTreeNodes);
            }

            _scaledLocalBounds = _preparedScaledLocalBounds;
            _scaledTotalArea = _preparedScaledTotalArea;
            _scaledTotalAreaWeight =
                _preparedScaledTotalAreaWeight;
            _scaledLocalRadius = _preparedScaledLocalRadius;
            _surfaceMassProperties = _preparedSurfaceMassProperties;
            for (CompoundMassCache? cache = _compoundMassCache; cache != null; cache = cache.Alternate)
            {
                cache.Current = cache.Prepared;
                cache.CurrentValid = cache.PreparedValid;
                if (cache.Runtime != null) cache.Runtime.Ready = false;
            }
            _triangleBvhBuildCount++;
            PublishPreparedClosedVolumeMassProperties();
        }

        _position = _preparedPosition;
        _rotation = _preparedRotation;
        _ownerScale = _preparedOwnerScale;
        _partScale = _preparedPartScale;
        _sourceFrame = _preparedSourceFrame;
        _geometryFrame = _preparedGeometryFrame;
        _geometryUsesSource = _preparedGeometryUsesSource;
        _scaleIsExact = _preparedScaleIsExact;
        _scaleInitialized = true;
        _bounds = _preparedBounds;
    }

    internal void ValidateSurfaceMassProperties(Vector3d scale)
    {
        ChronicleGuard.ThrowIfArgument(
            scale != _ownerScale || _partScale != Vector3d.One,
            nameof(scale),
            "Scale must match the committed standalone mesh scale.");
        _ = SurfaceMassProperties;
    }

    internal void ValidateClosedVolumeScaleRepresentability(Vector3d scale)
    {
        ChronicleGuard.ThrowIfArgument(
            scale != _ownerScale || _partScale != Vector3d.One,
            nameof(scale),
            "Scale must match the committed standalone mesh scale.");
        if (!TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult result)
            && IsClosedSurface)
        {
            throw new ArgumentException(
                $"Mesh scale must preserve a representable closed volume. Validation result: {result}.",
                nameof(scale));
        }
    }

    private void PrepareScaledGeometry(Vector3d ownerScale, Vector3d partScale)
    {
        Vector3d sourceCenter = _localBounds.Center;
        _preparedScaledLocalRadius = Fixed64.Zero;
        _preparedScaleIsExact = true;
        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector3d vertex = _localVertices[i];
            if (!Fixed64.TryMultiplyDifference(
                    vertex.X,
                    sourceCenter.X,
                    ownerScale.X,
                    partScale.X,
                    out Fixed64 x)
                || !Fixed64.TryMultiplyDifference(
                    vertex.Y,
                    sourceCenter.Y,
                    ownerScale.Y,
                    partScale.Y,
                    out Fixed64 y)
                || !Fixed64.TryMultiplyDifference(
                    vertex.Z,
                    sourceCenter.Z,
                    ownerScale.Z,
                    partScale.Z,
                    out Fixed64 z))
            {
                throw new ArgumentException(
                    "Mesh scale must keep every centered authored vertex representable.",
                    nameof(ownerScale));
            }

            Vector3d scaledVertex = new(x, y, z);
            _preparedScaledLocalVertices[i] = scaledVertex;
            bool vertexIsExact = _preparedSourceFrame.IsScaledLocalPointExact(vertex, scaledVertex);
            _preparedScaleIsExact &= vertexIsExact;
            Fixed64 vertexRadius;
            bool radiusRepresented;
            if (vertexIsExact)
            {
                // Exact scalar vertices have the same norm as their retained
                // affine numerators, so keep the compact integer square root.
                radiusRepresented = scaledVertex.TryGetMagnitudeCeiling(out vertexRadius);
            }
            else
            {
                _preparedSourceFrame.GetScaledLocalPointNumerators(vertex,
                    out Signed192 exactX, out Signed192 exactY, out Signed192 exactZ);
                radiusRepresented = WideGeometry.TryGetMagnitudeCeiling(exactX, exactY, exactZ,
                    new Signed192(0, 1, 0), out vertexRadius);
            }
            if (!radiusRepresented)
            {
                _preparedScaledLocalRadius = Fixed64.MaxValue;
            }
            else if (vertexRadius > _preparedScaledLocalRadius)
            {
                _preparedScaledLocalRadius = vertexRadius;
            }
        }

        ColliderGeometryPolicy.ValidateReach(_preparedScaledLocalRadius, nameof(ownerScale));
        _preparedScaledLocalBounds = CalculateBounds(_preparedScaledLocalVertices);
        _preparedScaledTotalArea = Fixed64.Zero;
        _preparedScaledTotalAreaWeight = ExactMassWeight.Zero;
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            Vector3d first = _preparedScaledLocalVertices[_triangles[triangleIndex]];
            Vector3d second = _preparedScaledLocalVertices[_triangles[triangleIndex + 1]];
            Vector3d third = _preparedScaledLocalVertices[_triangles[triangleIndex + 2]];
            var triangle = new FixedTriangle(first, second, third);
            Vector3d normal;
            Fixed64 area;
            if (_preparedScaleIsExact)
            { normal = triangle.Normal; area = triangle.Area; }
            else
            {
                var sourceTriangle = new FixedTriangle(_localVertices[_triangles[triangleIndex]],
                    _localVertices[_triangles[triangleIndex + 1]], _localVertices[_triangles[triangleIndex + 2]]);
                WideTriangleRelations.GetScaledAreaAndNormal(sourceTriangle, _preparedSourceFrame, out area, out normal);
            }
            if (area <= Fixed64.Epsilon)
            {
                throw new ArgumentException(
                    "Mesh scale must keep every source triangle representably nondegenerate.",
                    nameof(ownerScale));
            }
            if (normal == Vector3d.Zero)
            {
                // FixedTriangle's public degeneracy threshold intentionally uses
                // squared area. Mesh admission has always used surface area, so
                // preserve an admitted micro-triangle's representable direction.
                normal = triangle.UnnormalizedNormal.Normalized;
            }

            _preparedScaledTotalArea += area;
            _preparedScaledFaceAreas[i] = area;
            _preparedScaledFaceNormals[i] = normal;
        }

        _preparedSurfaceMassProperties = MeshSurfaceMassProperties.Create(
            _preparedScaleIsExact ? _preparedScaledLocalVertices : _localVertices,
            _triangles,
            _preparedScaledTotalArea,
            out _preparedScaledTotalAreaWeight,
            out MeshMassIntegrals shellIntegrals, _preparedSourceFrame, exactScale: !_preparedScaleIsExact);
        for (CompoundMassCache? cache = _compoundMassCache; cache != null; cache = cache.Alternate)
        {
            if (cache.Policy != MeshInertiaPolicy.SurfaceApproximation) continue;
            cache.Prepared = shellIntegrals;
            cache.PreparedValid = true;
        }

        SelectPreparedGeometryFrame(_preparedPosition, _preparedRotation);
        Vector3d[] geometryVertices = _preparedGeometryUsesSource
            ? _localVertices : _preparedScaledLocalVertices;
        BuildTriangleBVH(_preparedTriangleBVH, geometryVertices);
        if (_supportVertexIndices != null)
        {
            if (!_scaleInitialized)
            {
                _supportTreeNodeCount = 0;
                BuildSupportTreeNode(
                    geometryVertices,
                    _supportVertexIndices,
                    _preparedSupportTreeNodes!,
                    new SupportVertexIndexComparer(),
                    ref _supportTreeNodeCount,
                    0,
                    _supportVertexIndices.Length);
            }
            else
            {
                RefitSupportTree(
                    geometryVertices,
                    _supportTreeNodes!,
                    _preparedSupportTreeNodes!);
            }
        }
    }

    private void CopyCommittedGeometryCandidate()
    {
        _preparedScaleIsExact = _scaleIsExact;
        _preparedScaledLocalBounds = _scaledLocalBounds;
        _preparedScaledTotalArea = _scaledTotalArea;
        _preparedScaledTotalAreaWeight =
            _scaledTotalAreaWeight;
        _preparedScaledLocalRadius = _scaledLocalRadius;
        _preparedSurfaceMassProperties = _surfaceMassProperties;
    }

}
