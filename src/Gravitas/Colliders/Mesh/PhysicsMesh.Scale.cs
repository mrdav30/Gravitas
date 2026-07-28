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
using static Gravitas.Colliders.MeshCheckedMath;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    private Vector3d _position;
    private Vector3d _ownerScale = Vector3d.One;
    private Vector3d _partScale = Vector3d.One;
    private bool _scaleInitialized;
    private bool _validatedRotationValid;
    private FixedQuaternion _validatedRotation;
    private Fixed64[] _scaledFaceAreas;
    private Fixed64[] _preparedScaledFaceAreas;
    private Vector3d[] _scaledFaceNormals;
    private Vector3d[] _preparedScaledFaceNormals;
    private Fixed64 _scaledTotalArea;
    private FixedMassWeight _scaledTotalAreaWeight;
    private Fixed64 _scaledLocalRadius;
    private FixedBoundBox _scaledLocalBounds;
    private MeshSurfaceMassProperties _surfaceMassProperties;
    private bool _surfaceMassPropertiesValid;

    private Vector3d _preparedPosition;
    private Vector3d _preparedOwnerScale;
    private Vector3d _preparedPartScale;
    private FixedQuaternion _preparedRotation;
    private FixedBoundBox _preparedBounds;
    private FixedBoundBox _preparedScaledLocalBounds;
    private Fixed64 _preparedScaledTotalArea;
    private FixedMassWeight _preparedScaledTotalAreaWeight;
    private Fixed64 _preparedScaledLocalRadius;
    private MeshSurfaceMassProperties _preparedSurfaceMassProperties;
    private bool _preparedSurfaceMassPropertiesValid;
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

    internal FixedMassWeight SurfaceMassWeight =>
        _scaledTotalAreaWeight;

    internal FixedMassWeight PreparedSurfaceMassWeight =>
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

        Vector3d sourceCenter = _localBounds.Center;
        Fixed64 radius = Fixed64.Zero;
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
                    out Fixed64 z)
                || !new Vector3d(x, y, z).TryGetMagnitudeCeiling(
                    out Fixed64 vertexRadius))
            {
                return Fixed64.MaxValue;
            }

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
    public MeshSurfaceMassProperties SurfaceMassProperties
    {
        get
        {
            if (!_surfaceMassPropertiesValid)
                throw new InvalidOperationException("Scaled mesh surface mass properties are not representable.");

            return _surfaceMassProperties;
        }
    }

    internal void PrepareTransformation(
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d ownerScale,
        Vector3d partScale,
        MeshInertiaPolicy? inertiaPolicy)
    {
        ColliderScalePolicy.Validate(ownerScale);
        ColliderScalePolicy.Validate(partScale);
        ValidateRotation(rotation);

        bool geometryChanged = !_scaleInitialized
            || ownerScale != _ownerScale
            || partScale != _partScale;
        _preparedGeometryChanged = geometryChanged;
        if (geometryChanged)
            PrepareScaledGeometry(ownerScale, partScale);
        else
            CopyCommittedGeometryCandidate();

        if (inertiaPolicy == MeshInertiaPolicy.SurfaceApproximation
            && !_preparedSurfaceMassPropertiesValid)
        {
            throw new ArgumentException(
                "Mesh scale must preserve representable surface mass properties.",
                nameof(ownerScale));
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
        _preparedBounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                position,
                rotation,
                _preparedScaledLocalBounds.Min,
                _preparedScaledLocalBounds.Max,
                Vector3d.Zero,
                FixedQuaternion.Identity);
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
                (_supportVertexIndices, _preparedSupportVertexIndices) =
                    (_preparedSupportVertexIndices, _supportVertexIndices);
                (_supportTreeNodes, _preparedSupportTreeNodes) =
                    (_preparedSupportTreeNodes, _supportTreeNodes);
                _supportTreeNodeCount = _preparedSupportTreeNodeCount;
            }

            _scaledLocalBounds = _preparedScaledLocalBounds;
            _scaledTotalArea = _preparedScaledTotalArea;
            _scaledTotalAreaWeight =
                _preparedScaledTotalAreaWeight;
            _scaledLocalRadius = _preparedScaledLocalRadius;
            _surfaceMassProperties = _preparedSurfaceMassProperties;
            _surfaceMassPropertiesValid = _preparedSurfaceMassPropertiesValid;
            _triangleBvhBuildCount++;
            PublishPreparedClosedVolumeMassProperties();
        }

        _position = _preparedPosition;
        _rotation = _preparedRotation;
        _ownerScale = _preparedOwnerScale;
        _partScale = _preparedPartScale;
        _scaleInitialized = true;
        _bounds = _preparedBounds;
    }

    internal void ValidateSurfaceMassProperties(Vector3d scale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            scale != _ownerScale || _partScale != Vector3d.One,
            nameof(scale),
            "Scale must match the committed standalone mesh scale.");
        _ = SurfaceMassProperties;
    }

    internal void ValidateClosedVolumeScaleRepresentability(Vector3d scale)
    {
        SwiftThrowHelper.ThrowIfArgument(
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

    internal void ValidateRotation(FixedQuaternion rotation)
    {
        if (_validatedRotationValid & rotation == _validatedRotation)
            return;

        Fixed64 magnitudeSquared = rotation.MagnitudeSquared;
        bool rotationValid = IsRepresentable(rotation.X)
            & IsRepresentable(rotation.Y)
            & IsRepresentable(rotation.Z)
            & IsRepresentable(rotation.W)
            & magnitudeSquared > Fixed64.Epsilon
            & magnitudeSquared != Fixed64.MaxValue
            & rotation.IsNormalized();
        if (!rotationValid)
            throw new ArgumentException("Mesh rotation must be a normalized representable quaternion.", nameof(rotation));

        _validatedRotation = rotation;
        _validatedRotationValid = true;
    }

    private void PrepareScaledGeometry(Vector3d ownerScale, Vector3d partScale)
    {
        Vector3d sourceCenter = _localBounds.Center;
        _preparedScaledLocalRadius = Fixed64.Zero;
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
            if (!scaledVertex.TryGetMagnitudeCeiling(
                    out Fixed64 vertexRadius))
            {
                _preparedScaledLocalRadius = Fixed64.MaxValue;
            }
            else if (vertexRadius > _preparedScaledLocalRadius)
            {
                _preparedScaledLocalRadius = vertexRadius;
            }
        }

        _preparedScaledLocalBounds = CalculateBounds(_preparedScaledLocalVertices);
        _preparedScaledTotalArea = Fixed64.Zero;
        _preparedScaledTotalAreaWeight = FixedMassWeight.Zero;
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            Vector3d first = _preparedScaledLocalVertices[_triangles[triangleIndex]];
            Vector3d second = _preparedScaledLocalVertices[_triangles[triangleIndex + 1]];
            Vector3d third = _preparedScaledLocalVertices[_triangles[triangleIndex + 2]];
            var triangle = new FixedTriangle(first, second, third);
            Vector3d normal = triangle.Normal;
            Fixed64 area = triangle.Area;
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

        _preparedSurfaceMassPropertiesValid = MeshSurfaceMassProperties.TryCreate(
            _preparedScaledLocalVertices,
            _triangles,
            _preparedScaledTotalArea,
            out _preparedScaledTotalAreaWeight,
            out _preparedSurfaceMassProperties);

        BuildTriangleBVH(_preparedTriangleBVH, _preparedScaledLocalVertices);
        if (_preparedSupportVertexIndices != null)
        {
            for (int i = 0; i < _preparedSupportVertexIndices.Length; i++)
                _preparedSupportVertexIndices[i] = i;

            _preparedSupportTreeNodeCount = 0;
            BuildSupportTreeNode(
                _preparedScaledLocalVertices,
                _preparedSupportVertexIndices,
                _preparedSupportTreeNodes!,
                ref _preparedSupportTreeNodeCount,
                0,
                _preparedSupportVertexIndices.Length);
        }
    }

    private void CopyCommittedGeometryCandidate()
    {
        _preparedScaledLocalBounds = _scaledLocalBounds;
        _preparedScaledTotalArea = _scaledTotalArea;
        _preparedScaledTotalAreaWeight =
            _scaledTotalAreaWeight;
        _preparedScaledLocalRadius = _scaledLocalRadius;
        _preparedSurfaceMassProperties = _surfaceMassProperties;
        _preparedSurfaceMassPropertiesValid = _surfaceMassPropertiesValid;
    }

}
