//=======================================================================
// PhysicsMesh.Scale.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using System;
using static Gravitas.Colliders.MeshCheckedMath;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    private static readonly Fixed64 ScaleRoundTripTolerance = Fixed64.Epsilon * (Fixed64)8;

    private Vector3d _position;
    private Vector3d _scale = Vector3d.One;
    private bool _scaleInitialized;
    private bool _validatedScaleValid;
    private Vector3d _validatedScale;
    private Fixed3x3 _rotationMatrix = Fixed3x3.Identity;
    private Fixed3x3 _inverseRotationMatrix = Fixed3x3.Identity;
    private bool _validatedRotationValid;
    private FixedQuaternion _validatedRotation;
    private readonly Fixed64[] _scaledFaceAreas;
    private readonly Fixed64[] _surfaceValidationFaceAreas;
    private readonly Vector3d[] _scaledFaceNormals;
    private Fixed64 _scaledTotalArea;
    private FixedBoundBox _scaledLocalBounds;
    private MeshSurfaceMassProperties _surfaceMassProperties;
    private bool _surfaceMassPropertiesValid;
    private Vector3d _validatedSurfaceScale;
    private MeshSurfaceMassProperties _validatedSurfaceMassProperties;
    private bool _validatedSurfaceMassPropertiesValid;

    /// <summary>
    /// Gets the strictly positive local scale applied before mesh rotation.
    /// </summary>
    public Vector3d Scale => _scale;

    /// <summary>
    /// Gets the current scaled bounds in mesh-local coordinates.
    /// </summary>
    public FixedBoundBox ScaledLocalBounds => _scaledLocalBounds;

    /// <summary>
    /// Gets cached uniform thin-shell mass properties for the current scale.
    /// </summary>
    public MeshSurfaceMassProperties SurfaceMassProperties
    {
        get
        {
            if (!_surfaceMassPropertiesValid)
            {
                if (!MeshSurfaceMassProperties.TryCreate(
                    _localVertices,
                    _triangles,
                    _scaledFaceAreas,
                    _scale,
                    _scaledLocalBounds.Center,
                    _scaledTotalArea,
                    out _surfaceMassProperties))
                {
                    throw new InvalidOperationException("Scaled mesh surface mass properties are not representable.");
                }

                _surfaceMassPropertiesValid = true;
            }

            return _surfaceMassProperties;
        }
    }

    internal void ValidateSurfaceMassProperties(Vector3d scale)
    {
        if (_validatedSurfaceMassPropertiesValid & scale == _validatedSurfaceScale)
            return;

        Fixed64 totalArea = Fixed64.Zero;
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            Vector3d first = Vector3d.Multiply(_localVertices[_triangles[triangleIndex]], scale);
            Vector3d second = Vector3d.Multiply(_localVertices[_triangles[triangleIndex + 1]], scale);
            Vector3d third = Vector3d.Multiply(_localVertices[_triangles[triangleIndex + 2]], scale);
            Fixed64 area = Vector3d.Cross(second - first, third - first).Magnitude * Fixed64.Half;
            _surfaceValidationFaceAreas[i] = area;
            totalArea += area;
        }

        Vector3d scaledMin = Vector3d.Multiply(_localBounds.Min, scale);
        Vector3d scaledMax = Vector3d.Multiply(_localBounds.Max, scale);
        Vector3d reference = (scaledMin + scaledMax) * Fixed64.Half;
        if (!MeshSurfaceMassProperties.TryCreate(
            _localVertices,
            _triangles,
            _surfaceValidationFaceAreas,
            scale,
            reference,
            totalArea,
            out MeshSurfaceMassProperties properties))
        {
            throw new ArgumentException("Mesh scale must preserve representable surface mass properties.", nameof(scale));
        }

        _validatedSurfaceScale = scale;
        _validatedSurfaceMassProperties = properties;
        _validatedSurfaceMassPropertiesValid = true;
    }

    internal void ValidateScale(Vector3d scale)
    {
        if (_validatedScaleValid & scale == _validatedScale)
            return;

        ValidateScaleComponent(scale.X, nameof(scale));
        ValidateScaleComponent(scale.Y, nameof(scale));
        ValidateScaleComponent(scale.Z, nameof(scale));

        bool determinantValid = TryMultiply(scale.X, scale.Y, out Fixed64 determinant);
        determinantValid &= TryMultiply(determinant, scale.Z, out determinant);
        if (!determinantValid | determinant <= Fixed64.Zero)
        {
            throw new ArgumentException("Mesh scale determinant must be positive and representable.", nameof(scale));
        }

        ValidateScaleReciprocal(determinant, nameof(scale));

        for (int i = 0; i < _localVertices.Length; i++)
        {
            Vector3d source = _localVertices[i];
            Vector3d scaled = Vector3d.Multiply(source, scale);
            ValidateScaledComponent(source.X, scaled.X, scale.X, nameof(scale));
            ValidateScaledComponent(source.Y, scaled.Y, scale.Y, nameof(scale));
            ValidateScaledComponent(source.Z, scaled.Z, scale.Z, nameof(scale));
        }

        Vector3d scaledMin = Vector3d.Multiply(_localBounds.Min, scale);
        Vector3d scaledMax = Vector3d.Multiply(_localBounds.Max, scale);
        Vector3d boundsSpan = scaledMax - scaledMin;
        Vector3d boundsCenterSum = scaledMax + scaledMin;
        if (!IsRepresentable(boundsSpan) | !IsRepresentable(boundsCenterSum))
            throw new ArgumentException("Mesh scale must keep bounds arithmetic representable.", nameof(scale));

        Fixed64 totalArea = Fixed64.Zero;
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            Vector3d first = Vector3d.Multiply(_localVertices[_triangles[triangleIndex]], scale);
            Vector3d second = Vector3d.Multiply(_localVertices[_triangles[triangleIndex + 1]], scale);
            Vector3d third = Vector3d.Multiply(_localVertices[_triangles[triangleIndex + 2]], scale);
            Vector3d firstEdge = second - first;
            Vector3d secondEdge = third - first;
            if (!TryCrossRepresentable(firstEdge, secondEdge, out Vector3d cross))
                throw new ArgumentException("Mesh scale must keep triangle cross products representable.", nameof(scale));

            Fixed64 magnitudeSquared = cross.MagnitudeSquared;
            Fixed64 doubleArea = cross.Magnitude;
            Fixed64 area = doubleArea * Fixed64.Half;
            if (magnitudeSquared == Fixed64.MaxValue)
            {
                throw new ArgumentException("Mesh scale must keep every scaled triangle representable.", nameof(scale));
            }

            if (area <= Fixed64.Epsilon)
                throw new ArgumentException("Mesh scale must keep every source triangle representably nondegenerate.", nameof(scale));

            totalArea += area;
            if (totalArea == Fixed64.MaxValue)
                throw new ArgumentException("Mesh scale must keep total surface area representable.", nameof(scale));
        }

        _validatedScale = scale;
        _validatedScaleValid = true;
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
        {
            throw new ArgumentException("Mesh rotation must be a normalized representable quaternion.", nameof(rotation));
        }

        _validatedRotation = rotation;
        _validatedRotationValid = true;
    }

    private static void ValidateScaleComponent(Fixed64 component, string parameterName)
    {
        if (component <= Fixed64.Zero)
            throw new ArgumentException("Mesh scale components must be greater than zero.", parameterName);

        ValidateScaleReciprocal(component, parameterName);
    }

    private static void ValidateScaleReciprocal(Fixed64 value, string parameterName)
    {
        Fixed64 reciprocal = Fixed64.One / value;
        Fixed64 roundTrip = value * reciprocal;
        if (reciprocal == Fixed64.MaxValue
            | FixedMath.Abs(roundTrip - Fixed64.One) > ScaleRoundTripTolerance)
        {
            throw new ArgumentException("Mesh scale must have a representable deterministic inverse.", parameterName);
        }
    }

    private static void ValidateScaledComponent(
        Fixed64 source,
        Fixed64 scaled,
        Fixed64 scale,
        string parameterName)
    {
        if (!IsRepresentable(scaled)
            | (source != Fixed64.Zero & scaled == Fixed64.Zero)
            | FixedMath.Abs((scaled / scale) - source) > ScaleRoundTripTolerance)
        {
            throw new ArgumentException("Mesh scale must keep every authored vertex representable.", parameterName);
        }
    }

    private static bool TryCrossRepresentable(Vector3d first, Vector3d second, out Vector3d cross)
    {
        cross = default;
        bool valid = TryMultiply(first.Y, second.Z, out Fixed64 yz);
        valid &= TryMultiply(first.Z, second.Y, out Fixed64 zy);
        valid &= TryMultiply(first.Z, second.X, out Fixed64 zx);
        valid &= TryMultiply(first.X, second.Z, out Fixed64 xz);
        valid &= TryMultiply(first.X, second.Y, out Fixed64 xy);
        valid &= TryMultiply(first.Y, second.X, out Fixed64 yx);
        if (!valid)
        {
            return false;
        }

        valid = TrySubtract(yz, zy, out Fixed64 x);
        valid &= TrySubtract(zx, xz, out Fixed64 y);
        valid &= TrySubtract(xy, yx, out Fixed64 z);
        if (!valid)
        {
            return false;
        }

        cross = new Vector3d(x, y, z);
        return true;
    }

    private void UpdateTransformation(Vector3d position, FixedQuaternion rotation, Vector3d scale)
    {
        bool scaleChanged = !_scaleInitialized | scale != _scale;
        if (scaleChanged)
            ValidateScale(scale);
        if (!_validatedRotationValid | rotation != _rotation)
            ValidateRotation(rotation);

        _position = position;
        if (rotation != _rotation)
        {
            _rotationMatrix = rotation.ToMatrix3x3();
            _inverseRotationMatrix = Fixed3x3.Transpose(_rotationMatrix);
        }
        _rotation = rotation;
        if (scaleChanged)
        {
            _scale = scale;
            _scaleInitialized = true;
            UpdateScaleDerivedGeometry();
        }
        else
        {
            PromoteValidatedClosedVolumeMassProperties();
        }

        TransformationMatrix = BuildTransformationMatrix();
    }

    private void UpdateScaleDerivedGeometry()
    {
        UpdateClosedVolumeMassPropertiesScaleCache();
        Vector3d scaledMin = Vector3d.Multiply(_localBounds.Min, _scale);
        Vector3d scaledMax = Vector3d.Multiply(_localBounds.Max, _scale);
        _scaledLocalBounds = FixedBoundBox.FromMinMax(scaledMin, scaledMax);
        _scaledTotalArea = Fixed64.Zero;
        _surfaceMassPropertiesValid = false;

        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            Vector3d first = Vector3d.Multiply(_localVertices[_triangles[triangleIndex]], _scale);
            Vector3d second = Vector3d.Multiply(_localVertices[_triangles[triangleIndex + 1]], _scale);
            Vector3d third = Vector3d.Multiply(_localVertices[_triangles[triangleIndex + 2]], _scale);
            Vector3d cross = Vector3d.Cross(second - first, third - first);
            Fixed64 doubleArea = cross.Magnitude;
            _scaledFaceAreas[i] = doubleArea * Fixed64.Half;
            _scaledFaceNormals[i] = cross / doubleArea;
            _scaledTotalArea += _scaledFaceAreas[i];
        }

        if (_validatedSurfaceMassPropertiesValid & _validatedSurfaceScale == _scale)
        {
            _surfaceMassProperties = _validatedSurfaceMassProperties;
            _surfaceMassPropertiesValid = true;
        }

    }

    private Fixed4x4 BuildTransformationMatrix()
    {
        Fixed3x3 rotation = _rotationMatrix;
        return new Fixed4x4(
            _scale.X * rotation.M11, _scale.X * rotation.M12, _scale.X * rotation.M13, Fixed64.Zero,
            _scale.Y * rotation.M21, _scale.Y * rotation.M22, _scale.Y * rotation.M23, Fixed64.Zero,
            _scale.Z * rotation.M31, _scale.Z * rotation.M32, _scale.Z * rotation.M33, Fixed64.Zero,
            _position.X, _position.Y, _position.Z, Fixed64.One);
    }

    private Fixed4x4 BuildInverseTransformationMatrix()
    {
        Fixed3x3 inverseRotation = _inverseRotationMatrix;
        Fixed64 inverseX = Fixed64.One / _scale.X;
        Fixed64 inverseY = Fixed64.One / _scale.Y;
        Fixed64 inverseZ = Fixed64.One / _scale.Z;
        Fixed4x4 inverse = new(
            inverseRotation.M11 * inverseX, inverseRotation.M12 * inverseY, inverseRotation.M13 * inverseZ, Fixed64.Zero,
            inverseRotation.M21 * inverseX, inverseRotation.M22 * inverseY, inverseRotation.M23 * inverseZ, Fixed64.Zero,
            inverseRotation.M31 * inverseX, inverseRotation.M32 * inverseY, inverseRotation.M33 * inverseZ, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        inverse.M41 = -(_position.X * inverse.M11 + _position.Y * inverse.M21 + _position.Z * inverse.M31);
        inverse.M42 = -(_position.X * inverse.M12 + _position.Y * inverse.M22 + _position.Z * inverse.M32);
        inverse.M43 = -(_position.X * inverse.M13 + _position.Y * inverse.M23 + _position.Z * inverse.M33);
        return inverse;
    }

    private Vector3d InverseTransformWorldPoint(Vector3d worldPoint)
    {
        Vector3d rotated = Fixed3x3.TransformDirection(
            _inverseRotationMatrix,
            worldPoint - _position);
        return new Vector3d(
            rotated.X / _scale.X,
            rotated.Y / _scale.Y,
            rotated.Z / _scale.Z);
    }

}
