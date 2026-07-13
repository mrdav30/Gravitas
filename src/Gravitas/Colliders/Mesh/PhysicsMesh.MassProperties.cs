//=======================================================================
// PhysicsMesh.MassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    private bool _scaledClosedVolumeMassPropertiesEvaluated;
    private Vector3d _scaledClosedVolumeMassPropertiesScale;
    private MeshMassProperties _scaledClosedVolumeMassProperties;
    private MeshVolumeValidationResult _scaledClosedVolumeValidationResult;
    private bool _validatedClosedVolumeMassPropertiesValid;
    private Vector3d _validatedClosedVolumeMassPropertiesScale;
    private MeshMassProperties _validatedClosedVolumeMassProperties;

    internal int ClosedVolumeScaleEvaluationCount { get; private set; }

    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass) =>
        CalculateInertiaTensor(mass, MeshInertiaPolicy.RequireClosedVolume);

    /// <summary>
    /// Calculates mesh inertia for the supplied mass using the requested policy.
    /// This is a geometry/topology API; callers apply body mobility gates before requesting inertia.
    /// </summary>
    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, MeshInertiaPolicy policy)
    {
        if (policy == MeshInertiaPolicy.RequireClosedVolume
            && TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _))
        {
            return CalculateInertiaTensor(mass, policy, properties.CenterOfMass);
        }

        if (policy == MeshInertiaPolicy.SurfaceApproximation)
            return CalculateInertiaTensor(mass, policy, SurfaceMassProperties.CenterOfMass);

        return CalculateInertiaTensor(mass, policy, _scaledLocalBounds.Center);
    }

    /// <summary>
    /// Calculates mesh inertia for the supplied mass about a specific local reference point.
    /// This is a geometry/topology API; callers apply body mobility gates before requesting inertia.
    /// </summary>
    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, MeshInertiaPolicy policy, Vector3d localReferencePoint)
    {
        switch (policy)
        {
            case MeshInertiaPolicy.RequireClosedVolume:
                if (!TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out MeshVolumeValidationResult result))
                    throw new InvalidOperationException($"Mesh inertia requires a validated closed volume. Validation result: {result}.");

                return properties.CalculateInertiaTensor(mass, localReferencePoint);

            case MeshInertiaPolicy.SurfaceApproximation:
                return SurfaceMassProperties.CalculateInertiaTensor(mass, localReferencePoint);

            default:
                throw new ArgumentOutOfRangeException(nameof(policy), policy, "Unsupported mesh inertia policy.");
        }
    }

    /// <summary>
    /// Gets cached closed-volume mass properties when this mesh is a valid closed triangle shell.
    /// </summary>
    public bool TryGetClosedVolumeMassProperties(
        out MeshMassProperties properties,
        out MeshVolumeValidationResult result)
    {
        EnsureScaledClosedVolumeMassProperties();
        properties = _scaledClosedVolumeMassProperties;
        result = _scaledClosedVolumeValidationResult;
        return result == MeshVolumeValidationResult.Valid;
    }

    internal void ValidateClosedVolumeScaleRepresentability(Vector3d scale)
    {
        EnsureClosedVolumeMassProperties();
        if (_closedVolumeValidationResult != MeshVolumeValidationResult.Valid)
            return;

        if (_validatedClosedVolumeMassPropertiesValid
            & _validatedClosedVolumeMassPropertiesScale == scale)
        {
            return;
        }

        if (_scaledClosedVolumeMassPropertiesEvaluated
            & _scaledClosedVolumeMassPropertiesScale == scale)
        {
            if (_scaledClosedVolumeValidationResult != MeshVolumeValidationResult.Valid)
            {
                throw new ArgumentException(
                    $"Mesh scale must preserve a representable closed volume. Validation result: {_scaledClosedVolumeValidationResult}.",
                    nameof(scale));
            }

            _validatedClosedVolumeMassPropertiesScale = scale;
            _validatedClosedVolumeMassProperties = _scaledClosedVolumeMassProperties;
            _validatedClosedVolumeMassPropertiesValid = true;
            return;
        }

        if (!TryCalculateScaledClosedVolumeMassProperties(scale, out MeshMassProperties properties, out MeshVolumeValidationResult result))
        {
            throw new ArgumentException(
                $"Mesh scale must preserve a representable closed volume. Validation result: {result}.",
                nameof(scale));
        }

        _validatedClosedVolumeMassPropertiesScale = scale;
        _validatedClosedVolumeMassProperties = properties;
        _validatedClosedVolumeMassPropertiesValid = true;
    }

    private bool TryCalculateScaledClosedVolumeMassProperties(
        Vector3d scale,
        out MeshMassProperties properties,
        out MeshVolumeValidationResult result)
    {
        EnsureClosedVolumeMassProperties();
        if (_closedVolumeValidationResult != MeshVolumeValidationResult.Valid)
        {
            properties = default;
            result = _closedVolumeValidationResult;
            return false;
        }

        ClosedVolumeScaleEvaluationCount++;
        MeshMassScaleResult scaleResult = _closedVolumeMassProperties.TryScale(scale, out properties);
        if (scaleResult != MeshMassScaleResult.Valid)
        {
            properties = default;
            result = scaleResult == MeshMassScaleResult.NonRepresentableVolume
                ? MeshVolumeValidationResult.NonRepresentableVolume
                : MeshVolumeValidationResult.NonRepresentableMassProperties;
            return false;
        }

        if (properties.Volume <= Fixed64.Epsilon)
        {
            properties = default;
            result = MeshVolumeValidationResult.ZeroVolume;
            return false;
        }

        result = MeshVolumeValidationResult.Valid;
        return true;
    }

    private void EnsureScaledClosedVolumeMassProperties()
    {
        if (_scaledClosedVolumeMassPropertiesEvaluated
            & _scaledClosedVolumeMassPropertiesScale == _scale)
        {
            return;
        }

        TryCalculateScaledClosedVolumeMassProperties(
            _scale,
            out _scaledClosedVolumeMassProperties,
            out _scaledClosedVolumeValidationResult);
        _scaledClosedVolumeMassPropertiesScale = _scale;
        _scaledClosedVolumeMassPropertiesEvaluated = true;
    }

    private void UpdateClosedVolumeMassPropertiesScaleCache()
    {
        _scaledClosedVolumeMassPropertiesEvaluated = false;
        PromoteValidatedClosedVolumeMassProperties();
    }

    private void PromoteValidatedClosedVolumeMassProperties()
    {
        if (!_validatedClosedVolumeMassPropertiesValid
            | _validatedClosedVolumeMassPropertiesScale != _scale)
        {
            return;
        }

        if (_scaledClosedVolumeMassPropertiesEvaluated
            & _scaledClosedVolumeMassPropertiesScale == _scale)
        {
            return;
        }

        _scaledClosedVolumeMassPropertiesScale = _scale;
        _scaledClosedVolumeMassProperties = _validatedClosedVolumeMassProperties;
        _scaledClosedVolumeValidationResult = MeshVolumeValidationResult.Valid;
        _scaledClosedVolumeMassPropertiesEvaluated = true;
    }

    private void EnsureClosedVolumeMassProperties()
    {
        if (_closedVolumeMassPropertiesEvaluated)
            return;

        _closedVolumeMassPropertiesEvaluated = true;

        if (!ValidateClosedVolumeTopology(out MeshVolumeValidationResult topologyResult))
        {
            _closedVolumeValidationResult = topologyResult;
            _closedVolumeMassProperties = default;
            return;
        }

        if (!TryCalculateClosedVolumeMassProperties(out MeshMassProperties properties, out MeshVolumeValidationResult volumeResult))
        {
            _closedVolumeValidationResult = volumeResult;
            _closedVolumeMassProperties = default;
            return;
        }

        _closedVolumeValidationResult = MeshVolumeValidationResult.Valid;
        _closedVolumeMassProperties = properties;
    }

    private bool ValidateClosedVolumeTopology(out MeshVolumeValidationResult result)
    {
        var triangleUses = new TriangleUse[_triangleCount];
        var edgeUses = new EdgeUse[_triangleCount * 3];
        int edgeIndex = 0;
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            int index0 = _triangles[triangleIndex];
            int index1 = _triangles[triangleIndex + 1];
            int index2 = _triangles[triangleIndex + 2];

            triangleUses[i] = TriangleUse.Create(index0, index1, index2);
            edgeUses[edgeIndex++] = EdgeUse.Create(index0, index1, i);
            edgeUses[edgeIndex++] = EdgeUse.Create(index1, index2, i);
            edgeUses[edgeIndex++] = EdgeUse.Create(index2, index0, i);
        }

        if (ContainsDuplicateTriangle(triangleUses))
        {
            result = MeshVolumeValidationResult.DuplicateTriangle;
            return false;
        }

        Array.Sort(edgeUses, CompareEdgeUses);

        int[] parents = new int[_triangleCount];
        for (int i = 0; i < parents.Length; i++)
            parents[i] = i;

        for (int i = 0; i < edgeUses.Length;)
        {
            int groupStart = i;
            EdgeUse first = edgeUses[i++];
            while (i < edgeUses.Length && edgeUses[i].Key == first.Key)
                i++;

            int count = i - groupStart;
            if (count == 1)
            {
                result = MeshVolumeValidationResult.BoundaryEdge;
                return false;
            }

            if (count > 2)
            {
                result = MeshVolumeValidationResult.NonManifoldEdge;
                return false;
            }

            EdgeUse second = edgeUses[groupStart + 1];
            if (first.Direction + second.Direction != 0)
            {
                result = MeshVolumeValidationResult.InconsistentWinding;
                return false;
            }

            Union(parents, first.TriangleIndex, second.TriangleIndex);
        }

        int root = Find(parents, 0);
        for (int i = 1; i < parents.Length; i++)
        {
            if (Find(parents, i) == root)
                continue;

            result = MeshVolumeValidationResult.DisconnectedShell;
            return false;
        }

        result = MeshVolumeValidationResult.Valid;
        return true;
    }

    private bool TryCalculateClosedVolumeMassProperties(
        out MeshMassProperties properties,
        out MeshVolumeValidationResult result)
    {
        Vector3d reference = _localBounds.Center;
        Fixed64 signedVolume = Fixed64.Zero;
        Vector3d firstMoment = Vector3d.Zero;
        Fixed64 integralX2 = Fixed64.Zero;
        Fixed64 integralY2 = Fixed64.Zero;
        Fixed64 integralZ2 = Fixed64.Zero;
        Fixed64 integralXY = Fixed64.Zero;
        Fixed64 integralXZ = Fixed64.Zero;
        Fixed64 integralYZ = Fixed64.Zero;

        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            Vector3d a = _localVertices[_triangles[triangleIndex]] - reference;
            Vector3d b = _localVertices[_triangles[triangleIndex + 1]] - reference;
            Vector3d c = _localVertices[_triangles[triangleIndex + 2]] - reference;

            Fixed64 volume = Vector3d.Dot(a, Vector3d.Cross(b, c)) / TetrahedronVolumeDivisor;
            signedVolume += volume;
            firstMoment += (a + b + c) * (volume / TetrahedronCentroidDivisor);

            Fixed3x3 productSums = Fixed3x3.CreateBarycentricProductSums(a, b, c);
            integralX2 += volume * productSums.M11 / SecondMomentIntegralDivisor;
            integralY2 += volume * productSums.M22 / SecondMomentIntegralDivisor;
            integralZ2 += volume * productSums.M33 / SecondMomentIntegralDivisor;
            integralXY += volume * productSums.M12 / ProductMomentIntegralDivisor;
            integralXZ += volume * productSums.M13 / ProductMomentIntegralDivisor;
            integralYZ += volume * productSums.M23 / ProductMomentIntegralDivisor;
        }

        Fixed64 absoluteVolume = signedVolume.Abs();
        if (absoluteVolume <= Fixed64.Epsilon)
        {
            properties = default;
            result = MeshVolumeValidationResult.ZeroVolume;
            return false;
        }

        Fixed64 orientationSign = signedVolume < Fixed64.Zero ? -Fixed64.One : Fixed64.One;
        Vector3d centerOfMass = reference + firstMoment / signedVolume;
        Fixed64 ixx = orientationSign * (integralY2 + integralZ2) / absoluteVolume;
        Fixed64 iyy = orientationSign * (integralX2 + integralZ2) / absoluteVolume;
        Fixed64 izz = orientationSign * (integralX2 + integralY2) / absoluteVolume;
        Fixed64 ixy = -orientationSign * integralXY / absoluteVolume;
        Fixed64 ixz = -orientationSign * integralXZ / absoluteVolume;
        Fixed64 iyz = -orientationSign * integralYZ / absoluteVolume;

        properties = new MeshMassProperties(
            absoluteVolume,
            centerOfMass,
            reference,
            new Fixed3x3(
                ixx, ixy, ixz,
                ixy, iyy, iyz,
                ixz, iyz, izz));
        result = MeshVolumeValidationResult.Valid;
        return true;
    }

    private static bool ContainsDuplicateTriangle(TriangleUse[] triangleUses)
    {
        Array.Sort(triangleUses, CompareTriangleUses);
        for (int i = 1; i < triangleUses.Length; i++)
        {
            if (CompareTriangleUses(triangleUses[i], triangleUses[i - 1]) == 0)
                return true;
        }

        return false;
    }

    private static int CompareTriangleUses(TriangleUse first, TriangleUse second)
    {
        if (first.A != second.A)
            return first.A < second.A ? -1 : 1;

        if (first.B != second.B)
            return first.B < second.B ? -1 : 1;

        if (first.C != second.C)
            return first.C < second.C ? -1 : 1;

        return 0;
    }

    private static int Find(int[] parents, int index)
    {
        while (parents[index] != index)
        {
            parents[index] = parents[parents[index]];
            index = parents[index];
        }

        return index;
    }

    private static void Union(int[] parents, int first, int second)
    {
        int firstRoot = Find(parents, first);
        int secondRoot = Find(parents, second);
        if (firstRoot == secondRoot)
            return;

        if (firstRoot < secondRoot)
            parents[secondRoot] = firstRoot;
        else
            parents[firstRoot] = secondRoot;
    }
}
