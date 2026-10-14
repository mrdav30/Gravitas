//=======================================================================
// PhysicsMesh.MassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    private static readonly Signed576 ClosedVolumeRawDenominator =
        WideArithmetic.MultiplySigned576(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.One)), Signed192.One, Signed192.Signed(6));

    private bool _closedVolumeMassPropertiesEvaluated;
    private MeshMassProperties _closedVolumeMassProperties;
    private MeshVolumeValidationResult _closedVolumeValidationResult;
    private bool _scaledClosedVolumeMassPropertiesEvaluated;
    private MeshMassProperties _scaledClosedVolumeMassProperties;
    private MeshVolumeValidationResult _scaledClosedVolumeValidationResult;
    private bool _preparedClosedVolumeMassPropertiesEvaluated;
    private MeshMassProperties _preparedClosedVolumeMassProperties;
    private MeshVolumeValidationResult _preparedClosedVolumeValidationResult;
    private bool _preparedClosedVolumeEvaluationIncrement;
    private MeshVolumeValidationResult _surfaceClosureValidationResult;

    internal int ClosedVolumeScaleEvaluationCount { get; private set; }

    /// <summary>Calculates closed-volume mesh inertia for the supplied mass.</summary>
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

    internal bool TryGetPreparedClosedVolumeMassProperties(
        out MeshMassProperties properties,
        out MeshVolumeValidationResult result)
    {
        properties = _preparedClosedVolumeMassProperties;
        result = _preparedClosedVolumeValidationResult;
        return _preparedClosedVolumeMassPropertiesEvaluated
            && result == MeshVolumeValidationResult.Valid;
    }

    private void PrepareClosedVolumeMassProperties(
        MeshInertiaPolicy? inertiaPolicy,
        bool geometryChanged,
        Vector3d ownerScale,
        Vector3d partScale)
    {
        _preparedClosedVolumeEvaluationIncrement = false;
        if (!geometryChanged)
        {
            _preparedClosedVolumeMassPropertiesEvaluated =
                _scaledClosedVolumeMassPropertiesEvaluated;
            _preparedClosedVolumeMassProperties =
                _scaledClosedVolumeMassProperties;
            _preparedClosedVolumeValidationResult =
                _scaledClosedVolumeValidationResult;
            return;
        }

        _preparedClosedVolumeMassPropertiesEvaluated = false;
        if (!IsClosedSurface)
        {
            _preparedClosedVolumeMassPropertiesEvaluated = true;
            _preparedClosedVolumeValidationResult = _surfaceClosureValidationResult;
            _preparedClosedVolumeMassProperties = default;
            return;
        }

        if (inertiaPolicy != MeshInertiaPolicy.RequireClosedVolume)
            return;

        _preparedClosedVolumeMassPropertiesEvaluated = true;
        _preparedClosedVolumeEvaluationIncrement = true;
        if (!TryCalculateCandidateClosedVolumeMassProperties(
                ownerScale,
                partScale,
                usePreparedVertices: true,
                out _preparedClosedVolumeMassProperties,
                out _preparedClosedVolumeValidationResult))
        {
            throw new ArgumentException(
                $"Mesh scale must preserve a representable closed volume. Validation result: {_preparedClosedVolumeValidationResult}.",
                nameof(inertiaPolicy));
        }
    }

    private bool TryCalculateCandidateClosedVolumeMassProperties(
        Vector3d ownerScale,
        Vector3d partScale,
        bool usePreparedVertices,
        out MeshMassProperties properties,
        out MeshVolumeValidationResult result)
    {
        Fixed64 x = default;
        Fixed64 y = default;
        Fixed64 z = default;
        if (Fixed64.TryMultiplyDivide(
                ownerScale.X,
                partScale.X,
                Fixed64.One,
                out x)
            && Fixed64.TryMultiplyDivide(
                ownerScale.Y,
                partScale.Y,
                Fixed64.One,
                out y)
            && Fixed64.TryMultiplyDivide(
                ownerScale.Z,
                partScale.Z,
                Fixed64.One,
                out z))
        {
            EnsureClosedVolumeMassProperties();
            properties = _closedVolumeMassProperties;
            Vector3d scale = new(x, y, z);
            // Identity preserves the integrated rounded tensor; recovering its
            // covariance would round halves again without changing geometry.
            if (_closedVolumeValidationResult == MeshVolumeValidationResult.Valid
                && (scale == Vector3d.One
                    || _closedVolumeMassProperties.TryScale(scale, out properties) == MeshMassScaleResult.Valid))
            {
                result = properties.Volume > Fixed64.Epsilon
                    ? MeshVolumeValidationResult.Valid
                    : MeshVolumeValidationResult.ZeroVolume;
                return result == MeshVolumeValidationResult.Valid;
            }

            // A source/cache ratio can fail although the already prepared final
            // geometry is representable (compensated scales or a tiny source).
            // Retain the fast cache path; evaluate final vertices only on failure.
        }

        if (usePreparedVertices)
        {
            return TryCalculateClosedVolumeMassProperties(
                _preparedScaledLocalVertices,
                _preparedScaledLocalBounds,
                out properties,
                out result);
        }

        return TryCalculateClosedVolumeMassProperties(
            _scaledLocalVertices,
            _scaledLocalBounds,
            out properties,
            out result);
    }

    private void PublishPreparedClosedVolumeMassProperties()
    {
        _scaledClosedVolumeMassPropertiesEvaluated =
            _preparedClosedVolumeMassPropertiesEvaluated;
        _scaledClosedVolumeMassProperties =
            _preparedClosedVolumeMassProperties;
        _scaledClosedVolumeValidationResult =
            _preparedClosedVolumeValidationResult;
        if (_preparedClosedVolumeEvaluationIncrement)
            ClosedVolumeScaleEvaluationCount++;
    }

    private void EnsureScaledClosedVolumeMassProperties()
    {
        if (_scaledClosedVolumeMassPropertiesEvaluated)
            return;

        _scaledClosedVolumeMassPropertiesEvaluated = true;
        ClosedVolumeScaleEvaluationCount++;
        TryCalculateCandidateClosedVolumeMassProperties(
            _ownerScale,
            _partScale,
            usePreparedVertices: false,
            out _scaledClosedVolumeMassProperties,
            out _scaledClosedVolumeValidationResult);
    }

    private void EnsureClosedVolumeMassProperties()
    {
        if (_closedVolumeMassPropertiesEvaluated)
            return;

        _closedVolumeMassPropertiesEvaluated = true;
        TryCalculateClosedVolumeMassProperties(
            _localVertices,
            _localBounds,
            out _closedVolumeMassProperties,
            out _closedVolumeValidationResult,
            recenter: true);
    }

    private bool EvaluateClosedVolumeTopology(int[] triangles, out MeshVolumeValidationResult result)
    {
        var triangleUses = new TriangleUse[_triangleCount];
        var edgeUses = new EdgeUse[_triangleCount * 3];
        int edgeIndex = 0;
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            int index0 = triangles[triangleIndex];
            int index1 = triangles[triangleIndex + 1];
            int index2 = triangles[triangleIndex + 2];

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

        if (!HasSingleVertexLinks(triangles, edgeUses))
        {
            result = MeshVolumeValidationResult.NonManifoldVertex;
            return false;
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
        ReadOnlySpan<Vector3d> vertices,
        FixedBoundBox bounds,
        out MeshMassProperties properties,
        out MeshVolumeValidationResult result,
        bool recenter = false)
    {
        Vector3d reference = bounds.Center;
        IntegrateClosedVolume(vertices, reference, out Signed576 signedVolume,
            out Signed576 firstX, out Signed576 firstY, out Signed576 firstZ,
            out Signed576 integralX2, out Signed576 integralY2, out Signed576 integralZ2,
            out Signed576 integralXY, out Signed576 integralXZ, out Signed576 integralYZ);

        Signed576 absoluteWeight = signedVolume.Sign < 0
            ? WideArithmetic.SubtractSigned576(default, signedVolume) : signedVolume;
        properties = default;
        if (!Fixed64.TryGetSignedRawRatio(absoluteWeight, ClosedVolumeRawDenominator, out Fixed64 absoluteVolume))
        {
            result = MeshVolumeValidationResult.NonRepresentableVolume;
            return false;
        }
        if (absoluteVolume <= Fixed64.Epsilon)
        {
            result = MeshVolumeValidationResult.ZeroVolume;
            return false;
        }

        // Source properties are cached in the same centered frame as final
        // vertices. Rounding an authored centroid then subtracting its origin
        // loses half-even parity and makes equivalent authoring frames disagree.
        Vector3d outputReference = recenter ? Vector3d.Zero : reference;
        Signed576 centerDenominator = WideArithmetic.MultiplySigned576(signedVolume, Signed192.Signed(4));
        Signed576 diagonalDenominator = WideArithmetic.MultiplySigned576(signedVolume, Signed192.Signed(10), Signed192.One);
        Signed576 productDenominator = WideArithmetic.MultiplySigned576(signedVolume, Signed192.Signed(20), Signed192.One);
        // Signed ratios cancel winding. Add the reference before rounding so
        // half-even centroid ties depend on the final coordinate's raw parity.
        bool representable = TryGetIntegratedCenter(firstX, centerDenominator, outputReference.X, out Fixed64 centerX)
            & TryGetIntegratedCenter(firstY, centerDenominator, outputReference.Y, out Fixed64 centerY)
            & TryGetIntegratedCenter(firstZ, centerDenominator, outputReference.Z, out Fixed64 centerZ)
            & Fixed64.TryGetSignedRawRatio(WideArithmetic.AddSigned576(integralY2, integralZ2), diagonalDenominator, out Fixed64 ixx)
            & Fixed64.TryGetSignedRawRatio(WideArithmetic.AddSigned576(integralX2, integralZ2), diagonalDenominator, out Fixed64 iyy)
            & Fixed64.TryGetSignedRawRatio(WideArithmetic.AddSigned576(integralX2, integralY2), diagonalDenominator, out Fixed64 izz)
            & Fixed64.TryGetSignedRawRatio(WideArithmetic.SubtractSigned576(default, integralXY), productDenominator, out Fixed64 ixy)
            & Fixed64.TryGetSignedRawRatio(WideArithmetic.SubtractSigned576(default, integralXZ), productDenominator, out Fixed64 ixz)
            & Fixed64.TryGetSignedRawRatio(WideArithmetic.SubtractSigned576(default, integralYZ), productDenominator, out Fixed64 iyz);
        if (!representable)
        {
            result = MeshVolumeValidationResult.NonRepresentableMassProperties;
            return false;
        }

        properties = new MeshMassProperties(
            absoluteVolume,
            new Vector3d(centerX, centerY, centerZ),
            outputReference,
            new Fixed3x3(
                ixx, ixy, ixz,
                ixy, iyy, iyz,
                ixz, iyz, izz));
        result = MeshVolumeValidationResult.Valid;
        return true;
    }

    private void IntegrateClosedVolume(ReadOnlySpan<Vector3d> vertices, Vector3d reference,
        out Signed576 signedVolume,
        out Signed576 firstX, out Signed576 firstY, out Signed576 firstZ,
        out Signed576 integralX2, out Signed576 integralY2, out Signed576 integralZ2,
        out Signed576 integralXY, out Signed576 integralXZ, out Signed576 integralYZ)
    {
        signedVolume = default;
        firstX = firstY = firstZ = default;
        integralX2 = integralY2 = integralZ2 = default;
        integralXY = integralXZ = integralYZ = default;

        // Raw determinant weights retain all tetrahedra until normalization.
        // Degree-five products plus int-bounded triangle accumulation fit 576 bits.
        for (int i = 0; i < _triangleCount; i++)
        {
            int triangleIndex = i * 3;
            // The bounds midpoint keeps each coordinate difference representable,
            // including the full signed raw span; subsequent products stay wide.
            var triangle = new FixedTriangle(
                vertices[_triangles[triangleIndex]] - reference,
                vertices[_triangles[triangleIndex + 1]] - reference,
                vertices[_triangles[triangleIndex + 2]] - reference);
            Signed320 weight = GetTetrahedronDeterminant(triangle);
            signedVolume = WideArithmetic.AddSigned576(signedVolume, Signed576.ExtendValue(weight));
            firstX = TriangleShellMassProperties.AddWeightedVertexSum(firstX, triangle.A.X, triangle.B.X, triangle.C.X, weight);
            firstY = TriangleShellMassProperties.AddWeightedVertexSum(firstY, triangle.A.Y, triangle.B.Y, triangle.C.Y, weight);
            firstZ = TriangleShellMassProperties.AddWeightedVertexSum(firstZ, triangle.A.Z, triangle.B.Z, triangle.C.Z, weight);

            TriangleShellMassProperties.GetProductSums(triangle,
                out Signed320 x2, out Signed320 y2, out Signed320 z2,
                out Signed320 xy, out Signed320 xz, out Signed320 yz);
            integralX2 = WideArithmetic.AddSigned576(integralX2, WideArithmetic.MultiplySigned320(weight, x2));
            integralY2 = WideArithmetic.AddSigned576(integralY2, WideArithmetic.MultiplySigned320(weight, y2));
            integralZ2 = WideArithmetic.AddSigned576(integralZ2, WideArithmetic.MultiplySigned320(weight, z2));
            integralXY = WideArithmetic.AddSigned576(integralXY, WideArithmetic.MultiplySigned320(weight, xy));
            integralXZ = WideArithmetic.AddSigned576(integralXZ, WideArithmetic.MultiplySigned320(weight, xz));
            integralYZ = WideArithmetic.AddSigned576(integralYZ, WideArithmetic.MultiplySigned320(weight, yz));
        }

    }

    private static bool TryGetIntegratedCenter(Signed576 firstMoment, Signed576 denominator, Fixed64 reference, out Fixed64 center) =>
        Fixed64.TryGetSignedRawRatio(WideArithmetic.AddSigned576(firstMoment,
            WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(reference))), denominator, out center);

    private static Signed320 GetTetrahedronDeterminant(in FixedTriangle triangle)
    {
        Vector3d a = triangle.A, b = triangle.B, c = triangle.C;
        Signed192 x = WideGeometry.GetDifferenceCrossProduct2D(b.Y, Fixed64.Zero, b.Z, Fixed64.Zero, c.Y, Fixed64.Zero, c.Z, Fixed64.Zero);
        Signed192 y = WideGeometry.GetDifferenceCrossProduct2D(b.Z, Fixed64.Zero, b.X, Fixed64.Zero, c.Z, Fixed64.Zero, c.X, Fixed64.Zero);
        Signed192 z = WideGeometry.GetDifferenceCrossProduct2D(b.X, Fixed64.Zero, b.Y, Fixed64.Zero, c.X, Fixed64.Zero, c.Y, Fixed64.Zero);
        return WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(a.X), x),
            WideArithmetic.MultiplySigned192(Signed192.Raw(a.Y), y)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(a.Z), z));
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
