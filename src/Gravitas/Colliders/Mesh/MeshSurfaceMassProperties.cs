//=======================================================================
// MeshSurfaceMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using static Gravitas.Colliders.MeshCheckedMath;

namespace Gravitas.Colliders;

/// <summary>
/// Stores deterministic uniform thin-shell mass properties for scaled mesh triangles.
/// </summary>
public readonly struct MeshSurfaceMassProperties
{
    internal MeshSurfaceMassProperties(
        Fixed64 area,
        Vector3d centerOfMass,
        Vector3d inertiaReferencePoint,
        Fixed3x3 unitMassInertiaTensor)
    {
        Area = area;
        CenterOfMass = centerOfMass;
        InertiaReferencePoint = inertiaReferencePoint;
        UnitMassInertiaTensor = unitMassInertiaTensor;
    }

    /// <summary>Gets the scaled triangle surface area represented by this shell.</summary>
    public Fixed64 Area { get; }

    /// <summary>Gets the shell center of mass in scaled mesh-local coordinates.</summary>
    public Vector3d CenterOfMass { get; }

    /// <summary>Gets the scaled mesh-local point about which the cached unit tensor is expressed.</summary>
    public Vector3d InertiaReferencePoint { get; }

    /// <summary>Gets the unit-mass thin-shell inertia tensor about <see cref="InertiaReferencePoint"/>.</summary>
    public Fixed3x3 UnitMassInertiaTensor { get; }

    /// <summary>Calculates shell inertia for the supplied mass about a scaled mesh-local reference point.</summary>
    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localReferencePoint)
    {
        Fixed3x3 referenceTensor = UnitMassInertiaTensor * mass;
        Fixed3x3 centerTensor = InertiaTensorMath.SubtractParallelAxisTensor(
            referenceTensor,
            mass,
            InertiaReferencePoint - CenterOfMass);
        return InertiaTensorMath.AddParallelAxisTensor(
            centerTensor,
            mass,
            localReferencePoint - CenterOfMass);
    }

    internal static bool TryCreate(
        ReadOnlySpan<Vector3d> localVertices,
        ReadOnlySpan<int> triangles,
        ReadOnlySpan<Fixed64> scaledFaceAreas,
        Vector3d scale,
        Vector3d reference,
        Fixed64 totalArea,
        out MeshSurfaceMassProperties properties)
    {
        properties = default;
        Vector3d firstMoment = Vector3d.Zero;

        for (int i = 0; i < scaledFaceAreas.Length; i++)
        {
            int triangleIndex = i * 3;
            Vector3d a = Vector3d.Multiply(localVertices[triangles[triangleIndex]], scale) - reference;
            Vector3d b = Vector3d.Multiply(localVertices[triangles[triangleIndex + 1]], scale) - reference;
            Vector3d c = Vector3d.Multiply(localVertices[triangles[triangleIndex + 2]], scale) - reference;
            Fixed64 area = scaledFaceAreas[i];
            bool valid = TryAdd(a, b, out Vector3d vertexSum);
            valid &= TryAdd(vertexSum, c, out vertexSum);
            valid &= TryMultiply(vertexSum, area / (Fixed64)3, out Vector3d triangleMoment);
            valid &= TryAdd(firstMoment, triangleMoment, out firstMoment);
            if (!valid)
            {
                return false;
            }
        }

        Vector3d centerRelativeToReference = firstMoment / totalArea;
        Vector3d centerOfMass = reference + centerRelativeToReference;

        Fixed3x3 areaWeightedTensor = Fixed3x3.Zero;
        for (int i = 0; i < scaledFaceAreas.Length; i++)
        {
            int triangleIndex = i * 3;
            Vector3d a = Vector3d.Multiply(localVertices[triangles[triangleIndex]], scale) - reference;
            Vector3d b = Vector3d.Multiply(localVertices[triangles[triangleIndex + 1]], scale) - reference;
            Vector3d c = Vector3d.Multiply(localVertices[triangles[triangleIndex + 2]], scale) - reference;
            bool valid = TryAdd(a, b, out Vector3d triangleSum);
            valid &= TryAdd(triangleSum, c, out triangleSum);
            valid &= TryDivide(triangleSum, (Fixed64)3, out Vector3d triangleCenter);
            valid &= TrySubtract(a, triangleCenter, out Vector3d relativeA);
            valid &= TrySubtract(b, triangleCenter, out Vector3d relativeB);
            valid &= TrySubtract(c, triangleCenter, out Vector3d relativeC);
            valid &= TryCreateBarycentricProductSums(relativeA, relativeB, relativeC, out Fixed3x3 productSums);
            if (!valid)
            {
                return false;
            }

            Fixed64 x2 = productSums.M11 / (Fixed64)6;
            Fixed64 y2 = productSums.M22 / (Fixed64)6;
            Fixed64 z2 = productSums.M33 / (Fixed64)6;
            Fixed64 xy = productSums.M12 / (Fixed64)12;
            Fixed64 xz = productSums.M13 / (Fixed64)12;
            Fixed64 yz = productSums.M23 / (Fixed64)12;
            Fixed3x3 triangleCentralTensor = new(
                y2 + z2, -xy, -xz,
                -xy, x2 + z2, -yz,
                -xz, -yz, x2 + y2);
            valid = TrySubtract(triangleCenter, centerRelativeToReference, out Vector3d centerOffset);
            valid &= TryCreateParallelAxisTensor(centerOffset, out Fixed3x3 parallelAxisTensor);
            valid &= TryAdd(triangleCentralTensor, parallelAxisTensor, out Fixed3x3 triangleTensor);
            valid &= TryMultiply(triangleTensor, scaledFaceAreas[i], out Fixed3x3 weightedTriangleTensor);
            valid &= TryAdd(areaWeightedTensor, weightedTriangleTensor, out areaWeightedTensor);
            if (!valid)
            {
                return false;
            }
        }

        Fixed3x3 unitCenterTensor = areaWeightedTensor * (Fixed64.One / totalArea);

        properties = new MeshSurfaceMassProperties(totalArea, centerOfMass, centerOfMass, unitCenterTensor);
        return true;
    }

    private static bool TryCreateBarycentricProductSums(
        Vector3d a,
        Vector3d b,
        Vector3d c,
        out Fixed3x3 sums)
    {
        sums = default;
        bool valid = TrySquaredBarycentricSum(a.X, b.X, c.X, out Fixed64 xx);
        valid &= TrySquaredBarycentricSum(a.Y, b.Y, c.Y, out Fixed64 yy);
        valid &= TrySquaredBarycentricSum(a.Z, b.Z, c.Z, out Fixed64 zz);
        valid &= TryBarycentricCrossSum(a.X, b.X, c.X, a.Y, b.Y, c.Y, out Fixed64 xy);
        valid &= TryBarycentricCrossSum(a.X, b.X, c.X, a.Z, b.Z, c.Z, out Fixed64 xz);
        valid &= TryBarycentricCrossSum(a.Y, b.Y, c.Y, a.Z, b.Z, c.Z, out Fixed64 yz);
        if (!valid)
        {
            return false;
        }

        sums = new Fixed3x3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
        return true;
    }

    private static bool TrySquaredBarycentricSum(Fixed64 a, Fixed64 b, Fixed64 c, out Fixed64 sum)
    {
        sum = default;
        bool valid = TryMultiply(a, a, out Fixed64 aa);
        valid &= TryMultiply(b, b, out Fixed64 bb);
        valid &= TryMultiply(c, c, out Fixed64 cc);
        valid &= TryMultiply(a, b, out Fixed64 ab);
        valid &= TryMultiply(a, c, out Fixed64 ac);
        valid &= TryMultiply(b, c, out Fixed64 bc);
        valid &= TryAdd(aa, bb, out sum);
        valid &= TryAdd(sum, cc, out sum);
        valid &= TryAdd(sum, ab, out sum);
        valid &= TryAdd(sum, ac, out sum);
        valid &= TryAdd(sum, bc, out sum);
        return valid;
    }

    private static bool TryBarycentricCrossSum(
        Fixed64 firstA,
        Fixed64 firstB,
        Fixed64 firstC,
        Fixed64 secondA,
        Fixed64 secondB,
        Fixed64 secondC,
        out Fixed64 sum)
    {
        sum = default;
        bool valid = TryAdd(firstA, firstB, out Fixed64 firstSum);
        valid &= TryAdd(firstSum, firstC, out firstSum);
        valid &= TryAdd(secondA, secondB, out Fixed64 secondSum);
        valid &= TryAdd(secondSum, secondC, out secondSum);
        valid &= TryMultiply(firstSum, secondSum, out Fixed64 sumProduct);
        valid &= TryMultiply(firstA, secondA, out Fixed64 firstProduct);
        valid &= TryMultiply(firstB, secondB, out Fixed64 secondProduct);
        valid &= TryMultiply(firstC, secondC, out Fixed64 thirdProduct);
        valid &= TryAdd(firstProduct, secondProduct, out Fixed64 matchingProducts);
        valid &= TryAdd(matchingProducts, thirdProduct, out matchingProducts);
        valid &= TryAdd(sumProduct, matchingProducts, out sum);
        return valid;
    }

}
