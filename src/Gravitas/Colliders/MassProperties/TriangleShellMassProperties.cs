//=======================================================================
// TriangleShellMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>
/// Owns wide uniform thin-shell integration for indexed triangle surfaces.
/// </summary>
internal static class TriangleShellMassProperties
{
    private static readonly Signed192 Three = Signed192.Signed(3);
    private static readonly Signed192 Six = Signed192.Signed(6);
    private static readonly Signed192 Twelve = Signed192.Signed(12);

    internal static bool TryCreateUniformShell(
        ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> triangleIndices,
        out ExactMassWeight surfaceWeight,
        out Vector3d centerOfMass,
        out Fixed3x3 unitMassInertiaTensor,
        out MeshMassIntegrals integrals,
        in ScaledRigidFrame3d sourceFrame = default,
        bool exactScale = false)
    {
        integrals = default;
        if (triangleIndices.Length % FixedTriangle.VertexCount != 0)
        {
            throw new ArgumentException(
                "Triangle indices must contain complete index triplets.",
                nameof(triangleIndices));
        }

        Signed576 totalWeight = default;
        Signed576 firstMomentX = default;
        Signed576 firstMomentY = default;
        Signed576 firstMomentZ = default;
        Signed576 secondMomentX = default;
        Signed576 secondMomentY = default;
        Signed576 secondMomentZ = default;
        Signed576 productMomentXY = default;
        Signed576 productMomentXZ = default;
        Signed576 productMomentYZ = default;
        for (int i = 0; i < triangleIndices.Length; i += FixedTriangle.VertexCount)
        {
            GetTriangle(
                vertices,
                triangleIndices,
                i,
                out FixedTriangle triangle);
            var coordinates = new TriangleCoordinates(triangle, sourceFrame, exactScale);
            ExactMassWeight weight;
            if (exactScale)
            {
                WideDirection3d normal = WideTriangleRelations.GetScaledTriangleLocalNormal(triangle, sourceFrame);
                // Q128 area weights retain the exact scaled surface even when
                // its Q32.32 inspection vertices would collapse to a line.
                Signed576 root = WideArithmetic.GetFloorSquareRootOfProduct(normal.SquaredLength, Signed192.Signed(9));
                Signed192 numerator = WideArithmetic.DivideSigned576ToSigned192(root, new Signed320(0, 0, 0, 1, 0));
                weight = new ExactMassWeight(Signed320.ExtendValue(numerator));
            }
            else
            {
                triangle.GetExactNormal(out _, out _, out _, out Signed320 squaredDoubleArea);
                weight = ExactMassProperties.CreateTriangleAreaWeight(squaredDoubleArea);
            }
            Signed320 weightNumerator = weight.Numerator;
            totalWeight = WideArithmetic.AddSigned576(
                totalWeight,
                Signed576.ExtendValue(weightNumerator));
            firstMomentX = AddWeightedVertexSum(
                firstMomentX,
                coordinates.AX,
                coordinates.BX,
                coordinates.CX,
                weightNumerator);
            firstMomentY = AddWeightedVertexSum(
                firstMomentY,
                coordinates.AY,
                coordinates.BY,
                coordinates.CY,
                weightNumerator);
            firstMomentZ = AddWeightedVertexSum(
                firstMomentZ,
                coordinates.AZ,
                coordinates.BZ,
                coordinates.CZ,
                weightNumerator);
            GetProductSums(
                coordinates,
                out Signed320 x2,
                out Signed320 y2,
                out Signed320 z2,
                out Signed320 xy,
                out Signed320 xz,
                out Signed320 yz);
            secondMomentX = AddWeighted(
                secondMomentX,
                x2,
                weightNumerator);
            secondMomentY = AddWeighted(
                secondMomentY,
                y2,
                weightNumerator);
            secondMomentZ = AddWeighted(
                secondMomentZ,
                z2,
                weightNumerator);
            productMomentXY = AddWeighted(
                productMomentXY,
                xy,
                weightNumerator);
            productMomentXZ = AddWeighted(
                productMomentXZ,
                xz,
                weightNumerator);
            productMomentYZ = AddWeighted(
                productMomentYZ,
                yz,
                weightNumerator);
        }

        if (totalWeight.IsZero)
        {
            surfaceWeight = default;
            centerOfMass = default;
            unitMassInertiaTensor = default;
            integrals = default;
            return false;
        }

        surfaceWeight = new ExactMassWeight(
            Signed320.NarrowValue(totalWeight));
        integrals = MeshMassIntegrals.FromShell(surfaceWeight,
            firstMomentX, firstMomentY, firstMomentZ,
            secondMomentX, secondMomentY, secondMomentZ,
            productMomentXY, productMomentXZ, productMomentYZ, exactScale ? 64 : 0);
        Signed576 centerDenominator =
            WideArithmetic.MultiplySigned576(
                totalWeight,
                exactScale ? Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Three, new Signed192(0, 1, 0))) : Three);
        _ = Fixed64.TryGetSignedRawRatio(
            firstMomentX,
            centerDenominator,
            out Fixed64 centerX);
        _ = Fixed64.TryGetSignedRawRatio(
            firstMomentY,
            centerDenominator,
            out Fixed64 centerY);
        _ = Fixed64.TryGetSignedRawRatio(
            firstMomentZ,
            centerDenominator,
            out Fixed64 centerZ);
        centerOfMass = new Vector3d(centerX, centerY, centerZ);

        Signed576 centeredX = TranslateSquaredMoment(
            secondMomentX,
            firstMomentX,
            totalWeight,
            exactScale ? ScaleCoordinate(centerX) : Signed192.Raw(centerX));
        Signed576 centeredY = TranslateSquaredMoment(
            secondMomentY,
            firstMomentY,
            totalWeight,
            exactScale ? ScaleCoordinate(centerY) : Signed192.Raw(centerY));
        Signed576 centeredZ = TranslateSquaredMoment(
            secondMomentZ,
            firstMomentZ,
            totalWeight,
            exactScale ? ScaleCoordinate(centerZ) : Signed192.Raw(centerZ));
        Signed576 centeredXY = TranslateProductMoment(
            productMomentXY,
            firstMomentX,
            firstMomentY,
            totalWeight,
            exactScale ? ScaleCoordinate(centerX) : Signed192.Raw(centerX),
            exactScale ? ScaleCoordinate(centerY) : Signed192.Raw(centerY));
        Signed576 centeredXZ = TranslateProductMoment(
            productMomentXZ,
            firstMomentX,
            firstMomentZ,
            totalWeight,
            exactScale ? ScaleCoordinate(centerX) : Signed192.Raw(centerX),
            exactScale ? ScaleCoordinate(centerZ) : Signed192.Raw(centerZ));
        Signed576 centeredYZ = TranslateProductMoment(
            productMomentYZ,
            firstMomentY,
            firstMomentZ,
            totalWeight,
            exactScale ? ScaleCoordinate(centerY) : Signed192.Raw(centerY),
            exactScale ? ScaleCoordinate(centerZ) : Signed192.Raw(centerZ));
        Signed576 inertiaXX =
            WideArithmetic.AddSigned576(centeredY, centeredZ);
        Signed576 inertiaYY =
            WideArithmetic.AddSigned576(centeredX, centeredZ);
        Signed576 inertiaZZ =
            WideArithmetic.AddSigned576(centeredX, centeredY);

        Signed576 diagonalDenominator =
            WideArithmetic.MultiplySigned576(
                totalWeight,
                exactScale ? new Signed192(1UL << 32, 0, 0) : Signed192.One,
                Six);
        Signed576 productDenominator =
            WideArithmetic.MultiplySigned576(
                totalWeight,
                exactScale ? new Signed192(1UL << 32, 0, 0) : Signed192.One,
                Twelve);
        bool representable = Fixed64.TryGetSignedRawRatio(
                inertiaXX,
                diagonalDenominator,
                out Fixed64 tensorXX)
            & Fixed64.TryGetSignedRawRatio(
                inertiaYY,
                diagonalDenominator,
                out Fixed64 tensorYY)
            & Fixed64.TryGetSignedRawRatio(
                inertiaZZ,
                diagonalDenominator,
                out Fixed64 tensorZZ)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.SubtractSigned576(default, centeredXY),
                productDenominator,
                out Fixed64 tensorXY)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.SubtractSigned576(default, centeredXZ),
                productDenominator,
                out Fixed64 tensorXZ)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.SubtractSigned576(default, centeredYZ),
                productDenominator,
                out Fixed64 tensorYZ);
        if (!representable)
        {
            surfaceWeight = default;
            centerOfMass = default;
            unitMassInertiaTensor = default;
            integrals = default;
            return false;
        }

        unitMassInertiaTensor = new Fixed3x3(
            tensorXX, tensorXY, tensorXZ,
            tensorXY, tensorYY, tensorYZ,
            tensorXZ, tensorYZ, tensorZZ);
        return true;
    }

    internal static Signed576 AddWeightedVertexSum(Signed576 total, Signed192 first, Signed192 second, Signed192 third, Signed320 weight)
    {
        Signed320 sum = WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(
            Signed320.ExtendValue(first), Signed320.ExtendValue(second)), Signed320.ExtendValue(third));
        return AddWeighted(total, sum, weight);
    }

    private static Signed576 AddWeighted(
        Signed576 total,
        Signed320 value,
        Signed320 weight) =>
        WideArithmetic.AddSigned576(
            total,
            WideArithmetic.MultiplySigned320(
                value,
                weight));

    internal static void GetProductSums(in TriangleCoordinates triangle,
        out Signed320 x2, out Signed320 y2, out Signed320 z2, out Signed320 xy, out Signed320 xz, out Signed320 yz)
    {
        Signed192 ax = triangle.AX, ay = triangle.AY, az = triangle.AZ;
        Signed192 bx = triangle.BX, by = triangle.BY, bz = triangle.BZ;
        Signed192 cx = triangle.CX, cy = triangle.CY, cz = triangle.CZ;
        x2 = SquaredBarycentricSum(ax, bx, cx);
        y2 = SquaredBarycentricSum(ay, by, cy);
        z2 = SquaredBarycentricSum(az, bz, cz);
        xy = BarycentricCrossSum(ax, bx, cx, ay, by, cy);
        xz = BarycentricCrossSum(ax, bx, cx, az, bz, cz);
        yz = BarycentricCrossSum(ay, by, cy, az, bz, cz);
    }

    private static Signed320 SquaredBarycentricSum(
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(first, first),
                    WideArithmetic.MultiplySigned192(second, second)),
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(third, third),
                    WideArithmetic.MultiplySigned192(first, second))),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(first, third),
                WideArithmetic.MultiplySigned192(second, third)));

    private static Signed320 BarycentricCrossSum(
        Signed192 firstA,
        Signed192 firstB,
        Signed192 firstC,
        Signed192 secondA,
        Signed192 secondB,
        Signed192 secondC)
    {
        Signed192 firstSum = Signed192.NarrowProven(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    Signed320.ExtendValue(firstA),
                    Signed320.ExtendValue(firstB)),
                Signed320.ExtendValue(firstC)));
        Signed192 secondSum = Signed192.NarrowProven(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    Signed320.ExtendValue(secondA),
                    Signed320.ExtendValue(secondB)),
                Signed320.ExtendValue(secondC)));
        Signed320 matching = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(firstA, secondA),
                WideArithmetic.MultiplySigned192(firstB, secondB)),
            WideArithmetic.MultiplySigned192(firstC, secondC));
        return WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                firstSum,
                secondSum),
            matching);
    }

    private static Signed576 TranslateSquaredMoment(
        Signed576 secondMoment,
        Signed576 firstMoment,
        Signed576 totalWeight,
        Signed192 centerRaw)
    {
        Signed576 shifted = WideArithmetic.SubtractSigned576(
            secondMoment,
            WideArithmetic.MultiplySigned576(
                firstMoment,
                centerRaw,
                Signed192.Signed(4)));
        return WideArithmetic.AddSigned576(
            shifted,
            WideArithmetic.MultiplySigned576(
                totalWeight,
                centerRaw,
                centerRaw,
                Six));
    }

    private static Signed576 TranslateProductMoment(
        Signed576 productMoment,
        Signed576 firstMoment,
        Signed576 secondMoment,
        Signed576 totalWeight,
        Signed192 firstCenterRaw,
        Signed192 secondCenterRaw)
    {
        Signed576 shifted = WideArithmetic.SubtractSigned576(
            productMoment,
            WideArithmetic.MultiplySigned576(
                secondMoment,
                firstCenterRaw,
                Signed192.Signed(4)));
        shifted = WideArithmetic.SubtractSigned576(
            shifted,
            WideArithmetic.MultiplySigned576(
                firstMoment,
                secondCenterRaw,
                Signed192.Signed(4)));
        return WideArithmetic.AddSigned576(
            shifted,
            WideArithmetic.MultiplySigned576(
                totalWeight,
                firstCenterRaw,
                secondCenterRaw,
                Twelve));
    }

    private static Signed192 ScaleCoordinate(Fixed64 value) => Signed192.NarrowValue(
        WideArithmetic.MultiplySigned192(Signed192.Raw(value), new Signed192(0, 1, 0)));

    internal readonly struct TriangleCoordinates
    {
        internal readonly Signed192 AX, AY, AZ, BX, BY, BZ, CX, CY, CZ;
        internal TriangleCoordinates(in FixedTriangle triangle, in ScaledRigidFrame3d frame, bool exactScale)
        {
            if (exactScale)
            {
                frame.GetScaledLocalPointNumerators(triangle.A, out AX, out AY, out AZ);
                frame.GetScaledLocalPointNumerators(triangle.B, out BX, out BY, out BZ);
                frame.GetScaledLocalPointNumerators(triangle.C, out CX, out CY, out CZ);
            }
            else
            {
                AX = Signed192.Raw(triangle.A.X); AY = Signed192.Raw(triangle.A.Y); AZ = Signed192.Raw(triangle.A.Z);
                BX = Signed192.Raw(triangle.B.X); BY = Signed192.Raw(triangle.B.Y); BZ = Signed192.Raw(triangle.B.Z);
                CX = Signed192.Raw(triangle.C.X); CY = Signed192.Raw(triangle.C.Y); CZ = Signed192.Raw(triangle.C.Z);
            }
        }
    }

    private static void GetTriangle(
        ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> triangleIndices,
        int index,
        out FixedTriangle triangle)
    {
        int first = triangleIndices[index];
        int second = triangleIndices[index + 1];
        int third = triangleIndices[index + 2];
        if ((uint)first >= (uint)vertices.Length
            || (uint)second >= (uint)vertices.Length
            || (uint)third >= (uint)vertices.Length)
        {
            throw new ArgumentOutOfRangeException(
                nameof(triangleIndices),
                "Triangle indices must reference supplied vertices.");
        }

        triangle = new FixedTriangle(
            vertices[first],
            vertices[second],
            vertices[third]);
    }
}
