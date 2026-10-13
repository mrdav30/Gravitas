//=======================================================================
// PolygonMassProperties2D.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>
/// Retains convex polygon area, centroid and centered moment integrals without scalar saturation.
/// </summary>
internal readonly struct PolygonMassProperties2D
{
    internal readonly ExactMassWeight Weight;
    internal readonly Vector2d Centroid;
    private readonly Signed320 _centeredIntegral;
    private readonly Signed320 _momentDenominator;

    private PolygonMassProperties2D(
        ExactMassWeight weight,
        Vector2d centroid,
        Signed320 centeredIntegral,
        Signed320 momentDenominator)
    {
        Weight = weight;
        Centroid = centroid;
        _centeredIntegral = centeredIntegral;
        _momentDenominator = momentDenominator;
    }

    internal static PolygonMassProperties2D Create(ReadOnlySpan<Vector2d> vertices)
    {
        if (!WideConvex2dRelations.TryGetSignedDoubleAreaAndCentroid(
                vertices, out Signed320 signedDoubleArea, out Vector2d centroid))
            return default;

        Signed320 integral = default;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2d first = vertices[i];
            Vector2d second = vertices[(i + 1) % vertices.Length];
            Signed192 ax = WideArithmetic.SubtractSigned192(
                Signed192.Raw(first.X), Signed192.Raw(centroid.X));
            Signed192 ay = WideArithmetic.SubtractSigned192(
                Signed192.Raw(first.Y), Signed192.Raw(centroid.Y));
            Signed192 bx = WideArithmetic.SubtractSigned192(
                Signed192.Raw(second.X), Signed192.Raw(centroid.X));
            Signed192 by = WideArithmetic.SubtractSigned192(
                Signed192.Raw(second.Y), Signed192.Raw(centroid.Y));
            Signed320 cross = WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(ax, by), WideArithmetic.MultiplySigned192(ay, bx));
            Signed320 term = WideArithmetic.AddSigned320(
                SquaredSum(ax, bx), SquaredSum(ay, by));
            integral = WideArithmetic.AddSigned320(integral,
                Signed320.NarrowValue(WideArithmetic.MultiplySigned320(cross, term)));
        }

        // Degree-four raw-coordinate products plus an int-bounded vertex count
        // fit 320 bits. Retain their ratio until mass is known; density can round to zero.
        Signed320 denominator = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(signedDoubleArea),
            Signed192.One, Signed192.One, Signed192.Signed(6)));
        return new PolygonMassProperties2D(
            ExactMassProperties.CreateAreaWeight(signedDoubleArea), centroid, integral, denominator);
    }

    internal Fixed64 CalculateMoment(Fixed64 mass)
    {
        if (_momentDenominator.IsZero)
            return Fixed64.Zero;

        // I = mass * integral / (6 * signedDoubleArea * Q^2), about the
        // published Q32.32 centroid. Winding signs cancel before final rounding.
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned320(_centeredIntegral, Signed192.Raw(mass)),
            Signed576.ExtendValue(_momentDenominator), out Fixed64 moment)
            ? moment
            : Fixed64.MaxValue;
    }

    private static Signed320 SquaredSum(Signed192 first, Signed192 second) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(first, first),
                WideArithmetic.MultiplySigned192(first, second)),
            WideArithmetic.MultiplySigned192(second, second));
}
