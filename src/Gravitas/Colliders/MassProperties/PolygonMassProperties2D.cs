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
    private readonly Signed320 _centeredFirstX;
    private readonly Signed320 _centeredFirstY;
    private readonly Signed320 _momentDenominator;

    private PolygonMassProperties2D(
        ExactMassWeight weight,
        Vector2d centroid,
        Signed320 centeredIntegral,
        Signed320 centeredFirstX,
        Signed320 centeredFirstY,
        Signed320 momentDenominator)
    {
        Weight = weight;
        Centroid = centroid;
        _centeredIntegral = centeredIntegral;
        _centeredFirstX = centeredFirstX;
        _centeredFirstY = centeredFirstY;
        _momentDenominator = momentDenominator;
    }

    internal static PolygonMassProperties2D Create(ReadOnlySpan<Vector2d> vertices)
    {
        if (!WideConvex2dRelations.TryGetSignedDoubleAreaAndCentroid(
                vertices, out Signed320 signedDoubleArea, out Vector2d centroid))
            return default;

        Signed320 integral = default;
        Signed320 firstX = default;
        Signed320 firstY = default;
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
            firstX = WideArithmetic.AddSigned320(firstX, Signed320.NarrowValue(
                WideArithmetic.MultiplySigned320(cross, WideArithmetic.AddSigned192(ax, bx))));
            firstY = WideArithmetic.AddSigned320(firstY, Signed320.NarrowValue(
                WideArithmetic.MultiplySigned320(cross, WideArithmetic.AddSigned192(ay, by))));
        }

        // Degree-four raw-coordinate products plus an int-bounded vertex count
        // fit 320 bits. Retain their ratio until mass is known; density can round to zero.
        Signed320 denominator = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(signedDoubleArea),
            Signed192.One, Signed192.One, Signed192.Signed(6)));
        return new PolygonMassProperties2D(
            ExactMassProperties.CreateAreaWeight(signedDoubleArea), centroid, integral, firstX, firstY, denominator);
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

    /// <summary>
    /// Gets the centered integral weighted by semantic area and the compound's
    /// common moment factor of sixty, retaining Q224 products without division.
    /// </summary>
    internal Signed576 CalculateWeightedMoment()
    {
        // Scaled area is 3*|doubleArea|*Q^2. Its area factor cancels
        // I/m = integral/(6*doubleArea*Q^2), leaving 30*|integral|*Q^3.
        Signed576 integral = Signed576.ExtendValue(_centeredIntegral);
        if (_momentDenominator.Sign < 0)
            integral = WideArithmetic.SubtractSigned576(default, integral);
        return WideArithmetic.MultiplySigned576(integral,
            Signed192.One, Signed192.One, Signed192.One, Signed192.Signed(30));
    }

    internal Signed576 CalculateWeightedFirstMomentContribution(Fixed64 rotation, Signed320 dx, Signed320 dy)
    {
        // Area weighting and the compound factor sixty give 120*Q² times the dot product.
        Signed576 dot = CalculateFirstMomentDot(rotation, dx, dy);
        if (_momentDenominator.Sign < 0)
            dot = WideArithmetic.SubtractSigned576(default, dot);
        return WideArithmetic.MultiplySigned576(dot,
            Signed192.One, Signed192.One, Signed192.Signed(120));
    }

    private Signed576 CalculateFirstMomentDot(Fixed64 rotation, Signed320 dx, Signed320 dy)
    {
        if (_centeredFirstX.IsZero && _centeredFirstY.IsZero)
            return default;

        // The published centroid is rounded, so its centered first moments need
        // not vanish. With d = centroid-reference, add W|d|² + 2*d·F(c); omitting F loses accuracy
        // whenever the requested reference differs from that rounded centroid.
        Signed320 firstX = _centeredFirstX;
        Signed320 firstY = _centeredFirstY;
        Signed192 cosine = Signed192.Raw(FixedMath.Cos(rotation));
        Signed192 sine = Signed192.Raw(FixedMath.Sin(rotation));
        Signed320 rotatedX = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(firstX, cosine), WideArithmetic.MultiplySigned320(firstY, sine)));
        Signed320 rotatedY = Signed320.NarrowValue(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(firstX, sine), WideArithmetic.MultiplySigned320(firstY, cosine)));
        // Degree-three edge sums, an int-bounded vertex count, and these fixed
        // factors fit 320 bits before rotation and 576 bits after translation.
        return WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(rotatedX, dx), WideArithmetic.MultiplySigned320(rotatedY, dy));
    }

    internal bool TryCalculateMoment(Fixed64 mass, Fixed64 rotation,
        Signed320 dx, Signed320 dy, out Fixed64 moment)
    {
        // Cancel compound weighting before evaluation: A=6*signedDoubleAreaRaw,
        // N=I*Q²+A*|d|²+4*d·Rot(F), D=A*Q^4, with d in Q64.
        // Semantic coordinates fit 130 bits; N*mass fits 576 and D fits 320.
        Signed192 areaFactor = WideArithmetic.TruncateSigned320By64(_momentDenominator);
        Signed320 squaredDistance = Signed320.NarrowValue(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(dx, dx), WideArithmetic.MultiplySigned320(dy, dy)));
        Signed576 parallelAxis = WideArithmetic.MultiplySigned320(squaredDistance, areaFactor);
        Signed576 intrinsic = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(_centeredIntegral), Signed192.One, Signed192.One);
        Signed576 geometry = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(intrinsic, parallelAxis),
            WideArithmetic.MultiplySigned576(CalculateFirstMomentDot(rotation, dx, dy), 4));
        Signed320 denominator = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(_momentDenominator), Signed192.One, Signed192.One));
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576(geometry, Signed192.Raw(mass)),
            denominator, out moment);
    }

    private static Signed320 SquaredSum(Signed192 first, Signed192 second) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(first, first),
                WideArithmetic.MultiplySigned192(first, second)),
            WideArithmetic.MultiplySigned192(second, second));
}
