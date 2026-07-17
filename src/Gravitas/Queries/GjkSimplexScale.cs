//=======================================================================
// GjkSimplexScale.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Uniformly scales GJK simplex coordinates before evaluating products whose
/// ratios and signs are invariant under a common scale.
/// </summary>
internal static class GjkSimplexScale
{
    private static readonly Fixed64 ProductSafeComponentLimit = (Fixed64)8;

    /// <summary>
    /// Creates a two-term Minkowski difference in the shared GJK working
    /// coordinate. An exact arithmetic raw shift avoids round-to-even endpoint
    /// overshoot, so halving before subtraction covers every Fixed64 pair.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d CreateWorkingDifference(Vector3d first, Vector3d second) =>
        CreateWorkingDifference(first, second, 1);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d CreateWorkingDifference(Vector3d first, Vector3d second, int shift)
    {
        ValidateShift(shift);
        if (Vector3d.TrySubtract(
            ScaleByPowerOfTwo(first, shift),
            ScaleByPowerOfTwo(second, shift),
            out Vector3d difference))
        {
            return difference;
        }

        throw new InvalidOperationException("The selected GJK working shift does not preserve an exact difference.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d CreateWorkingDifference(Vector2d first, Vector2d second) =>
        CreateWorkingDifference(first, second, 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d CreateWorkingDifference(Vector2d first, Vector2d second, int shift)
    {
        ValidateShift(shift);
        if (Vector2d.TrySubtract(
            ScaleByPowerOfTwo(first, shift),
            ScaleByPowerOfTwo(second, shift),
            out Vector2d difference))
        {
            return difference;
        }

        throw new InvalidOperationException("The selected GJK working shift does not preserve an exact difference.");
    }

    /// <summary>
    /// Creates a three-term Minkowski difference in the shared GJK working
    /// coordinate without first forming a potentially saturated sum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d CreateWorkingDifference(Vector2d first, Vector2d second, Vector2d third) =>
        CreateWorkingDifference(first, second, third, 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d CreateWorkingDifference(
        Vector2d first,
        Vector2d second,
        Vector2d third,
        int shift)
    {
        ValidateShift(shift);
        if (Vector2d.TrySubtract(
                ScaleByPowerOfTwo(first, shift),
                ScaleByPowerOfTwo(second, shift),
                out Vector2d difference)
            && Vector2d.TrySubtract(
                difference,
                ScaleByPowerOfTwo(third, shift),
                out Vector2d result))
        {
            return result;
        }

        throw new InvalidOperationException("The selected GJK working shift does not preserve an exact difference.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RestoreTwoTermDistance(Fixed64 workingDistance) =>
        RestoreDistance(workingDistance, 1);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RestoreThreeTermDistance(Fixed64 workingDistance) =>
        RestoreDistance(workingDistance, 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RestoreDistance(Fixed64 workingDistance, int shift)
    {
        ValidateShift(shift);
        return shift == 0 ? workingDistance : workingDistance * (1 << shift);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetCoordinateScale(int shift)
    {
        ValidateShift(shift);
        return shift switch
        {
            0 => Fixed64.One,
            1 => Fixed64.Half,
            _ => Fixed64.Quarter
        };
    }

    public static int SelectTwoTermShift(
        Vector3d firstMin,
        Vector3d firstMax,
        Vector3d secondMin,
        Vector3d secondMax) =>
        CanSubtractBounds(firstMin, firstMax, secondMin, secondMax) ? 0 : 1;

    public static int SelectThreeTermShift(
        Vector2d point,
        Vector2d targetMin,
        Vector2d targetMax,
        Fixed64 expansionRadius)
    {
        if (CanSubtractExpandedBounds(point, targetMin, targetMax, expansionRadius, 0))
            return 0;

        return CanSubtractExpandedBounds(point, targetMin, targetMax, expansionRadius, 1) ? 1 : 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ScaleByPowerOfTwo(Vector3d value, int shift) =>
        new(
            value.X >> shift,
            value.Y >> shift,
            value.Z >> shift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ScaleByPowerOfTwo(Vector2d value, int shift) =>
        new(
            value.X >> shift,
            value.Y >> shift);

    private static bool CanSubtractBounds(
        Vector3d firstMin,
        Vector3d firstMax,
        Vector3d secondMin,
        Vector3d secondMax) =>
        Vector3d.TrySubtract(firstMax, secondMin, out _)
            && Vector3d.TrySubtract(firstMin, secondMax, out _);

    private static bool CanSubtractExpandedBounds(
        Vector2d point,
        Vector2d targetMin,
        Vector2d targetMax,
        Fixed64 expansionRadius,
        int shift)
    {
        Vector2d scaledPoint = ScaleByPowerOfTwo(point, shift);
        Vector2d scaledMin = ScaleByPowerOfTwo(targetMin, shift);
        Vector2d scaledMax = ScaleByPowerOfTwo(targetMax, shift);
        Fixed64 scaledRadius = ScaleRadiusCeiling(expansionRadius, shift);
        Vector2d radius = new(scaledRadius, scaledRadius);

        return Vector2d.TrySubtract(scaledPoint, scaledMin, out Vector2d positiveDifference)
            && Vector2d.TryAdd(positiveDifference, radius, out _)
            && Vector2d.TrySubtract(scaledPoint, scaledMax, out Vector2d negativeDifference)
            && Vector2d.TrySubtract(negativeDifference, radius, out _);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ScaleRadiusCeiling(Fixed64 radius, int shift)
    {
        Fixed64 scaled = radius >> shift;
        if (shift == 0 || (scaled << shift) == radius)
            return scaled;

        return scaled + Fixed64.MinIncrement;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateShift(int shift)
    {
        if ((uint)shift > 2U)
            throw new ArgumentOutOfRangeException(nameof(shift), "GJK coordinate shifts must be between zero and two.");
    }

    public static Fixed64 ScaleForProducts(Span<Vector3d> points)
    {
        Fixed64 largestComponent = Fixed64.Zero;
        for (int i = 0; i < points.Length; i++)
        {
            Vector3d point = points[i];
            largestComponent = FixedMath.Max(
                largestComponent,
                FixedMath.Max(point.X.Abs(), FixedMath.Max(point.Y.Abs(), point.Z.Abs())));
        }

        Fixed64 scale = Fixed64.One;
        while (largestComponent > ProductSafeComponentLimit)
        {
            largestComponent *= Fixed64.Half;
            scale *= Fixed64.Half;
        }

        if (scale == Fixed64.One)
            return scale;

        // With components <= 8, 3D simplex differences are <= 16 and the
        // largest tetrahedron face-side product remains inside Fixed64.
        for (int i = 0; i < points.Length; i++)
            points[i] *= scale;

        return scale;
    }

    public static Fixed64 ScaleForProducts(Span<Vector2d> points)
    {
        Fixed64 largestComponent = Fixed64.Zero;
        for (int i = 0; i < points.Length; i++)
        {
            Vector2d point = points[i];
            largestComponent = FixedMath.Max(
                largestComponent,
                FixedMath.Max(point.X.Abs(), point.Y.Abs()));
        }

        Fixed64 scale = Fixed64.One;
        while (largestComponent > ProductSafeComponentLimit)
        {
            largestComponent *= Fixed64.Half;
            scale *= Fixed64.Half;
        }

        if (scale == Fixed64.One)
            return scale;

        for (int i = 0; i < points.Length; i++)
            points[i] *= scale;

        return scale;
    }
}
