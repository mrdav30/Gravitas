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
    public static Vector3d CreateWorkingDifference(Vector3d first, Vector3d second, int shift) =>
        ScaleByPowerOfTwo(first, shift) - ScaleByPowerOfTwo(second, shift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d CreateWorkingDifference(Vector2d first, Vector2d second) =>
        CreateWorkingDifference(first, second, 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d CreateWorkingDifference(Vector2d first, Vector2d second, int shift) =>
        ScaleByPowerOfTwo(first, shift) - ScaleByPowerOfTwo(second, shift);

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
        int shift) =>
        ScaleByPowerOfTwo(first, shift)
            - ScaleByPowerOfTwo(second, shift)
            - ScaleByPowerOfTwo(third, shift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RestoreTwoTermDistance(Fixed64 workingDistance) =>
        RestoreDistance(workingDistance, 1);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RestoreThreeTermDistance(Fixed64 workingDistance) =>
        RestoreDistance(workingDistance, 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RestoreDistance(Fixed64 workingDistance, int shift) =>
        shift == 0 ? workingDistance : workingDistance * (1 << shift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetCoordinateScale(int shift) =>
        shift switch
        {
            0 => Fixed64.One,
            1 => Fixed64.Half,
            _ => Fixed64.Quarter
        };

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
            Fixed64.FromRaw(value.X.m_rawValue >> shift),
            Fixed64.FromRaw(value.Y.m_rawValue >> shift),
            Fixed64.FromRaw(value.Z.m_rawValue >> shift));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ScaleByPowerOfTwo(Vector2d value, int shift) =>
        new(
            Fixed64.FromRaw(value.X.m_rawValue >> shift),
            Fixed64.FromRaw(value.Y.m_rawValue >> shift));

    private static bool CanSubtractBounds(
        Vector3d firstMin,
        Vector3d firstMax,
        Vector3d secondMin,
        Vector3d secondMax) =>
        CanSubtract(firstMax.X.m_rawValue, secondMin.X.m_rawValue)
            && CanSubtract(firstMin.X.m_rawValue, secondMax.X.m_rawValue)
            && CanSubtract(firstMax.Y.m_rawValue, secondMin.Y.m_rawValue)
            && CanSubtract(firstMin.Y.m_rawValue, secondMax.Y.m_rawValue)
            && CanSubtract(firstMax.Z.m_rawValue, secondMin.Z.m_rawValue)
            && CanSubtract(firstMin.Z.m_rawValue, secondMax.Z.m_rawValue);

    private static bool CanSubtractExpandedBounds(
        Vector2d point,
        Vector2d targetMin,
        Vector2d targetMax,
        Fixed64 expansionRadius,
        int shift)
    {
        long pointX = point.X.m_rawValue >> shift;
        long pointY = point.Y.m_rawValue >> shift;
        long targetMinX = targetMin.X.m_rawValue >> shift;
        long targetMinY = targetMin.Y.m_rawValue >> shift;
        long targetMaxX = targetMax.X.m_rawValue >> shift;
        long targetMaxY = targetMax.Y.m_rawValue >> shift;
        long radius = expansionRadius.m_rawValue >> shift;

        return CanSubtractThenAdd(pointX, targetMinX, radius)
            && CanSubtractThenSubtract(pointX, targetMaxX, radius)
            && CanSubtractThenAdd(pointY, targetMinY, radius)
            && CanSubtractThenSubtract(pointY, targetMaxY, radius);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanSubtract(long first, long second)
    {
        long result = unchecked(first - second);
        return ((first ^ second) & (first ^ result)) >= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanSubtractThenAdd(long first, long second, long third)
    {
        long difference = unchecked(first - second);
        if (((first ^ second) & (first ^ difference)) < 0)
            return false;

        long result = unchecked(difference + third);
        return ((difference ^ result) & (third ^ result)) >= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanSubtractThenSubtract(long first, long second, long third)
    {
        long difference = unchecked(first - second);
        if (((first ^ second) & (first ^ difference)) < 0)
            return false;

        long result = unchecked(difference - third);
        return ((difference ^ third) & (difference ^ result)) >= 0;
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
