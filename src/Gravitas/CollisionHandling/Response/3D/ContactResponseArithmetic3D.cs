//=======================================================================
// ContactResponseArithmetic3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Preserves the existing compact response arithmetic when a conservative
/// operand bound proves every intermediate representable, otherwise delegates
/// to FixedMathSharp's checked full-domain operations.
/// </summary>
internal static class ContactResponseArithmetic3D
{
    // For Q32.32 raw magnitudes below 2^n, one product lands below
    // 2^(2n-32). The 46/40/37 bounds leave sign-and-sum headroom for,
    // respectively, three products, the point-velocity chain, and the
    // cross/matrix/cross/dot angular-response chain.
    private const int SafeProductMagnitudeShift = 46;
    private const int SafePointVelocityMagnitudeShift = 40;
    private const int SafeAngularResponseMagnitudeShift = 37;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool CanUseFastPointVelocity(
        Vector3d linearA,
        Vector3d angularA,
        Vector3d leverA,
        Vector3d linearB,
        Vector3d angularB,
        Vector3d leverB,
        Vector3d axis) =>
        IsSafeMagnitude(
            GetAggregateMagnitude(linearA)
            | GetAggregateMagnitude(angularA)
            | GetAggregateMagnitude(leverA)
            | GetAggregateMagnitude(linearB)
            | GetAggregateMagnitude(angularB)
            | GetAggregateMagnitude(leverB)
            | GetAggregateMagnitude(axis),
            SafePointVelocityMagnitudeShift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool CanUseFastAngularResponse(
        Vector3d lever,
        Vector3d vector,
        Fixed3x3 inverseInertia) =>
        IsSafeMagnitude(
            GetAggregateMagnitude(lever)
            | GetAggregateMagnitude(vector)
            | GetAggregateMagnitude(inverseInertia),
            SafeAngularResponseMagnitudeShift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryCross(
        Vector3d left,
        Vector3d right,
        out Vector3d result)
    {
        if (HasSafeProductInputs(left, right))
        {
            result = Vector3d.Cross(left, right);
            return true;
        }

        return Vector3d.TryCross(left, right, out result);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryDot(
        Vector3d left,
        Vector3d right,
        out Fixed64 result)
    {
        if (HasSafeProductInputs(left, right))
        {
            result = Vector3d.Dot(left, right);
            return true;
        }

        return Vector3d.TryDot(left, right, out result);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryTransformDirection(
        Fixed3x3 matrix,
        Vector3d direction,
        out Vector3d result)
    {
        if (HasSafeProductInputs(matrix, direction))
        {
            result = Fixed3x3.TransformDirection(matrix, direction);
            return true;
        }

        return Fixed3x3.TryTransformDirection(
            matrix,
            direction,
            out result);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryLinearCombination(
        Vector3d first,
        Fixed64 firstScale,
        Vector3d second,
        Fixed64 secondScale,
        Vector3d third,
        Fixed64 thirdScale,
        out Vector3d result)
    {
        if (HasSafeProductInputs(
                first,
                firstScale,
                second,
                secondScale,
                third,
                thirdScale))
        {
            result = first * firstScale
                + second * secondScale
                + third * thirdScale;
            return true;
        }

        return Vector3d.TryLinearCombination(
            first,
            firstScale,
            second,
            secondScale,
            third,
            thirdScale,
            out result);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryScale(
        Vector3d value,
        Fixed64 scale,
        out Vector3d result)
    {
        if (HasSafeProductInputs(value, scale))
        {
            result = value * scale;
            return true;
        }

        bool resolved = Fixed64.TryMultiplyDivide(
                value.X,
                scale,
                Fixed64.One,
                out Fixed64 x)
            & Fixed64.TryMultiplyDivide(
                value.Y,
                scale,
                Fixed64.One,
                out Fixed64 y)
            & Fixed64.TryMultiplyDivide(
                value.Z,
                scale,
                Fixed64.One,
                out Fixed64 z);
        result = resolved ? new Vector3d(x, y, z) : default;
        return resolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasSafeProductInputs(
        Vector3d left,
        Vector3d right) =>
        IsSafeMagnitude(
            GetAggregateMagnitude(left)
            | GetAggregateMagnitude(right),
            SafeProductMagnitudeShift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasSafeProductInputs(
        Fixed3x3 matrix,
        Vector3d direction) =>
        IsSafeMagnitude(
            GetAggregateMagnitude(matrix)
            | GetAggregateMagnitude(direction),
            SafeProductMagnitudeShift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasSafeProductInputs(
        Vector3d first,
        Fixed64 firstScale,
        Vector3d second,
        Fixed64 secondScale,
        Vector3d third,
        Fixed64 thirdScale) =>
        IsSafeMagnitude(
            GetAggregateMagnitude(first)
            | GetRawMagnitude(firstScale)
            | GetAggregateMagnitude(second)
            | GetRawMagnitude(secondScale)
            | GetAggregateMagnitude(third)
            | GetRawMagnitude(thirdScale),
            SafeProductMagnitudeShift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasSafeProductInputs(
        Vector3d value,
        Fixed64 scale) =>
        IsSafeMagnitude(
            GetAggregateMagnitude(value)
            | GetRawMagnitude(scale),
            SafeProductMagnitudeShift);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsSafeMagnitude(
        ulong aggregateMagnitude,
        int shift) =>
        (aggregateMagnitude >> shift) == 0UL;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong GetAggregateMagnitude(Vector3d value) =>
        GetRawMagnitude(value.X)
        | GetRawMagnitude(value.Y)
        | GetRawMagnitude(value.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong GetAggregateMagnitude(Fixed3x3 matrix) =>
        GetRawMagnitude(matrix.M11)
        | GetRawMagnitude(matrix.M12)
        | GetRawMagnitude(matrix.M13)
        | GetRawMagnitude(matrix.M21)
        | GetRawMagnitude(matrix.M22)
        | GetRawMagnitude(matrix.M23)
        | GetRawMagnitude(matrix.M31)
        | GetRawMagnitude(matrix.M32)
        | GetRawMagnitude(matrix.M33);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong GetRawMagnitude(Fixed64 value)
    {
        ulong raw = unchecked((ulong)value.m_rawValue);
        ulong sign = unchecked((ulong)(value.m_rawValue >> 63));
        return (raw ^ sign) - sign;
    }
}
