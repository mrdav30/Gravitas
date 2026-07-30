//=======================================================================
// ContactResponseArithmetic3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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

    internal static bool TryGetRelativePointVelocity(
        Vector3d linearA,
        Vector3d angularA,
        Vector3d leverA,
        Vector3d linearB,
        Vector3d angularB,
        Vector3d leverB,
        Vector3d axis,
        out Vector3d relativeVelocity)
    {
        if (CanUseFastPointVelocity(
                linearA,
                angularA,
                leverA,
                linearB,
                angularB,
                leverB,
                axis))
        {
            Vector3d fastAngularVelocityA =
                Vector3d.Cross(angularA, leverA);
            Vector3d fastAngularVelocityB =
                Vector3d.Cross(angularB, leverB);
            if (!PreservesNonzeroCrossProduct(
                    angularA,
                    leverA,
                    fastAngularVelocityA)
                || !PreservesNonzeroCrossProduct(
                    angularB,
                    leverB,
                    fastAngularVelocityB))
            {
                relativeVelocity = default;
                return false;
            }

            Vector3d fastPointVelocityA =
                linearA + fastAngularVelocityA;
            Vector3d fastPointVelocityB =
                linearB + fastAngularVelocityB;
            relativeVelocity =
                fastPointVelocityB - fastPointVelocityA;
            return true;
        }

        bool firstCrossResolved = TryCross(
                angularA,
                leverA,
                out Vector3d angularVelocityA);
        bool secondCrossResolved = TryCross(
                angularB,
                leverB,
                out Vector3d angularVelocityB);
        bool pointVelocitiesResolved = firstCrossResolved
            & secondCrossResolved
            & Vector3d.TryAdd(
                linearA,
                angularVelocityA,
                out Vector3d pointVelocityA)
            & Vector3d.TryAdd(
                linearB,
                angularVelocityB,
                out Vector3d pointVelocityB);
        if (!pointVelocitiesResolved)
        {
            relativeVelocity = default;
            return false;
        }

        return Vector3d.TrySubtract(
            pointVelocityB,
            pointVelocityA,
            out relativeVelocity);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryCross(
        Vector3d left,
        Vector3d right,
        out Vector3d result)
    {
        if (HasSafeProductInputs(left, right))
        {
            result = Vector3d.Cross(left, right);
            return PreservesNonzeroCrossProduct(
                left,
                right,
                result);
        }

        return Vector3d.TryCross(left, right, out result)
            && PreservesNonzeroCrossProduct(left, right, result);
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
            return PreservesNonzeroDotProduct(
                left,
                right,
                result);
        }

        return Vector3d.TryDot(left, right, out result)
            && PreservesNonzeroDotProduct(left, right, result);
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
            return PreservesNonzeroTransformDirection(
                matrix,
                direction,
                result);
        }

        return Fixed3x3.TryTransformDirection(
            matrix,
            direction,
            out result)
            && PreservesNonzeroTransformDirection(matrix, direction, result);
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
            return PreservesNonzeroLinearCombination(
                first,
                firstScale,
                second,
                secondScale,
                third,
                thirdScale,
                result);
        }

        return Vector3d.TryLinearCombination(
            first,
            firstScale,
            second,
            secondScale,
            third,
            thirdScale,
            out result)
            && PreservesNonzeroLinearCombination(
                first,
                firstScale,
                second,
                secondScale,
                third,
                thirdScale,
                result);
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

    internal static bool PreservesNonzeroCrossProduct(
        Vector3d left,
        Vector3d right,
        Vector3d result)
    {
        bool inspectX = result.X == Fixed64.Zero
            && HasNonzeroDifferenceTerm(
                left.Y,
                right.Z,
                left.Z,
                right.Y);
        bool inspectY = result.Y == Fixed64.Zero
            && HasNonzeroDifferenceTerm(
                left.Z,
                right.X,
                left.X,
                right.Z);
        bool inspectZ = result.Z == Fixed64.Zero
            && HasNonzeroDifferenceTerm(
                left.X,
                right.Y,
                left.Y,
                right.X);
        if (!(inspectX | inspectY | inspectZ))
            return true;

        WideGeometry.GetDifferenceCrossProduct3D(
            left.X, Fixed64.Zero,
            left.Y, Fixed64.Zero,
            left.Z, Fixed64.Zero,
            right.X, Fixed64.Zero,
            right.Y, Fixed64.Zero,
            right.Z, Fixed64.Zero,
            out Signed192 exactX,
            out Signed192 exactY,
            out Signed192 exactZ);
        return PreservesExactValue(inspectX, exactX)
            & PreservesExactValue(inspectY, exactY)
            & PreservesExactValue(inspectZ, exactZ);
    }

    internal static bool PreservesNonzeroDotProduct(
        Vector3d left,
        Vector3d right,
        Fixed64 result)
    {
        if (result != Fixed64.Zero
            || !HasNonzeroTerm(
                left.X,
                right.X,
                left.Y,
                right.Y,
                left.Z,
                right.Z))
        {
            return true;
        }

        return GetLinearCombinationComponent(
            left.X,
            right.X,
            left.Y,
            right.Y,
            left.Z,
            right.Z).IsZero;
    }

    internal static bool PreservesNonzeroTransformDirection(
        Fixed3x3 matrix,
        Vector3d direction,
        Vector3d result)
    {
        bool inspectX = result.X == Fixed64.Zero
            && HasNonzeroTerm(
                direction.X,
                matrix.M11,
                direction.Y,
                matrix.M21,
                direction.Z,
                matrix.M31);
        bool inspectY = result.Y == Fixed64.Zero
            && HasNonzeroTerm(
                direction.X,
                matrix.M12,
                direction.Y,
                matrix.M22,
                direction.Z,
                matrix.M32);
        bool inspectZ = result.Z == Fixed64.Zero
            && HasNonzeroTerm(
                direction.X,
                matrix.M13,
                direction.Y,
                matrix.M23,
                direction.Z,
                matrix.M33);
        if (!(inspectX | inspectY | inspectZ))
            return true;

        Signed192 exactX = GetLinearCombinationComponent(
            direction.X,
            matrix.M11,
            direction.Y,
            matrix.M21,
            direction.Z,
            matrix.M31);
        Signed192 exactY = GetLinearCombinationComponent(
            direction.X,
            matrix.M12,
            direction.Y,
            matrix.M22,
            direction.Z,
            matrix.M32);
        Signed192 exactZ = GetLinearCombinationComponent(
            direction.X,
            matrix.M13,
            direction.Y,
            matrix.M23,
            direction.Z,
            matrix.M33);
        return PreservesExactValue(inspectX, exactX)
            & PreservesExactValue(inspectY, exactY)
            & PreservesExactValue(inspectZ, exactZ);
    }

    private static bool PreservesNonzeroLinearCombination(
        Vector3d first,
        Fixed64 firstScale,
        Vector3d second,
        Fixed64 secondScale,
        Vector3d third,
        Fixed64 thirdScale,
        Vector3d result)
    {
        bool inspectX = result.X == Fixed64.Zero
            && HasNonzeroTerm(
                first.X,
                firstScale,
                second.X,
                secondScale,
                third.X,
                thirdScale);
        bool inspectY = result.Y == Fixed64.Zero
            && HasNonzeroTerm(
                first.Y,
                firstScale,
                second.Y,
                secondScale,
                third.Y,
                thirdScale);
        bool inspectZ = result.Z == Fixed64.Zero
            && HasNonzeroTerm(
                first.Z,
                firstScale,
                second.Z,
                secondScale,
                third.Z,
                thirdScale);
        if (!(inspectX | inspectY | inspectZ))
            return true;

        Signed192 exactX = GetLinearCombinationComponent(
            first.X,
            firstScale,
            second.X,
            secondScale,
            third.X,
            thirdScale);
        Signed192 exactY = GetLinearCombinationComponent(
            first.Y,
            firstScale,
            second.Y,
            secondScale,
            third.Y,
            thirdScale);
        Signed192 exactZ = GetLinearCombinationComponent(
            first.Z,
            firstScale,
            second.Z,
            secondScale,
            third.Z,
            thirdScale);
        return PreservesExactValue(inspectX, exactX)
            & PreservesExactValue(inspectY, exactY)
            & PreservesExactValue(inspectZ, exactZ);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PreservesExactValue(
        bool inspect,
        Signed192 exact) =>
        !inspect || exact.IsZero;

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
    private static bool HasNonzeroTerm(
        Fixed64 first,
        Fixed64 firstScale,
        Fixed64 second,
        Fixed64 secondScale,
        Fixed64 third,
        Fixed64 thirdScale) =>
        (first != Fixed64.Zero && firstScale != Fixed64.Zero)
        || (second != Fixed64.Zero && secondScale != Fixed64.Zero)
        || (third != Fixed64.Zero && thirdScale != Fixed64.Zero);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasNonzeroDifferenceTerm(
        Fixed64 first,
        Fixed64 firstScale,
        Fixed64 second,
        Fixed64 secondScale) =>
        (first != Fixed64.Zero && firstScale != Fixed64.Zero)
        || (second != Fixed64.Zero && secondScale != Fixed64.Zero);

    private static Signed192 GetLinearCombinationComponent(
        Fixed64 first,
        Fixed64 firstScale,
        Fixed64 second,
        Fixed64 secondScale,
        Fixed64 third,
        Fixed64 thirdScale) =>
        WideGeometry.GetDifferenceDotProduct3D(
            first, Fixed64.Zero,
            second, Fixed64.Zero,
            third, Fixed64.Zero,
            firstScale, Fixed64.Zero,
            secondScale, Fixed64.Zero,
            thirdScale, Fixed64.Zero);

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
