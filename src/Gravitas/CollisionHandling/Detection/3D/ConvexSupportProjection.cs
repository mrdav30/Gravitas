//=======================================================================
// ConvexSupportProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Compares convex support projections without first forming saturating
/// fixed-point products or world-space differences.
/// </summary>
internal static class ConvexSupportProjection
{
    private const ulong PositiveFixed64RawHighLimit = 0x0000_0000_7FFF_FFFFUL;

    public static int Compare(Vector2d candidate, Vector2d current, Vector2d normalizedDirection)
    {
        ulong high = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            candidate.X.m_rawValue,
            current.X.m_rawValue,
            normalizedDirection.X.m_rawValue,
            ref high,
            ref low);
        AccumulateDifferenceProduct(
            candidate.Y.m_rawValue,
            current.Y.m_rawValue,
            normalizedDirection.Y.m_rawValue,
            ref high,
            ref low);
        return CompareToZero(high, low);
    }

    public static int Compare(Vector3d candidate, Vector3d current, Vector3d normalizedDirection)
    {
        ulong high = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            candidate.X.m_rawValue,
            current.X.m_rawValue,
            normalizedDirection.X.m_rawValue,
            ref high,
            ref low);
        AccumulateDifferenceProduct(
            candidate.Y.m_rawValue,
            current.Y.m_rawValue,
            normalizedDirection.Y.m_rawValue,
            ref high,
            ref low);
        AccumulateDifferenceProduct(
            candidate.Z.m_rawValue,
            current.Z.m_rawValue,
            normalizedDirection.Z.m_rawValue,
            ref high,
            ref low);
        return CompareToZero(high, low);
    }

    /// <summary>
    /// Projects <paramref name="target"/> minus <paramref name="source"/> onto
    /// a normalized direction without saturating either the component difference
    /// or its products. Negative results clamp to zero because sweep lower bounds
    /// cannot be negative.
    /// </summary>
    public static Fixed64 ProjectNonNegativeDifference(
        Vector3d target,
        Vector3d source,
        Vector3d normalizedDirection)
    {
        ulong high = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            target.X.m_rawValue,
            source.X.m_rawValue,
            normalizedDirection.X.m_rawValue,
            ref high,
            ref low);
        AccumulateDifferenceProduct(
            target.Y.m_rawValue,
            source.Y.m_rawValue,
            normalizedDirection.Y.m_rawValue,
            ref high,
            ref low);
        AccumulateDifferenceProduct(
            target.Z.m_rawValue,
            source.Z.m_rawValue,
            normalizedDirection.Z.m_rawValue,
            ref high,
            ref low);

        if ((high & 0x8000_0000_0000_0000UL) != 0UL || (high == 0UL && low == 0UL))
            return Fixed64.Zero;

        // The accumulator is Q64.64. A right shift by 32 produces the
        // conservative Q32.32 lower bound; clamp only if that shifted value
        // exceeds the positive Fixed64 range.
        if (high > PositiveFixed64RawHighLimit)
            return Fixed64.MaxValue;

        long raw = unchecked((long)((high << 32) | (low >> 32)));
        return Fixed64.FromRaw(raw);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareToZero(ulong high, ulong low)
    {
        if (high == 0UL)
            return low == 0UL ? 0 : 1;

        return (high & 0x8000_0000_0000_0000UL) != 0UL ? -1 : 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AccumulateDifferenceProduct(
        long candidate,
        long current,
        long direction,
        ref ulong sumHigh,
        ref ulong sumLow)
    {
        if (candidate == current || direction == 0L)
            return;

        bool negativeDifference = candidate < current;
        ulong difference = negativeDifference
            ? unchecked((ulong)current - (ulong)candidate)
            : unchecked((ulong)candidate - (ulong)current);
        bool negativeDirection = direction < 0L;
        ulong directionMagnitude = negativeDirection
            ? unchecked((ulong)(~direction) + 1UL)
            : (ulong)direction;

        Multiply64To128(difference, directionMagnitude, out ulong productHigh, out ulong productLow);
        if (negativeDifference != negativeDirection)
        {
            productLow = unchecked(~productLow + 1UL);
            productHigh = unchecked(~productHigh + (productLow == 0UL ? 1UL : 0UL));
        }

        ulong previousLow = sumLow;
        sumLow = unchecked(sumLow + productLow);
        sumHigh = unchecked(sumHigh + productHigh + (sumLow < previousLow ? 1UL : 0UL));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Multiply64To128(ulong left, ulong right, out ulong high, out ulong low)
    {
        ulong leftLow = (uint)left;
        ulong leftHigh = left >> 32;
        ulong rightLow = (uint)right;
        ulong rightHigh = right >> 32;

        ulong product0 = leftLow * rightLow;
        ulong product1 = leftLow * rightHigh;
        ulong product2 = leftHigh * rightLow;
        ulong product3 = leftHigh * rightHigh;
        ulong middle = (product0 >> 32) + (uint)product1 + (uint)product2;

        low = (product0 & 0xFFFF_FFFFUL) | (middle << 32);
        high = product3 + (product1 >> 32) + (product2 >> 32) + (middle >> 32);
    }
}
