//=======================================================================
// ExactContactResponseKernel.CoulombDiskCache.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <content>
/// Owns exact cached-disk projection comparison and materialization.
/// </content>
internal static partial class ExactContactResponseKernel
{
    // With anchor denominators <202 bits, an angular effective-mass denominator
    // is <532 bits; effective mass is <1432/<1130 bits and point-velocity D <468.
    // A normal impulse is therefore <2088/<1932 bits even with maximum scales.
    // Int32 iterations add at most 31 carry bits. Multiplying by maximum friction
    // bounds cached tangent raw magnitudes below 2150 bits. A desired tangent
    // at the common denominator is <6014 bits, within the existing 96 words.
    // Disk squares and radial comparisons remain below the existing 320 words.
    private static void AddCachedTangentToRatio(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator, int sign, Fixed64 scalar,
        ExactContactImpulseState? state, bool primary, Span<ulong> result, out int resultSign)
    {
        if (state?.HasTangents != true)
        {
            AddFixedToRatio(numerator, denominator, sign, scalar, result, out resultSign);
            return;
        }
        Span<ulong> cached = stackalloc ulong[MaxCoulombWords];
        WideArithmetic.MultiplyMagnitudes(primary ? state.PrimaryMagnitude : state.SecondaryMagnitude,
            denominator, cached);
        AddSignedMagnitudes(numerator, sign, cached,
            primary ? state.PrimarySign : state.SecondarySign, result, out resultSign);
    }

    private static void SubtractCachedTangentFromRatio(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator, int sign, Fixed64 scalar,
        ExactContactImpulseState? state, bool primary, Span<ulong> result,
        Span<ulong> resultDenominator, out int resultSign)
    {
        if (state?.HasTangents != true)
        {
            SubtractFixedFromRatio(numerator, denominator, sign, scalar,
                result, resultDenominator, out resultSign);
            return;
        }
        Span<ulong> cached = stackalloc ulong[MaxCoulombWords];
        WideArithmetic.MultiplyMagnitudes(primary ? state.PrimaryMagnitude : state.SecondaryMagnitude,
            denominator, cached);
        AddSignedMagnitudes(numerator, sign, cached,
            -(primary ? state.PrimarySign : state.SecondarySign), result, out resultSign);
        resultDenominator.Clear();
        denominator.CopyTo(resultDenominator);
    }

    private static void RoundCachedRatio(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator, Span<ulong> result)
    {
        Span<ulong> quotient = stackalloc ulong[numerator.Length];
        Span<ulong> remainder = stackalloc ulong[denominator.Length];
        Span<ulong> division = stackalloc ulong[numerator.Length + denominator.Length + 1];
        WideArithmetic.DivideMagnitudes(numerator, denominator, quotient, remainder, division);
        // Compare r with D-r to decide the midpoint without overflowing 2*r.
        Span<ulong> distance = stackalloc ulong[denominator.Length];
        WideArithmetic.SubtractEqualMagnitudes(denominator, remainder, distance);
        int midpoint = WideArithmetic.CompareMagnitudeEqualLength(remainder, distance);
        if (midpoint > 0 || midpoint == 0 && (quotient[0] & 1UL) != 0)
        {
            Span<ulong> one = stackalloc ulong[1];
            one[0] = 1;
            WideArithmetic.AddMagnitudeInto(one, quotient);
        }
#if DEBUG
        // The retained tangent bound is <2150 bits, below the 3072-bit cache.
        System.Diagnostics.Debug.Assert(WideArithmetic.IsZeroMagnitude(quotient[result.Length..]));
#endif
        quotient[..result.Length].CopyTo(result);
    }

    private static void RoundCachedRadialRatio(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator, ReadOnlySpan<ulong> radicand, Span<ulong> result)
    {
        result.Clear();
        if (WideArithmetic.IsZeroMagnitude(numerator)) return;
        // The bit enclosure only bounds the integer search. Every admitted bit
        // and the final midpoint are compared against the exact radical.
        int bits = Math.Max(0, GetRadialQuotientBitBound(numerator, denominator, radicand));
#if DEBUG
        // The enclosure adds at most two bits to the <2150-bit tangent bound.
        System.Diagnostics.Debug.Assert(bits <= result.Length * 64);
#endif
        for (int bit = bits - 1; bit >= 0; bit--)
        {
            ulong mask = 1UL << (bit & 63);
            result[bit >> 6] |= mask;
            if (CompareRatioOverSquareRoot(numerator, denominator, radicand, result,
                    numeratorMultiplier: 1) < 0) result[bit >> 6] &= ~mask;
        }
        Span<ulong> midpoint = stackalloc ulong[result.Length];
        Span<ulong> two = stackalloc ulong[1];
        two[0] = 2;
        WideArithmetic.MultiplyMagnitudes(result, two, midpoint);
        midpoint[0] |= 1UL;
        int comparison = CompareRatioOverSquareRoot(numerator, denominator, radicand, midpoint,
            numeratorMultiplier: 2);
        if (comparison > 0 || comparison == 0 && (result[0] & 1UL) != 0)
        {
            two[0] = 1;
            WideArithmetic.AddMagnitudeInto(two, result);
        }
    }

    private static bool IsRadialProjectionEqualToFixed(
        ReadOnlySpan<ulong> directionNumerator,
        int directionSign,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        Fixed64 value,
        ExactContactImpulseState? state = null,
        bool primary = true)
    {
        Span<ulong> projectedNumerator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> valueMagnitude = stackalloc ulong[state?.HasTangents == true ? MaxResponseWords : 3];
        WideArithmetic.MultiplyMagnitudes(
            directionNumerator,
            radialNumerator,
            projectedNumerator);
        if (state?.HasTangents == true)
            (primary ? state.PrimaryMagnitude : state.SecondaryMagnitude).CopyTo(valueMagnitude);
        else
            SetMagnitude(value, valueMagnitude);
        int valueSign =
            state?.HasTangents == true ? primary ? state.PrimarySign : state.SecondarySign
                : value == Fixed64.Zero ? 0 : value < Fixed64.Zero ? -1 : 1;
        Span<ulong> valueCoefficient = stackalloc ulong[MaxCoulombComparisonWords];
        WideArithmetic.MultiplyMagnitudes(valueMagnitude, radialDenominator, valueCoefficient);
        return WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                projectedNumerator,
                directionSign,
                valueCoefficient,
                -valueSign,
                radicand)
            == 0;
    }

    // The radical denominator is the supplied factor times the rational denominator.
    // Both factors and the radicand are positive.
    internal static bool TryGetSignedRadicalAndRationalSum(
        ReadOnlySpan<ulong> radicalNumerator,
        ReadOnlySpan<ulong> radicalDenominatorFactor,
        ReadOnlySpan<ulong> radicand,
        int radicalSign,
        ReadOnlySpan<ulong> rationalNumerator,
        ReadOnlySpan<ulong> rationalDenominator,
        int rationalSign,
        out Fixed64 result)
    {
        radicalNumerator = TrimMagnitude(radicalNumerator);
        radicalDenominatorFactor = TrimMagnitude(radicalDenominatorFactor);
        rationalNumerator = TrimMagnitude(rationalNumerator);
        rationalDenominator = TrimMagnitude(rationalDenominator);
        radicand = TrimMagnitude(radicand);

        // A/(B*D*sqrt(R)) + N/D compared to c/2 has the sign of
        // 2*A + (2*N*B - c*B*D)*sqrt(R). Prepare the invariant products once.
        ReadOnlySpan<ulong> two = stackalloc ulong[] { 2UL };
        Span<ulong> doubledRadical = stackalloc ulong[radicalNumerator.Length + 1];
        Span<ulong> doubledCoefficient =
            stackalloc ulong[rationalNumerator.Length + radicalDenominatorFactor.Length + 1];
        Span<ulong> denominator =
            stackalloc ulong[radicalDenominatorFactor.Length + rationalDenominator.Length];
        WideArithmetic.MultiplyMagnitudes(radicalNumerator, two, doubledRadical);
        Multiply3(rationalNumerator, radicalDenominatorFactor, two, doubledCoefficient);
        WideArithmetic.MultiplyMagnitudes(radicalDenominatorFactor, rationalDenominator, denominator);
        int resultSign = WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
            doubledRadical, radicalSign, doubledCoefficient, rationalSign, radicand);
        if (resultSign == 0)
        {
            result = Fixed64.Zero;
            return true;
        }

        int normalizedRadicalSign =
            resultSign < 0 ? -radicalSign : radicalSign;
        int normalizedRationalSign =
            resultSign < 0 ? -rationalSign : rationalSign;
        ulong limit =
            resultSign < 0 ? 1UL << 63 : (ulong)long.MaxValue;
        int limitMidpointComparison =
            ComparePreparedSignedSum(doubledRadical, normalizedRadicalSign, doubledCoefficient,
                normalizedRationalSign, denominator, radicand, limit, midpoint: true);
        if (limitMidpointComparison > 0
            || (limitMidpointComparison == 0
                && (limit & 1UL) != 0UL))
        {
            result = default;
            return false;
        }

        ulong low = 0UL, high = limit;
        if (TryGetSignedSumIntegerParts(radicalNumerator, TrimMagnitude(denominator), radicand,
                normalizedRadicalSign, rationalNumerator, rationalDenominator, normalizedRationalSign,
                out long integerParts))
        {
            // The normalized sum is positive. With opposite signs the fractional
            // correction is in (-1, 1), and with positive terms it is in [0, 2).
            low = (ulong)Math.Max(0L, integerParts - 1L);
            high = (ulong)(integerParts + 1L);
        }
        while (low < high)
        {
            ulong difference = high - low;
            ulong middle =
                low + (difference >> 1) + (difference & 1UL);
            if (ComparePreparedSignedSum(doubledRadical, normalizedRadicalSign, doubledCoefficient,
                    normalizedRationalSign, denominator, radicand, middle, midpoint: false) >= 0)
            {
                low = middle;
            }
            else
            {
                high = middle - 1UL;
            }
        }

        ulong quotient = low;
        int midpointComparison =
            ComparePreparedSignedSum(doubledRadical, normalizedRadicalSign, doubledCoefficient,
                normalizedRationalSign, denominator, radicand, quotient, midpoint: true);
        bool roundUp = midpointComparison > 0
            | (midpointComparison == 0 & (quotient & 1UL) != 0UL);
        if (roundUp & quotient < limit)
            quotient++;

        result = new Fixed64(
            resultSign < 0
                ? unchecked(-(long)quotient)
                : (long)quotient);
        return true;
    }

    private static bool TryGetSignedSumIntegerParts(
        ReadOnlySpan<ulong> radicalNumerator,
        ReadOnlySpan<ulong> radicalDenominator,
        ReadOnlySpan<ulong> radicand,
        int radicalSign,
        ReadOnlySpan<ulong> rationalNumerator,
        ReadOnlySpan<ulong> rationalDenominator,
        int rationalSign,
        out long integerParts)
    {
        // Two unclipped parts and a one-unit search margin fit in Int64.
        const ulong cap = 1UL << 62;
        Span<ulong> rationalFloor = stackalloc ulong[Math.Max(1, rationalNumerator.Length)];
        Span<ulong> remainder = stackalloc ulong[rationalDenominator.Length];
        Span<ulong> division = stackalloc ulong[rationalNumerator.Length + rationalDenominator.Length + 1];
        WideArithmetic.DivideMagnitudes(rationalNumerator, rationalDenominator, rationalFloor, remainder, division);
        integerParts = 0;
        if (rationalFloor[0] >= cap || !WideArithmetic.IsZeroMagnitude(rationalFloor[1..]))
            return false;

        int words = Math.Max(radicalNumerator.Length * 2, radicalDenominator.Length * 2 + radicand.Length);
        Span<ulong> squareNumerator = stackalloc ulong[words];
        Span<ulong> squareDenominator = stackalloc ulong[words];
        WideArithmetic.MultiplyMagnitudes(radicalNumerator, radicalNumerator, squareNumerator);
        Multiply3(radicalDenominator, radicalDenominator, radicand, squareDenominator);
        ulong radicalFloor = WideArithmetic.GetRatioFloorSquareRoot(squareNumerator, squareDenominator, cap);
        if (radicalFloor >= cap)
            return false;

        integerParts = radicalSign * (long)radicalFloor + rationalSign * (long)rationalFloor[0];
        return true;
    }

    private static int GetRadialQuotientBitBound(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand) =>
        WideArithmetic.GetMagnitudeBitLength(numerator)
        - WideArithmetic.GetMagnitudeBitLength(denominator)
        - (WideArithmetic.GetMagnitudeBitLength(radicand) - 1) / 2 + 2;

    private static int ComparePreparedSignedSum(
        ReadOnlySpan<ulong> doubledRadical,
        int radicalSign,
        ReadOnlySpan<ulong> doubledCoefficient,
        int rationalSign,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        ulong candidate,
        bool midpoint)
    {
        ReadOnlySpan<ulong> doubledCandidate =
            stackalloc ulong[] { (candidate << 1) | (midpoint ? 1UL : 0UL), candidate >> 63 };
        Span<ulong> threshold = stackalloc ulong[denominator.Length + doubledCandidate.Length];
        Span<ulong> adjusted = stackalloc ulong[Math.Max(doubledCoefficient.Length, threshold.Length) + 1];
        WideArithmetic.MultiplyMagnitudes(doubledCandidate, denominator, threshold);
        AddSignedMagnitudes(doubledCoefficient, rationalSign, threshold, -1, adjusted, out int adjustedSign);
        return WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
            doubledRadical, radicalSign, adjusted, adjustedSign, radicand);
    }

    private static void GetWeightedFixedSum(
        Fixed64 firstFactor,
        Fixed64 firstValue,
        Fixed64 secondFactor,
        Fixed64 secondValue,
        Span<ulong> result,
        out int resultSign,
        ExactContactImpulseState? state = null)
    {
        if (state?.HasTangents == true)
        {
            GetWeightedSignedSum(firstFactor, state.PrimaryMagnitude, state.PrimarySign,
                secondFactor, state.SecondaryMagnitude, state.SecondarySign, result, out resultSign);
            return;
        }
        Span<ulong> factorMagnitude = stackalloc ulong[3];
        Span<ulong> valueMagnitude = stackalloc ulong[3];
        Span<ulong> first = stackalloc ulong[3];
        Span<ulong> second = stackalloc ulong[3];
        SetMagnitude(firstFactor, factorMagnitude);
        SetMagnitude(firstValue, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            valueMagnitude,
            first);
        SetMagnitude(secondFactor, factorMagnitude);
        SetMagnitude(secondValue, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            valueMagnitude,
            second);
        AddSignedMagnitudes(
            first,
            GetProductSign(firstFactor, firstValue),
            second,
            GetProductSign(secondFactor, secondValue),
            result,
            out resultSign);
    }

    private static void GetWeightedSignedWideSum(
        Signed832 firstFactor,
        Fixed64 firstValue,
        Signed832 secondFactor,
        Fixed64 secondValue,
        Span<ulong> result,
        out int resultSign,
        ExactContactImpulseState? state = null)
    {
        if (state?.HasTangents == true)
        {
            Span<ulong> firstMagnitude = stackalloc ulong[13];
            Span<ulong> secondMagnitude = stackalloc ulong[13];
            Span<ulong> firstProduct = stackalloc ulong[64];
            Span<ulong> secondProduct = stackalloc ulong[64];
            SetMagnitude(firstFactor, firstMagnitude);
            SetMagnitude(secondFactor, secondMagnitude);
            WideArithmetic.MultiplyMagnitudes(firstMagnitude, state.PrimaryMagnitude, firstProduct);
            WideArithmetic.MultiplyMagnitudes(secondMagnitude, state.SecondaryMagnitude, secondProduct);
            AddSignedMagnitudes(firstProduct, firstFactor.Sign * state.PrimarySign,
                secondProduct, secondFactor.Sign * state.SecondarySign, result, out resultSign);
            return;
        }
        Span<ulong> factorMagnitude = stackalloc ulong[13];
        Span<ulong> valueMagnitude = stackalloc ulong[3];
        Span<ulong> first = stackalloc ulong[15];
        Span<ulong> second = stackalloc ulong[15];
        SetMagnitude(firstFactor, factorMagnitude);
        SetMagnitude(firstValue, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            valueMagnitude,
            first);
        SetMagnitude(secondFactor, factorMagnitude);
        SetMagnitude(secondValue, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            valueMagnitude,
            second);
        AddSignedMagnitudes(
            first,
            firstFactor.Sign * Math.Sign(firstValue.m_rawValue),
            second,
            secondFactor.Sign * Math.Sign(secondValue.m_rawValue),
            result,
            out resultSign);
    }

    private static int GetProductSign(Fixed64 first, Fixed64 second) =>
        Math.Sign(first.m_rawValue) * Math.Sign(second.m_rawValue);

    private static ReadOnlySpan<ulong> TrimMagnitude(ReadOnlySpan<ulong> value) =>
        value[..WideArithmetic.GetActiveMagnitudeLength(value)];

    private static bool TryGetDiskVelocityDeltas(
        in ExactContactResponseOperand3D primaryFirst,
        in ExactContactResponseOperand3D primarySecond,
        in ExactContactResponseOperand3D secondaryFirst,
        in ExactContactResponseOperand3D secondarySecond,
        Vector3d primaryTangent,
        Vector3d secondaryTangent,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        Fixed64 accumulatedPrimaryTangentImpulse,
        Fixed64 accumulatedSecondaryTangentImpulse,
        out Vector3d firstLinear,
        out Vector3d firstAngular,
        out Vector3d secondLinear,
        out Vector3d secondAngular,
        ExactContactImpulseState? state = null)
    {
        bool resolved = TryGetDiskLinearVelocityDelta(
            primaryFirst.LinearImpulseAxis,
            secondaryFirst.LinearImpulseAxis,
            primaryFirst.InverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out firstLinear, state);
        resolved &= TryGetDiskAngularVelocityDelta(
            primaryFirst.Lever,
            -primaryTangent,
            -secondaryTangent,
            primaryFirst.InverseInertia,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out firstAngular, state);
        resolved &= TryGetDiskLinearVelocityDelta(
            primarySecond.LinearImpulseAxis,
            secondarySecond.LinearImpulseAxis,
            primarySecond.InverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out secondLinear, state);
        resolved &= TryGetDiskAngularVelocityDelta(
            primarySecond.Lever,
            primaryTangent,
            secondaryTangent,
            primarySecond.InverseInertia,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out secondAngular, state);
        return resolved;
    }

    private static bool TryGetDiskLinearVelocityDelta(
        Vector3d primaryAxis,
        Vector3d secondaryAxis,
        Fixed64 inverseMass,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        Fixed64 accumulatedPrimaryTangentImpulse,
        Fixed64 accumulatedSecondaryTangentImpulse,
        out Vector3d result,
        ExactContactImpulseState? state = null)
    {
        bool xResolved = TryGetDiskLinearComponent(
            primaryAxis.X,
            secondaryAxis.X,
            inverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out Fixed64 x, state);
        bool yResolved = TryGetDiskLinearComponent(
            primaryAxis.Y,
            secondaryAxis.Y,
            inverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out Fixed64 y, state);
        bool zResolved = TryGetDiskLinearComponent(
            primaryAxis.Z,
            secondaryAxis.Z,
            inverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out Fixed64 z, state);
        result = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
    }

    private static bool TryGetDiskLinearComponent(
        Fixed64 primaryAxis,
        Fixed64 secondaryAxis,
        Fixed64 inverseMass,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        Fixed64 accumulatedPrimaryTangentImpulse,
        Fixed64 accumulatedSecondaryTangentImpulse,
        out Fixed64 result,
        ExactContactImpulseState? state = null)
    {
        Span<ulong> combined = stackalloc ulong[MaxCoulombWords + 2];
        GetWeightedSignedSum(
            primaryAxis,
            primaryNumerator,
            primarySign,
            secondaryAxis,
            secondaryNumerator,
            secondarySign,
            combined,
            out int combinedSign);
        if (inverseMass == Fixed64.Zero)
        {
            result = Fixed64.Zero;
            return true;
        }

        Span<ulong> inverseMassMagnitude =
            stackalloc ulong[3];
        Span<ulong> fixedScale = stackalloc ulong[1];
        Span<ulong> numerator = stackalloc ulong[MaxCoulombComparisonWords / 2];
        SetMagnitude(inverseMass, inverseMassMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        if (rational)
        {
            if (combinedSign == 0)
            {
                result = Fixed64.Zero;
                return true;
            }

            Span<ulong> denominator = stackalloc ulong[MaxCoulombComparisonWords / 2];
            Multiply3(commonDenominator, fixedScale, fixedScale, denominator);
            WideArithmetic.MultiplyMagnitudes(
                combined,
                inverseMassMagnitude,
                numerator);
            return Fixed64.TryGetSignedRawRatio(
                numerator,
                denominator,
                combinedSign < 0,
                out result);
        }

        Span<ulong> scaledNumerator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> cachedCombined = stackalloc ulong[state?.HasTangents == true ? 64 : 3];
        Span<ulong> cachedNumerator = stackalloc ulong[state?.HasTangents == true ? 65 : 4];
        Span<ulong> cachedDenominator = stackalloc ulong[2];
        WideArithmetic.MultiplyMagnitudes(
            combined,
            inverseMassMagnitude,
            numerator);
        WideArithmetic.MultiplyMagnitudes(
            numerator,
            radialNumerator,
            scaledNumerator);
        GetWeightedFixedSum(
            primaryAxis,
            accumulatedPrimaryTangentImpulse,
            secondaryAxis,
            accumulatedSecondaryTangentImpulse,
            cachedCombined,
            out int cachedSign, state);
        WideArithmetic.MultiplyMagnitudes(
            cachedCombined,
            inverseMassMagnitude,
            cachedNumerator);
        WideArithmetic.MultiplyMagnitudes(
            fixedScale,
            fixedScale,
            cachedDenominator);
        return TryGetSignedRadicalAndRationalSum(
            scaledNumerator,
            radialDenominator,
            radicand,
            combinedSign,
            cachedNumerator,
            cachedDenominator,
            -cachedSign,
            out result);
    }

    private static bool TryGetDiskAngularVelocityDelta(
        in ExactLever3D lever,
        Vector3d primaryAxis,
        Vector3d secondaryAxis,
        Fixed3x3 inverseInertia,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        Fixed64 accumulatedPrimaryTangentImpulse,
        Fixed64 accumulatedSecondaryTangentImpulse,
        out Vector3d result,
        ExactContactImpulseState? state = null)
    {
        ExactLever3D.GetTransformedCrossProduct(
            lever,
            primaryAxis,
            inverseInertia,
            out Signed832 primaryX,
            out Signed832 primaryY,
            out Signed832 primaryZ);
        ExactLever3D.GetTransformedCrossProduct(
            lever,
            secondaryAxis,
            inverseInertia,
            out Signed832 secondaryX,
            out Signed832 secondaryY,
            out Signed832 secondaryZ);
        Signed320 fixedScaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        Signed704 transformedDenominator =
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.Denominator,
                fixedScaleSquared);

        bool xResolved = TryGetDiskAngularComponent(
            primaryX,
            secondaryX,
            transformedDenominator,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out Fixed64 x, state);
        bool yResolved = TryGetDiskAngularComponent(
            primaryY,
            secondaryY,
            transformedDenominator,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out Fixed64 y, state);
        bool zResolved = TryGetDiskAngularComponent(
            primaryZ,
            secondaryZ,
            transformedDenominator,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            accumulatedPrimaryTangentImpulse,
            accumulatedSecondaryTangentImpulse,
            out Fixed64 z, state);
        result = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
    }

    private static bool TryGetDiskAngularComponent(
        Signed832 primary,
        Signed832 secondary,
        Signed704 transformedDenominator,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        Fixed64 accumulatedPrimaryTangentImpulse,
        Fixed64 accumulatedSecondaryTangentImpulse,
        out Fixed64 result,
        ExactContactImpulseState? state = null)
    {
        Span<ulong> primaryMagnitude = stackalloc ulong[13];
        Span<ulong> secondaryMagnitude = stackalloc ulong[13];
        SetMagnitude(primary, primaryMagnitude);
        SetMagnitude(secondary, secondaryMagnitude);
        Span<ulong> primaryProduct =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> secondaryProduct =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        WideArithmetic.MultiplyMagnitudes(
            primaryMagnitude,
            primaryNumerator,
            primaryProduct);
        WideArithmetic.MultiplyMagnitudes(
            secondaryMagnitude,
            secondaryNumerator,
            secondaryProduct);
        Span<ulong> combined =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        AddSignedMagnitudes(
            primaryProduct,
            primary.Sign * primarySign,
            secondaryProduct,
            secondary.Sign * secondarySign,
            combined,
            out int combinedSign);

        Span<ulong> transformedDenominatorMagnitude =
            stackalloc ulong[11];
        Span<ulong> fixedScale = stackalloc ulong[1];
        SetMagnitude(transformedDenominator, transformedDenominatorMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        if (rational)
        {
            if (combinedSign == 0)
            {
                result = Fixed64.Zero;
                return true;
            }

            Span<ulong> denominator = stackalloc ulong[MaxCoulombComparisonWords / 2];
            Multiply3(transformedDenominatorMagnitude, commonDenominator, fixedScale, denominator);
            return Fixed64.TryGetSignedRawRatio(
                combined,
                denominator,
                combinedSign < 0,
                out result);
        }

        Span<ulong> scaledNumerator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> cachedCombined = stackalloc ulong[state?.HasTangents == true ? 64 : 15];
        Span<ulong> cachedDenominator = stackalloc ulong[12];
        WideArithmetic.MultiplyMagnitudes(
            combined,
            radialNumerator,
            scaledNumerator);
        GetWeightedSignedWideSum(
            primary,
            accumulatedPrimaryTangentImpulse,
            secondary,
            accumulatedSecondaryTangentImpulse,
            cachedCombined,
            out int cachedSign, state);
        WideArithmetic.MultiplyMagnitudes(
            transformedDenominatorMagnitude,
            fixedScale,
            cachedDenominator);
        return TryGetSignedRadicalAndRationalSum(
            scaledNumerator,
            radialDenominator,
            radicand,
            combinedSign,
            cachedCombined,
            cachedDenominator,
            -cachedSign,
            out result);
    }

    private static void GetWeightedSignedSum(
        Fixed64 firstFactor,
        ReadOnlySpan<ulong> firstMagnitude,
        int firstSign,
        Fixed64 secondFactor,
        ReadOnlySpan<ulong> secondMagnitude,
        int secondSign,
        Span<ulong> result,
        out int resultSign)
    {
        Span<ulong> factorMagnitude = stackalloc ulong[3];
        Span<ulong> first = stackalloc ulong[MaxCoulombWords + 2];
        Span<ulong> second = stackalloc ulong[MaxCoulombWords + 2];
        SetMagnitude(firstFactor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            firstMagnitude,
            first);
        SetMagnitude(secondFactor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            secondMagnitude,
            second);
        AddSignedMagnitudes(
            first,
            firstFactor == Fixed64.Zero
                ? 0
                : firstFactor < Fixed64.Zero ? -firstSign : firstSign,
            second,
            secondFactor == Fixed64.Zero
                ? 0
                : secondFactor < Fixed64.Zero ? -secondSign : secondSign,
            result,
            out resultSign);
    }

}
