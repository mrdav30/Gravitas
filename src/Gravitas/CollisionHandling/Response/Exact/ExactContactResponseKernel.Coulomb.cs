//=======================================================================
// ExactContactResponseKernel.Coulomb.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <content>
/// Owns exact line and disk Coulomb-friction policy and materialization.
/// </content>
internal static partial class ExactContactResponseKernel
{
    private const int MaxCoulombWords = MaxResponseWords * 2;
    private const int MaxCoulombSquareWords = MaxCoulombWords * 2;
    // Normal-response ratios use fewer than 46 active words. A common tangent
    // denominator therefore uses fewer than 92, its squared magnitude fewer
    // than 185 including carry, and the largest radial comparison fewer than
    // 307. The remaining words prevent silent product truncation.
    private const int MaxCoulombComparisonWords = 320;

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetCoulombLineResponse(
        in ExactNormalConstraint3D normalConstraint,
        in ExactContactResponseOperand3D firstTangent,
        in ExactContactResponseOperand3D secondTangent,
        Vector3d tangent,
        Fixed64 accumulatedTangentImpulse,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out ExactCoulombResponse3D response)
    {
        response = default;
        bool inputsValid =
            staticFriction >= Fixed64.Zero
            & dynamicFriction >= Fixed64.Zero
            & tangent.IsNormalized();
        if (!inputsValid)
            return false;

        bool constraintValid =
            FixedMath.Abs(Vector3d.Dot(
                normalConstraint.Normal,
                tangent)) <= Fixed64.Epsilon
            & HaveMatchingParticipants(
                normalConstraint.First,
                firstTangent)
            & HaveMatchingParticipants(
                normalConstraint.Second,
                secondTangent);
        if (!constraintValid)
            return false;

        Span<ulong> normalNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> normalDenominator = stackalloc ulong[MaxResponseWords];
        Span<ulong> tangentNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> tangentDenominator = stackalloc ulong[MaxResponseWords];
        if (!TryGetCompletedNormalAccumulatorRatio(
                normalConstraint,
                normalNumerator,
                normalDenominator))
            return false;
        GetBilateralImpulseRatio(
            firstTangent,
            secondTangent,
            tangent,
            Fixed64.Zero,
            tangentNumerator,
            tangentDenominator,
            out int tangentSign);

        Span<ulong> desiredNumerator = stackalloc ulong[MaxResponseWords];
        AddFixedToRatio(
            tangentNumerator,
            tangentDenominator,
            tangentSign,
            accumulatedTangentImpulse,
            desiredNumerator,
            out int desiredSign);

        Span<ulong> staticNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> staticDenominator = stackalloc ulong[MaxResponseWords];
        GetFrictionLimit(
            normalNumerator,
            normalDenominator,
            staticFriction,
            staticNumerator,
            staticDenominator);

        Span<ulong> appliedNumerator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> appliedDenominator = stackalloc ulong[MaxCoulombWords];
        int appliedSign;
        Span<ulong> accumulatedNumerator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> accumulatedDenominator = stackalloc ulong[MaxCoulombWords];
        int accumulatedSign;
        appliedNumerator.Clear();
        appliedDenominator.Clear();
        accumulatedNumerator.Clear();
        accumulatedDenominator.Clear();
        Span<ulong> clampedNumerator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> clampedDenominator = stackalloc ulong[MaxResponseWords];
        if (CompareRatios(
                desiredNumerator,
                tangentDenominator,
                staticNumerator,
                staticDenominator) <= 0)
        {
            tangentNumerator.CopyTo(appliedNumerator);
            tangentDenominator.CopyTo(appliedDenominator);
            appliedSign = tangentSign;
            desiredNumerator.CopyTo(accumulatedNumerator);
            tangentDenominator.CopyTo(accumulatedDenominator);
            accumulatedSign = desiredSign;
        }
        else
        {
            GetFrictionLimit(
                normalNumerator,
                normalDenominator,
                dynamicFriction,
                clampedNumerator,
                clampedDenominator);
            clampedNumerator.CopyTo(accumulatedNumerator);
            clampedDenominator.CopyTo(accumulatedDenominator);
            accumulatedSign = WideArithmetic.IsZeroMagnitude(
                clampedNumerator)
                ? 0
                : desiredSign;
            SubtractFixedFromRatio(
                clampedNumerator,
                clampedDenominator,
                accumulatedSign,
                accumulatedTangentImpulse,
                appliedNumerator,
                appliedDenominator,
                out appliedSign);
        }

        bool resolved = TryGetLinearVelocityDelta(
            firstTangent.LinearImpulseAxis,
            firstTangent.InverseMass,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d firstLinear);
        resolved &= TryGetAngularVelocityDelta(
            firstTangent.Lever,
            -tangent,
            firstTangent.InverseInertia,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d firstAngular);
        resolved &= TryGetLinearVelocityDelta(
            secondTangent.LinearImpulseAxis,
            secondTangent.InverseMass,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d secondLinear);
        resolved &= TryGetAngularVelocityDelta(
            secondTangent.Lever,
            tangent,
            secondTangent.InverseInertia,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d secondAngular);
        if (!resolved)
            return false;

        bool hasAccumulatedProjection = Fixed64.TryGetSignedRawRatio(
            accumulatedNumerator,
            accumulatedDenominator,
            accumulatedSign < 0,
            out Fixed64 accumulatedProjection);
        response = new ExactCoulombResponse3D(
            !WideArithmetic.IsZeroMagnitude(appliedNumerator),
            firstLinear,
            firstAngular,
            secondLinear,
            secondAngular,
            hasAccumulatedProjection,
            accumulatedProjection,
            hasSecondaryAccumulatedImpulse: false,
            secondaryAccumulatedImpulse: default);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetCoulombDiskResponse(
        Vector3d normal,
        Fixed64 completedNormalImpulse,
        in ExactContactResponseOperand3D primaryFirst,
        in ExactContactResponseOperand3D primarySecond,
        Vector3d primaryTangent,
        Fixed64 accumulatedPrimaryTangentImpulse,
        in ExactContactResponseOperand3D secondaryFirst,
        in ExactContactResponseOperand3D secondarySecond,
        Vector3d secondaryTangent,
        Fixed64 accumulatedSecondaryTangentImpulse,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out ExactCoulombResponse3D response)
    {
        response = default;
        if (completedNormalImpulse < Fixed64.Zero
            || !AreCoulombDiskInputsValid(
                normal,
                primaryFirst,
                primarySecond,
                primaryTangent,
                secondaryFirst,
                secondarySecond,
                secondaryTangent,
                staticFriction,
                dynamicFriction))
        {
            return false;
        }

        Span<ulong> normalNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> normalDenominator = stackalloc ulong[MaxResponseWords];
        SetMagnitude(completedNormalImpulse, normalNumerator);
        normalDenominator.Clear();
        normalDenominator[0] = 1UL;
        return TryGetCoulombDiskResponseCore(
            normalNumerator,
            normalDenominator,
            primaryFirst,
            primarySecond,
            primaryTangent,
            accumulatedPrimaryTangentImpulse,
            secondaryFirst,
            secondarySecond,
            secondaryTangent,
            accumulatedSecondaryTangentImpulse,
            staticFriction,
            dynamicFriction,
            Fixed64.Epsilon,
            out response);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetCoulombDiskResponse(
        in ExactNormalConstraint3D normalConstraint,
        in ExactContactResponseOperand3D primaryFirst,
        in ExactContactResponseOperand3D primarySecond,
        Vector3d primaryTangent,
        Fixed64 accumulatedPrimaryTangentImpulse,
        in ExactContactResponseOperand3D secondaryFirst,
        in ExactContactResponseOperand3D secondarySecond,
        Vector3d secondaryTangent,
        Fixed64 accumulatedSecondaryTangentImpulse,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out ExactCoulombResponse3D response)
    {
        response = default;
        bool inputsValid = AreCoulombDiskInputsValid(
                normalConstraint.Normal,
                primaryFirst,
                primarySecond,
                primaryTangent,
                secondaryFirst,
                secondarySecond,
                secondaryTangent,
                staticFriction,
                dynamicFriction)
            & HaveMatchingParticipants(
                normalConstraint.First,
                primaryFirst)
            & HaveMatchingParticipants(
                normalConstraint.Second,
                primarySecond)
            & HaveMatchingParticipants(primaryFirst, secondaryFirst)
            & HaveMatchingParticipants(primarySecond, secondarySecond);
        if (!inputsValid)
            return false;

        Span<ulong> normalNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> normalDenominator = stackalloc ulong[MaxResponseWords];
        if (!TryGetCompletedNormalAccumulatorRatio(
                normalConstraint,
                normalNumerator,
                normalDenominator))
            return false;

        return TryGetCoulombDiskResponseCore(
            normalNumerator,
            normalDenominator,
            primaryFirst,
            primarySecond,
            primaryTangent,
            accumulatedPrimaryTangentImpulse,
            secondaryFirst,
            secondarySecond,
            secondaryTangent,
            accumulatedSecondaryTangentImpulse,
            staticFriction,
            dynamicFriction,
            Fixed64.Zero,
            out response);
    }

    private static bool TryGetCoulombDiskResponseCore(
        ReadOnlySpan<ulong> normalNumerator,
        ReadOnlySpan<ulong> normalDenominator,
        in ExactContactResponseOperand3D primaryFirst,
        in ExactContactResponseOperand3D primarySecond,
        Vector3d primaryTangent,
        Fixed64 accumulatedPrimaryTangentImpulse,
        in ExactContactResponseOperand3D secondaryFirst,
        in ExactContactResponseOperand3D secondarySecond,
        Vector3d secondaryTangent,
        Fixed64 accumulatedSecondaryTangentImpulse,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        Fixed64 velocityDeadzone,
        out ExactCoulombResponse3D response)
    {
        response = default;
        Span<ulong> primaryNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> primaryDenominator = stackalloc ulong[MaxResponseWords];
        Span<ulong> secondaryNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> secondaryDenominator = stackalloc ulong[MaxResponseWords];
        GetBilateralImpulseRatio(
            primaryFirst,
            primarySecond,
            primaryTangent,
            velocityDeadzone,
            primaryNumerator,
            primaryDenominator,
            out int primarySign);
        GetBilateralImpulseRatio(
            secondaryFirst,
            secondarySecond,
            secondaryTangent,
            velocityDeadzone,
            secondaryNumerator,
            secondaryDenominator,
            out int secondarySign);

        Span<ulong> desiredPrimaryNumerator =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> desiredSecondaryNumerator =
            stackalloc ulong[MaxResponseWords];
        AddFixedToRatio(
            primaryNumerator,
            primaryDenominator,
            primarySign,
            accumulatedPrimaryTangentImpulse,
            desiredPrimaryNumerator,
            out int desiredPrimarySign);
        AddFixedToRatio(
            secondaryNumerator,
            secondaryDenominator,
            secondarySign,
            accumulatedSecondaryTangentImpulse,
            desiredSecondaryNumerator,
            out int desiredSecondarySign);

        Span<ulong> commonDenominator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> primaryAtCommon = stackalloc ulong[MaxCoulombWords];
        Span<ulong> secondaryAtCommon = stackalloc ulong[MaxCoulombWords];
        WideArithmetic.MultiplyMagnitudes(
            primaryDenominator,
            secondaryDenominator,
            commonDenominator);
        WideArithmetic.MultiplyMagnitudes(
            desiredPrimaryNumerator,
            secondaryDenominator,
            primaryAtCommon);
        WideArithmetic.MultiplyMagnitudes(
            desiredSecondaryNumerator,
            primaryDenominator,
            secondaryAtCommon);

        Span<ulong> magnitudeSquared = stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> square = stackalloc ulong[MaxCoulombSquareWords];
        WideArithmetic.MultiplyMagnitudes(
            primaryAtCommon,
            primaryAtCommon,
            magnitudeSquared);
        WideArithmetic.MultiplyMagnitudes(
            secondaryAtCommon,
            secondaryAtCommon,
            square);
        WideArithmetic.AddMagnitudeInto(square, magnitudeSquared);

        Span<ulong> staticNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> staticDenominator = stackalloc ulong[MaxResponseWords];
        GetFrictionLimit(
            normalNumerator,
            normalDenominator,
            staticFriction,
            staticNumerator,
            staticDenominator);
        bool withinStaticLimit = IsVectorWithinLimit(
            magnitudeSquared,
            commonDenominator,
            staticNumerator,
            staticDenominator);

        Span<ulong> dynamicNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> dynamicDenominator = stackalloc ulong[MaxResponseWords];
        if (!withinStaticLimit)
        {
            GetFrictionLimit(
                normalNumerator,
                normalDenominator,
                dynamicFriction,
                dynamicNumerator,
                dynamicDenominator);
        }

        bool useDynamicProjection =
            !withinStaticLimit
            && !WideArithmetic.IsZeroMagnitude(magnitudeSquared)
            && !WideArithmetic.IsZeroMagnitude(dynamicNumerator);
        bool hasAppliedImpulse;
        bool resolved;
        Vector3d firstLinear;
        Vector3d firstAngular;
        Vector3d secondLinear;
        Vector3d secondAngular;
        if (useDynamicProjection)
        {
            hasAppliedImpulse =
                !IsRadialProjectionEqualToFixed(
                    primaryAtCommon,
                    desiredPrimarySign,
                    dynamicNumerator,
                    dynamicDenominator,
                    magnitudeSquared,
                    accumulatedPrimaryTangentImpulse)
                || !IsRadialProjectionEqualToFixed(
                    secondaryAtCommon,
                    desiredSecondarySign,
                    dynamicNumerator,
                    dynamicDenominator,
                    magnitudeSquared,
                    accumulatedSecondaryTangentImpulse);
            resolved = TryGetDiskVelocityDeltas(
                primaryFirst,
                primarySecond,
                secondaryFirst,
                secondarySecond,
                primaryTangent,
                secondaryTangent,
                primaryAtCommon,
                desiredPrimarySign,
                secondaryAtCommon,
                desiredSecondarySign,
                commonDenominator,
                rational: false,
                dynamicNumerator,
                dynamicDenominator,
                magnitudeSquared,
                accumulatedPrimaryTangentImpulse,
                accumulatedSecondaryTangentImpulse,
                out firstLinear,
                out firstAngular,
                out secondLinear,
                out secondAngular);
        }
        else
        {
            Span<ulong> zero = stackalloc ulong[MaxCoulombWords];
            Span<ulong> appliedPrimaryNumerator =
                stackalloc ulong[MaxCoulombWords];
            Span<ulong> appliedDenominator =
                stackalloc ulong[MaxCoulombWords];
            Span<ulong> appliedSecondaryNumerator =
                stackalloc ulong[MaxCoulombWords];
            zero.Clear();
            SubtractFixedFromRatio(
                withinStaticLimit ? primaryAtCommon : zero,
                commonDenominator,
                withinStaticLimit ? desiredPrimarySign : 0,
                accumulatedPrimaryTangentImpulse,
                appliedPrimaryNumerator,
                appliedDenominator,
                out int appliedPrimarySign);
            SubtractFixedFromRatio(
                withinStaticLimit ? secondaryAtCommon : zero,
                commonDenominator,
                withinStaticLimit ? desiredSecondarySign : 0,
                accumulatedSecondaryTangentImpulse,
                appliedSecondaryNumerator,
                appliedDenominator,
                out int appliedSecondarySign);
            hasAppliedImpulse =
                appliedPrimarySign != 0
                || appliedSecondarySign != 0;
            resolved = TryGetDiskVelocityDeltas(
                primaryFirst,
                primarySecond,
                secondaryFirst,
                secondarySecond,
                primaryTangent,
                secondaryTangent,
                appliedPrimaryNumerator,
                appliedPrimarySign,
                appliedSecondaryNumerator,
                appliedSecondarySign,
                appliedDenominator,
                rational: true,
                dynamicNumerator,
                dynamicDenominator,
                magnitudeSquared,
                Fixed64.Zero,
                Fixed64.Zero,
                out firstLinear,
                out firstAngular,
                out secondLinear,
                out secondAngular);
        }
        if (!resolved)
            return false;

        bool hasPrimaryProjection;
        bool hasSecondaryProjection;
        Fixed64 primaryProjection;
        Fixed64 secondaryProjection;
        if (withinStaticLimit)
        {
            hasPrimaryProjection = Fixed64.TryGetSignedRawRatio(
                primaryAtCommon,
                commonDenominator,
                desiredPrimarySign < 0,
                out primaryProjection);
            hasSecondaryProjection = Fixed64.TryGetSignedRawRatio(
                secondaryAtCommon,
                commonDenominator,
                desiredSecondarySign < 0,
                out secondaryProjection);
        }
        else if (useDynamicProjection)
        {
            Span<ulong> projectedNumerator =
                stackalloc ulong[MaxCoulombWords + MaxResponseWords];
            WideArithmetic.MultiplyMagnitudes(
                primaryAtCommon,
                dynamicNumerator,
                projectedNumerator);
            hasPrimaryProjection = TryGetSignedRatioOverSquareRoot(
                projectedNumerator,
                dynamicDenominator,
                magnitudeSquared,
                desiredPrimarySign < 0,
                out primaryProjection);
            WideArithmetic.MultiplyMagnitudes(
                secondaryAtCommon,
                dynamicNumerator,
                projectedNumerator);
            hasSecondaryProjection = TryGetSignedRatioOverSquareRoot(
                projectedNumerator,
                dynamicDenominator,
                magnitudeSquared,
                desiredSecondarySign < 0,
                out secondaryProjection);
        }
        else
        {
            hasPrimaryProjection = true;
            hasSecondaryProjection = true;
            primaryProjection = Fixed64.Zero;
            secondaryProjection = Fixed64.Zero;
        }

        response = new ExactCoulombResponse3D(
            hasAppliedImpulse,
            firstLinear,
            firstAngular,
            secondLinear,
            secondAngular,
            hasPrimaryProjection,
            primaryProjection,
            hasSecondaryProjection,
            secondaryProjection);
        return true;
    }

    private static bool AreCoulombDiskInputsValid(
        Vector3d normal,
        in ExactContactResponseOperand3D primaryFirst,
        in ExactContactResponseOperand3D primarySecond,
        Vector3d primaryTangent,
        in ExactContactResponseOperand3D secondaryFirst,
        in ExactContactResponseOperand3D secondarySecond,
        Vector3d secondaryTangent,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction) =>
        staticFriction >= Fixed64.Zero
        & dynamicFriction >= Fixed64.Zero
        & primaryFirst.Lever.Denominator.Sign != 0
        & primarySecond.Lever.Denominator.Sign != 0
        & primaryFirst.InverseMass >= Fixed64.Zero
        & primarySecond.InverseMass >= Fixed64.Zero
        & normal.IsNormalized()
        & primaryTangent.IsNormalized()
        & secondaryTangent.IsNormalized()
        & FixedMath.Abs(Vector3d.Dot(
            primaryTangent,
            secondaryTangent)) <= Fixed64.Epsilon
        & FixedMath.Abs(Vector3d.Dot(
            normal,
            primaryTangent)) <= Fixed64.Epsilon
        & FixedMath.Abs(Vector3d.Dot(
            normal,
            secondaryTangent)) <= Fixed64.Epsilon
        & HaveMatchingParticipants(primaryFirst, secondaryFirst)
        & HaveMatchingParticipants(primarySecond, secondarySecond);

    private static bool TryGetCompletedNormalAccumulatorRatio(
        in ExactNormalConstraint3D constraint,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        numerator.Clear();
        denominator.Clear();
        ExactContactResponseOperand3D first = constraint.First;
        ExactContactResponseOperand3D second = constraint.Second;
        bool inputsValid =
            first.Lever.Denominator.Sign != 0
            & second.Lever.Denominator.Sign != 0
            & constraint.Restitution >= Fixed64.Zero
            & constraint.RestitutionVelocityThreshold >= Fixed64.Zero
            & constraint.AccumulatedImpulse >= Fixed64.Zero
            & constraint.PositiveImpulseScale >= Fixed64.Zero
            & constraint.NegativeImpulseScale >= Fixed64.Zero
            & first.InverseMass >= Fixed64.Zero
            & second.InverseMass >= Fixed64.Zero
            & constraint.Normal.IsNormalized();
        if (!inputsValid)
            return false;

        ExactLever3D.GetRelativePointVelocityRatio(
            first.LinearVelocity,
            first.AngularVelocity,
            first.Lever,
            second.LinearVelocity,
            second.AngularVelocity,
            second.Lever,
            constraint.Normal,
            out Signed832 velocityNumerator,
            out Signed832 velocityDenominator);

        Span<ulong> effectiveNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> effectiveDenominator = stackalloc ulong[MaxResponseWords];
        GetEffectiveMassRatio(
            first,
            second,
            constraint.Normal,
            effectiveNumerator,
            effectiveDenominator);
        int velocitySign =
            velocityNumerator.Sign * velocityDenominator.Sign;
        if (velocitySign == 0
            || WideArithmetic.IsZeroMagnitude(effectiveNumerator))
        {
            SetMagnitude(constraint.AccumulatedImpulse, numerator);
            denominator[0] = 1UL;
            return true;
        }

        bool hasVelocityProjection = Fixed64.TryGetSignedRawRatio(
            velocityNumerator,
            velocityDenominator,
            0,
            out Fixed64 velocityProjection);
        bool applyRestitution = velocitySign < 0
            & (!hasVelocityProjection
                | velocityProjection
                    < -constraint.RestitutionVelocityThreshold);
        Fixed64 scale = velocitySign < 0
            ? constraint.PositiveImpulseScale
            : constraint.NegativeImpulseScale;
        Span<ulong> impulseNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> impulseDenominator = stackalloc ulong[MaxResponseWords];
        BuildImpulseRatio(
            velocityNumerator,
            velocityDenominator,
            effectiveNumerator,
            effectiveDenominator,
            applyRestitution ? constraint.Restitution : Fixed64.Zero,
            scale,
            impulseNumerator,
            impulseDenominator);

        Span<ulong> accumulatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> accumulatorAtImpulseDenominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(constraint.AccumulatedImpulse, accumulatorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            accumulatorMagnitude,
            impulseDenominator,
            accumulatorAtImpulseDenominator);
        int impulseSign = -velocitySign;
        if (impulseSign < 0
            && WideArithmetic.CompareMagnitudeEqualLength(
                impulseNumerator,
                accumulatorAtImpulseDenominator) >= 0)
        {
            denominator[0] = 1UL;
            return true;
        }

        accumulatorAtImpulseDenominator.CopyTo(numerator);
        if (impulseSign > 0)
            WideArithmetic.AddMagnitudeInto(impulseNumerator, numerator);
        else
            WideArithmetic.SubtractEqualMagnitudes(
                accumulatorAtImpulseDenominator,
                impulseNumerator,
                numerator);
        impulseDenominator.CopyTo(denominator);
        return true;
    }

    private static void GetBilateralImpulseRatio(
        in ExactContactResponseOperand3D first,
        in ExactContactResponseOperand3D second,
        Vector3d tangent,
        Fixed64 velocityDeadzone,
        Span<ulong> numerator,
        Span<ulong> denominator,
        out int sign)
    {
        numerator.Clear();
        denominator.Clear();
        sign = 0;
        ExactLever3D.GetRelativePointVelocityRatio(
            first.LinearVelocity,
            first.AngularVelocity,
            first.Lever,
            second.LinearVelocity,
            second.AngularVelocity,
            second.Lever,
            tangent,
            out Signed832 velocityNumerator,
            out Signed832 velocityDenominator);

        Span<ulong> effectiveNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> effectiveDenominator = stackalloc ulong[MaxResponseWords];
        GetEffectiveMassRatio(
            first,
            second,
            tangent,
            effectiveNumerator,
            effectiveDenominator);
        int velocitySign =
            velocityNumerator.Sign * velocityDenominator.Sign;
        bool suppressVelocity = velocityDeadzone != Fixed64.Zero
            && Fixed64.TryGetSignedRawRatio(
                velocityNumerator,
                velocityDenominator,
                0,
                out Fixed64 velocityProjection)
            && velocityProjection >= -velocityDeadzone
            && velocityProjection <= velocityDeadzone;
        if (velocitySign == 0
            || suppressVelocity
            || WideArithmetic.IsZeroMagnitude(effectiveNumerator))
        {
            denominator[0] = 1UL;
            return;
        }

        BuildImpulseRatio(
            velocityNumerator,
            velocityDenominator,
            effectiveNumerator,
            effectiveDenominator,
            Fixed64.Zero,
            Fixed64.One,
            numerator,
        denominator);
        sign = -velocitySign;
    }

    private static void GetFrictionLimit(
        ReadOnlySpan<ulong> normalNumerator,
        ReadOnlySpan<ulong> normalDenominator,
        Fixed64 friction,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        Span<ulong> frictionMagnitude =
            stackalloc ulong[3];
        Span<ulong> fixedScale =
            stackalloc ulong[1];
        SetMagnitude(friction, frictionMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        WideArithmetic.MultiplyMagnitudes(
            normalNumerator,
            frictionMagnitude,
            numerator);
        WideArithmetic.MultiplyMagnitudes(
            normalDenominator,
            fixedScale,
            denominator);
    }

    private static void AddFixedToRatio(
        ReadOnlySpan<ulong> ratioNumerator,
        ReadOnlySpan<ulong> ratioDenominator,
        int ratioSign,
        Fixed64 value,
        Span<ulong> resultNumerator,
        out int resultSign)
    {
        Span<ulong> valueMagnitude = stackalloc ulong[3];
        Span<ulong> valueAtDenominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(value, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            valueMagnitude,
            ratioDenominator,
            valueAtDenominator);
        AddSignedMagnitudes(
            ratioNumerator,
            ratioSign,
            valueAtDenominator,
            value == Fixed64.Zero ? 0 : value < Fixed64.Zero ? -1 : 1,
            resultNumerator,
            out resultSign);
    }

    private static void SubtractFixedFromRatio(
        ReadOnlySpan<ulong> ratioNumerator,
        ReadOnlySpan<ulong> ratioDenominator,
        int ratioSign,
        Fixed64 value,
        Span<ulong> resultNumerator,
        Span<ulong> resultDenominator,
        out int resultSign)
    {
        Span<ulong> valueMagnitude = stackalloc ulong[3];
        Span<ulong> valueAtDenominator =
            stackalloc ulong[MaxCoulombWords];
        SetMagnitude(value, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            valueMagnitude,
            ratioDenominator,
            valueAtDenominator);
        AddSignedMagnitudes(
            ratioNumerator,
            ratioSign,
            valueAtDenominator,
            value == Fixed64.Zero ? 0 : value < Fixed64.Zero ? 1 : -1,
            resultNumerator,
            out resultSign);
        resultDenominator.Clear();
        ratioDenominator.CopyTo(resultDenominator);
    }

    private static int CompareRatios(
        ReadOnlySpan<ulong> firstNumerator,
        ReadOnlySpan<ulong> firstDenominator,
        ReadOnlySpan<ulong> secondNumerator,
        ReadOnlySpan<ulong> secondDenominator)
    {
        Span<ulong> first = stackalloc ulong[MaxCoulombWords];
        Span<ulong> second = stackalloc ulong[MaxCoulombWords];
        WideArithmetic.MultiplyMagnitudes(
            firstNumerator,
            secondDenominator,
            first);
        WideArithmetic.MultiplyMagnitudes(
            secondNumerator,
            firstDenominator,
            second);
        return WideArithmetic.CompareMagnitudeEqualLength(first, second);
    }

    private static bool IsVectorWithinLimit(
        ReadOnlySpan<ulong> vectorMagnitudeSquared,
        ReadOnlySpan<ulong> vectorDenominator,
        ReadOnlySpan<ulong> limitNumerator,
        ReadOnlySpan<ulong> limitDenominator)
    {
        Span<ulong> denominatorSquared =
            stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> limitNumeratorSquared =
            stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> limitDenominatorSquared =
            stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> left = stackalloc ulong[MaxCoulombComparisonWords];
        Span<ulong> right = stackalloc ulong[MaxCoulombComparisonWords];
        WideArithmetic.MultiplyMagnitudes(
            vectorDenominator,
            vectorDenominator,
            denominatorSquared);
        WideArithmetic.MultiplyMagnitudes(
            limitNumerator,
            limitNumerator,
            limitNumeratorSquared);
        WideArithmetic.MultiplyMagnitudes(
            limitDenominator,
            limitDenominator,
            limitDenominatorSquared);
        WideArithmetic.MultiplyMagnitudes(
            vectorMagnitudeSquared,
            limitDenominatorSquared,
            left);
        WideArithmetic.MultiplyMagnitudes(
            limitNumeratorSquared,
            denominatorSquared,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) <= 0;
    }

    private static void AddSignedMagnitudes(
        ReadOnlySpan<ulong> first,
        int firstSign,
        ReadOnlySpan<ulong> second,
        int secondSign,
        Span<ulong> result,
        out int resultSign)
    {
        result.Clear();
        resultSign = 0;
        if (firstSign != 0 & !WideArithmetic.IsZeroMagnitude(first))
            WideArithmetic.AddSignedMagnitude(
                first,
                firstSign,
                result,
                ref resultSign);
        if (secondSign != 0 & !WideArithmetic.IsZeroMagnitude(second))
            WideArithmetic.AddSignedMagnitude(
                second,
                secondSign,
                result,
                ref resultSign);
    }

    private static bool TryGetSignedRatioOverSquareRoot(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        bool negative,
        out Fixed64 result)
    {
        if (WideArithmetic.IsZeroMagnitude(numerator))
        {
            result = Fixed64.Zero;
            return true;
        }

        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (CompareRatioOverSquareRoot(
                numerator,
                denominator,
                radicand,
                limit) > 0)
        {
            result = default;
            return false;
        }

        ulong low = 0UL;
        ulong high = limit;
        while (low < high)
        {
            ulong difference = high - low;
            ulong middle = low + (difference >> 1) + (difference & 1UL);
            if (CompareRatioOverSquareRoot(
                    numerator,
                    denominator,
                    radicand,
                    middle) >= 0)
            {
                low = middle;
            }
            else
            {
                high = middle - 1UL;
            }
        }

        ulong quotient = low;
        Span<ulong> doubledQuotient = stackalloc ulong[2];
        doubledQuotient[0] = (quotient << 1) | 1UL;
        doubledQuotient[1] = quotient >> 63;
        int midpointComparison = CompareRatioOverSquareRoot(
            numerator,
            denominator,
            radicand,
            doubledQuotient,
            numeratorMultiplier: 2UL);
        bool roundUp = midpointComparison > 0
            | (midpointComparison == 0 & (quotient & 1UL) != 0UL);
        if (roundUp & quotient < limit)
        {
            quotient++;
        }

        result = new Fixed64(
            negative ? unchecked(-(long)quotient) : (long)quotient);
        return true;
    }

    private static int CompareRatioOverSquareRoot(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        ulong candidate)
    {
        Span<ulong> candidateMagnitude = stackalloc ulong[1];
        candidateMagnitude.Clear();
        candidateMagnitude[0] = candidate;
        return CompareRatioOverSquareRoot(
            numerator,
            denominator,
            radicand,
            candidateMagnitude,
            numeratorMultiplier: 1UL);
    }

    private static int CompareRatioOverSquareRoot(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        ReadOnlySpan<ulong> candidate,
        ulong numeratorMultiplier)
    {
        Span<ulong> multiplier = stackalloc ulong[1];
        multiplier.Clear();
        multiplier[0] = numeratorMultiplier;
        Span<ulong> scaledNumerator =
            stackalloc ulong[(MaxCoulombComparisonWords / 2) + 1];
        Span<ulong> left = stackalloc ulong[MaxCoulombComparisonWords];
        Span<ulong> denominatorTimesCandidate =
            stackalloc ulong[(MaxCoulombComparisonWords / 2) + 1];
        Span<ulong> rightBase =
            stackalloc ulong[MaxCoulombComparisonWords];
        Span<ulong> right = stackalloc ulong[MaxCoulombComparisonWords];
        WideArithmetic.MultiplyMagnitudes(
            numerator,
            multiplier,
            scaledNumerator);
        WideArithmetic.MultiplyMagnitudes(
            scaledNumerator,
            scaledNumerator,
            left);
        WideArithmetic.MultiplyMagnitudes(
            denominator,
            candidate,
            denominatorTimesCandidate);
        WideArithmetic.MultiplyMagnitudes(
            denominatorTimesCandidate,
            denominatorTimesCandidate,
            rightBase);
        WideArithmetic.MultiplyMagnitudes(rightBase, radicand, right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right);
    }

    private static bool HaveMatchingParticipants(
        in ExactContactResponseOperand3D first,
        in ExactContactResponseOperand3D second) =>
        first.Lever.XNumerator.Equals(second.Lever.XNumerator)
        & first.Lever.YNumerator.Equals(second.Lever.YNumerator)
        & first.Lever.ZNumerator.Equals(second.Lever.ZNumerator)
        & first.Lever.Denominator.Equals(second.Lever.Denominator)
        & first.InverseMass == second.InverseMass
        & first.InverseInertia == second.InverseInertia;
}
