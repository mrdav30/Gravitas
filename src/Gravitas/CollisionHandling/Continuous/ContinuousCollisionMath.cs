//=======================================================================
// ContinuousCollisionMath.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionMath
{
    public const int RotationalIntervalMaxDepth = 12;
    public const int RotationalIntervalNodeBudget = 64;

    public enum IntervalSearchStatus : byte
    {
        CertifiedNoHit,
        ExactHit,
        Unresolved,
    }

    // Pose reconstruction uses normalized fixed-point rotations. Cover both
    // absolute operation rounding and its radius-scaled positional effect;
    // failure to represent either term keeps the interval unresolved.
    private static readonly Fixed64 RotationalMotionUncertainty = Fixed64.Epsilon * 64;
    private static readonly Fixed64 RotationalRelativeUncertainty =
        FixedMath.CanonicalSinCosErrorBound * 16;
    private static readonly Fixed64 ClosestFeatureRelativeUncertainty =
        Fixed64.MinIncrement * 128;

    public readonly struct RotationalInterval
    {
        public RotationalInterval(Fixed64 lowerTime, Fixed64 upperTime, int depth)
        {
            LowerTime = lowerTime;
            UpperTime = upperTime;
            Depth = depth;
        }

        public Fixed64 LowerTime { get; }

        public Fixed64 UpperTime { get; }

        public int Depth { get; }
    }

    public static bool TryResolveRotationalSearchLimit(
        RotationalInterval interval,
        int processedNodeCount,
        bool hasWitness,
        Fixed64 witnessTime,
        out Fixed64 safeTime,
        out Fixed64 contactTime,
        out bool retainsWitness)
    {
        bool nodeBudgetExhausted = processedNodeCount >= RotationalIntervalNodeBudget;
        if (!nodeBudgetExhausted && interval.Depth < RotationalIntervalMaxDepth)
        {
            safeTime = default;
            contactTime = default;
            retainsWitness = false;
            return false;
        }

        retainsWitness = hasWitness;
        contactTime = hasWitness ? witnessTime : interval.LowerTime;
        // Max depth defines the accepted temporal resolution, so its same-target
        // witness may resolve this leaf only when it brackets the leaf itself. A
        // global witness from a later interval cannot certify the unresolved gap.
        // Hard node-budget exhaustion has no convergence guarantee either.
        bool witnessIsBracketed = hasWitness
            && witnessTime >= interval.LowerTime
            && witnessTime <= interval.UpperTime;
        safeTime = witnessIsBracketed && !nodeBudgetExhausted
            ? witnessTime
            : interval.LowerTime;
        return true;
    }

    public static bool ShouldContinueRotationalArbiter(
        Vector2d displacement,
        Fixed64 angularDistance) =>
        angularDistance > Fixed64.Epsilon
        || displacement.MagnitudeSquared > Fixed64.Epsilon;

    public static bool TryResolveRotationalIntervalMotionBound(
        Vector2d displacement,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 intervalSpan,
        out Fixed64 motionBound)
    {
        Vector2d halfIntervalDisplacement = displacement * (intervalSpan * Fixed64.Half);
        bool linearResolved = Vector2d.TryGetMagnitude(
            halfIntervalDisplacement,
            out Fixed64 linearMotion);
        bool angularResolved = Fixed64.TryMultiplyDivide(
            angularDistance,
            pivotRadius,
            intervalSpan,
            Fixed64.Two,
            out Fixed64 angularMotion);
        bool poseResolved = Fixed64.TryMultiplyDivide(
            pivotRadius,
            RotationalRelativeUncertainty,
            Fixed64.One,
            out Fixed64 poseUncertainty);
        bool combined = Fixed64.TryAdd(linearMotion, angularMotion, out motionBound)
            & Fixed64.TryAdd(motionBound, poseUncertainty, out motionBound)
            & Fixed64.TryAdd(motionBound, RotationalMotionUncertainty, out motionBound);
        if (!(linearResolved & angularResolved & poseResolved & combined))
        {
            motionBound = default;
            return false;
        }

        return true;
    }

    public static bool TryResolveRotationalIntervalMotionBound(
        Vector3d displacement,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 intervalSpan,
        out Fixed64 motionBound)
    {
        Vector3d halfIntervalDisplacement = displacement * (intervalSpan * Fixed64.Half);
        bool linearResolved = Vector3d.TryGetMagnitude(
            halfIntervalDisplacement,
            out Fixed64 linearMotion);
        bool angularResolved = Fixed64.TryMultiplyDivide(
            angularDistance,
            pivotRadius,
            intervalSpan,
            Fixed64.Two,
            out Fixed64 angularMotion);
        bool poseResolved = Fixed64.TryMultiplyDivide(
            pivotRadius,
            RotationalRelativeUncertainty,
            Fixed64.One,
            out Fixed64 poseUncertainty);
        bool combined = Fixed64.TryAdd(linearMotion, angularMotion, out motionBound)
            & Fixed64.TryAdd(motionBound, poseUncertainty, out motionBound)
            & Fixed64.TryAdd(motionBound, RotationalMotionUncertainty, out motionBound);
        if (!(linearResolved & angularResolved & poseResolved & combined))
        {
            motionBound = default;
            return false;
        }

        return true;
    }

    public static bool AreBoundsSeparatedByMoreThan(
        FixedBoundArea source,
        FixedBoundArea target,
        Fixed64 motionBound) =>
        IsAxisGapGreaterThan(source.Min.X, source.Max.X, target.Min.X, target.Max.X, motionBound)
        | IsAxisGapGreaterThan(source.Min.Y, source.Max.Y, target.Min.Y, target.Max.Y, motionBound);

    public static bool AreBoundsSeparatedByMoreThan(
        FixedBoundBox source,
        FixedBoundBox target,
        Fixed64 motionBound) =>
        IsAxisGapGreaterThan(source.Min.X, source.Max.X, target.Min.X, target.Max.X, motionBound)
        | IsAxisGapGreaterThan(source.Min.Y, source.Max.Y, target.Min.Y, target.Max.Y, motionBound)
        | IsAxisGapGreaterThan(source.Min.Z, source.Max.Z, target.Min.Z, target.Max.Z, motionBound);

    public static bool TrySubtractClosestFeatureUncertainty(
        Fixed64 separationGap,
        Fixed64 characteristicScale,
        out Fixed64 conservativeGap)
    {
        bool inputValid = separationGap > Fixed64.Zero
            & characteristicScale >= Fixed64.Zero;
        bool scaled = Fixed64.TryMultiplyDivide(
            characteristicScale,
            ClosestFeatureRelativeUncertainty,
            Fixed64.One,
            out Fixed64 scaledUncertainty);
        bool combined = Fixed64.TryAdd(
            scaledUncertainty,
            RotationalMotionUncertainty,
            out Fixed64 uncertainty);
        bool subtracted = Fixed64.TrySubtract(
            separationGap,
            uncertainty,
            out conservativeGap);
        if (!(inputValid & scaled & combined & subtracted))
        {
            conservativeGap = default;
            return false;
        }

        return conservativeGap > Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAxisGapGreaterThan(
        Fixed64 sourceMin,
        Fixed64 sourceMax,
        Fixed64 targetMin,
        Fixed64 targetMax,
        Fixed64 motionBound)
    {
        if (sourceMax < targetMin)
            return !Fixed64.TrySubtract(targetMin, sourceMax, out Fixed64 gap)
                | gap > motionBound;

        return targetMax < sourceMin
            & (!Fixed64.TrySubtract(sourceMin, targetMax, out Fixed64 reverseGap)
                | reverseGap > motionBound);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsWithinProxyRadius(
        Vector3d displacement,
        Fixed64 displacementMagnitudeSquared,
        Fixed64 proxyRadius)
    {
        Fixed64 proxyRadiusSquared = proxyRadius * proxyRadius;
        if (displacementMagnitudeSquared != Fixed64.MaxValue || proxyRadiusSquared != Fixed64.MaxValue)
            return displacementMagnitudeSquared <= proxyRadiusSquared;

        return Vector3d.TryGetMagnitude(displacement, out Fixed64 displacementMagnitude)
            && displacementMagnitude <= proxyRadius;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsWithinProxyRadius(
        Vector2d displacement,
        Fixed64 displacementMagnitudeSquared,
        Fixed64 proxyRadius)
    {
        Fixed64 proxyRadiusSquared = proxyRadius * proxyRadius;
        if (displacementMagnitudeSquared != Fixed64.MaxValue || proxyRadiusSquared != Fixed64.MaxValue)
            return displacementMagnitudeSquared <= proxyRadiusSquared;

        return Vector2d.TryGetMagnitude(displacement, out Fixed64 displacementMagnitude)
            && displacementMagnitude <= proxyRadius;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ResolveContactPointOnTarget(
        Vector3d sourceCenter,
        Vector3d targetCenter,
        Vector3d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector3d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool ShouldReplaceContinuousCollisionHit(
        Fixed64 candidateSafeTime,
        int candidateTargetId,
        bool hasCurrent,
        Fixed64 currentSafeTime,
        int currentTargetId)
    {
        if (!hasCurrent)
            return true;

        int timeCompare = candidateSafeTime.CompareTo(currentSafeTime);
        if (timeCompare != 0)
            return timeCompare < 0;

        return candidateTargetId < currentTargetId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ClipTranslationalTrajectoryInterval(
        Fixed64 queryStart,
        Fixed64 segmentStart,
        Fixed64 segmentEnd,
        out Fixed64 overlapStart,
        out Fixed64 overlapEnd,
        out Fixed64 sourceStartTime,
        out Fixed64 sourceEndTime)
    {
        overlapStart = FixedMath.Max(queryStart, segmentStart);
        overlapEnd = segmentEnd;
        Fixed64 querySpan = Fixed64.One - queryStart;
        sourceStartTime = FixedMath.Clamp01((overlapStart - queryStart) / querySpan);
        sourceEndTime = FixedMath.Clamp01((overlapEnd - queryStart) / querySpan);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsSupersededTranslationalBoundaryHit(
        bool hitAtSegmentEnd,
        Fixed64 overlapEnd,
        int segmentIndex,
        int segmentCount,
        Fixed64 successorStart) =>
        hitAtSegmentEnd
        && overlapEnd < Fixed64.One
        && segmentIndex + 1 < segmentCount
        && successorStart == overlapEnd;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryNormalizeTranslationalClosingSpeed(
        Fixed64 localClosingSpeed,
        Fixed64 sourceIntervalSpan,
        out Fixed64 closingSpeed)
    {
        if (localClosingSpeed <= Fixed64.Epsilon
            || sourceIntervalSpan <= Fixed64.Zero)
        {
            closingSpeed = Fixed64.Zero;
            return false;
        }

        closingSpeed = localClosingSpeed / sourceIntervalSpan;
        return true;
    }

    public static bool TryGetRelativeSphereOverlapDistanceInterval(
        Vector3d sourceStart,
        Vector3d sourceDisplacement,
        Fixed64 sourceRadius,
        Vector3d targetStart,
        Vector3d targetDisplacement,
        Fixed64 targetRadius,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out Vector3d relativeDisplacement,
        out Fixed64 relativeLength,
        out Vector3d normalForSource,
        out Fixed64 closingSpeed)
    {
        entryDistance = Fixed64.Zero;
        exitDistance = Fixed64.Zero;
        relativeDisplacement = Vector3d.Zero;
        normalForSource = Vector3d.Zero;
        closingSpeed = Fixed64.Zero;

        relativeDisplacement = ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            sourceDisplacement,
            targetDisplacement,
            out relativeLength);
        Vector3d sourceEnd = GetSweepEnd(sourceStart, sourceDisplacement);
        Vector3d targetEnd = GetSweepEnd(targetStart, targetDisplacement);
        if (relativeDisplacement.MagnitudeSquared <= Fixed64.Epsilon
            || !HasUsableCombinedRadius(sourceRadius, targetRadius))
        {
            return false;
        }

        if (!WideFiniteAxisIntersection.TryGetSphereDirectionDistanceInterval(
                sourceStart,
                relativeDisplacement,
                new FixedBoundSphere(targetStart, targetRadius),
                sourceRadius,
                relativeLength,
                out entryDistance,
                out exitDistance))
        {
            return false;
        }

        Vector3d sourceImpact = new FixedSegment(sourceStart, sourceEnd)
            .GetPointAtDistance(entryDistance, relativeLength);
        Vector3d targetImpact = new FixedSegment(targetStart, targetEnd)
            .GetPointAtDistance(entryDistance, relativeLength);
        normalForSource = ResolveNormal(targetImpact, sourceImpact, relativeDisplacement);
        closingSpeed = -Vector3d.Dot(relativeDisplacement, normalForSource);
        return true;
    }

    public static bool TryGetRelativeCircleOverlapDistanceInterval(
        Vector2d sourceStart,
        Vector2d sourceDisplacement,
        Fixed64 sourceRadius,
        Vector2d targetStart,
        Vector2d targetDisplacement,
        Fixed64 targetRadius,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out Vector2d relativeDisplacement,
        out Fixed64 relativeLength,
        out Vector2d normalForSource,
        out Fixed64 closingSpeed)
    {
        entryDistance = Fixed64.Zero;
        exitDistance = Fixed64.Zero;
        relativeDisplacement = Vector2d.Zero;
        normalForSource = Vector2d.Zero;
        closingSpeed = Fixed64.Zero;

        relativeDisplacement = ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            sourceDisplacement,
            targetDisplacement,
            out relativeLength);
        Vector2d sourceEnd = GetSweepEnd(sourceStart, sourceDisplacement);
        Vector2d targetEnd = GetSweepEnd(targetStart, targetDisplacement);
        if (relativeDisplacement.MagnitudeSquared <= Fixed64.Epsilon
            || !HasUsableCombinedRadius(sourceRadius, targetRadius))
        {
            return false;
        }

        if (!WideFiniteAxisIntersection.TryGetCircleDirectionDistanceInterval(
                sourceStart,
                relativeDisplacement,
                new FixedBoundCircle(targetStart, targetRadius),
                sourceRadius,
                relativeLength,
                out entryDistance,
                out exitDistance))
        {
            return false;
        }

        Vector2d sourceImpact = new FixedSegment2d(sourceStart, sourceEnd)
            .GetPointAtDistance(entryDistance, relativeLength);
        Vector2d targetImpact = new FixedSegment2d(targetStart, targetEnd)
            .GetPointAtDistance(entryDistance, relativeLength);
        normalForSource = ResolveNormal(targetImpact, sourceImpact, relativeDisplacement);
        closingSpeed = -Vector2d.Dot(relativeDisplacement, normalForSource);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasUsableCombinedRadius(Fixed64 sourceRadius, Fixed64 targetRadius)
    {
        if (sourceRadius > Fixed64.Epsilon || targetRadius > Fixed64.Epsilon)
            return true;

        return sourceRadius + targetRadius > Fixed64.Epsilon;
    }

    private static Vector3d GetSweepEnd(
        Vector3d start,
        Vector3d displacement)
    {
        if (!Vector3d.TryAdd(start, displacement, out Vector3d end))
            throw new System.ArgumentOutOfRangeException(nameof(displacement), "Continuous collision endpoint is outside the fixed-point coordinate domain.");

        return end;
    }

    private static Vector2d GetSweepEnd(
        Vector2d start,
        Vector2d displacement)
    {
        if (!Vector2d.TryAdd(start, displacement, out Vector2d end))
            throw new System.ArgumentOutOfRangeException(nameof(displacement), "Continuous collision endpoint is outside the fixed-point coordinate domain.");

        return end;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveNormal(
        Vector3d targetPosition,
        Vector3d sourcePosition,
        Vector3d relativeDisplacement)
    {
        Vector3d normal = Vector3d.GetDirection(targetPosition, sourcePosition);
        return normal != Vector3d.Zero ? normal : -relativeDisplacement.Normalized;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveNormal(
        Vector2d targetPosition,
        Vector2d sourcePosition,
        Vector2d relativeDisplacement)
    {
        Vector2d normal = Vector2d.GetDirection(targetPosition, sourcePosition);
        return normal != Vector2d.Zero ? normal : -relativeDisplacement.Normalized;
    }
}
