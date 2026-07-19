//=======================================================================
// ContinuousCollisionMath.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionMath
{
    public const int RotationalIntervalMaxDepth = 12;
    public const int RotationalIntervalNodeBudget = 64;

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

    public static bool TryResolveRotationalIntervalMotionBound(
        Vector2d displacement,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 intervalSpan,
        out Fixed64 motionBound)
    {
        Vector2d halfIntervalDisplacement = displacement * (intervalSpan * Fixed64.Half);
        if (!Vector2d.TryGetMagnitude(halfIntervalDisplacement, out Fixed64 linearMotion)
            || !Fixed64.TryMultiplyDivide(
                angularDistance,
                pivotRadius,
                intervalSpan,
                Fixed64.Two,
                out Fixed64 angularMotion)
            || !Fixed64.TryMultiplyDivide(
                pivotRadius,
                RotationalRelativeUncertainty,
                Fixed64.One,
                out Fixed64 poseUncertainty)
            || !Fixed64.TryAdd(linearMotion, angularMotion, out motionBound)
            || !Fixed64.TryAdd(motionBound, poseUncertainty, out motionBound)
            || !Fixed64.TryAdd(motionBound, RotationalMotionUncertainty, out motionBound))
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
        if (!Vector3d.TryGetMagnitude(halfIntervalDisplacement, out Fixed64 linearMotion)
            || !Fixed64.TryMultiplyDivide(
                angularDistance,
                pivotRadius,
                intervalSpan,
                Fixed64.Two,
                out Fixed64 angularMotion)
            || !Fixed64.TryMultiplyDivide(
                pivotRadius,
                RotationalRelativeUncertainty,
                Fixed64.One,
                out Fixed64 poseUncertainty)
            || !Fixed64.TryAdd(linearMotion, angularMotion, out motionBound)
            || !Fixed64.TryAdd(motionBound, poseUncertainty, out motionBound)
            || !Fixed64.TryAdd(motionBound, RotationalMotionUncertainty, out motionBound))
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
        || IsAxisGapGreaterThan(source.Min.Y, source.Max.Y, target.Min.Y, target.Max.Y, motionBound);

    public static bool AreBoundsSeparatedByMoreThan(
        FixedBoundBox source,
        FixedBoundBox target,
        Fixed64 motionBound) =>
        IsAxisGapGreaterThan(source.Min.X, source.Max.X, target.Min.X, target.Max.X, motionBound)
        || IsAxisGapGreaterThan(source.Min.Y, source.Max.Y, target.Min.Y, target.Max.Y, motionBound)
        || IsAxisGapGreaterThan(source.Min.Z, source.Max.Z, target.Min.Z, target.Max.Z, motionBound);

    public static bool TrySubtractClosestFeatureUncertainty(
        Fixed64 separationGap,
        Fixed64 characteristicScale,
        out Fixed64 conservativeGap)
    {
        if (separationGap <= Fixed64.Zero
            || characteristicScale < Fixed64.Zero
            || !Fixed64.TryMultiplyDivide(
                characteristicScale,
                ClosestFeatureRelativeUncertainty,
                Fixed64.One,
                out Fixed64 scaledUncertainty)
            || !Fixed64.TryAdd(
                scaledUncertainty,
                RotationalMotionUncertainty,
                out Fixed64 uncertainty)
            || !Fixed64.TrySubtract(separationGap, uncertainty, out conservativeGap))
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
            return Fixed64.TrySubtract(targetMin, sourceMax, out Fixed64 gap) && gap > motionBound;

        return targetMax < sourceMin
            && Fixed64.TrySubtract(sourceMin, targetMax, out Fixed64 reverseGap)
            && reverseGap > motionBound;
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
    public static bool TrySweepRelativeSpheres(
        Vector3d sourceStart,
        Vector3d sourceDisplacement,
        Fixed64 sourceRadius,
        Vector3d targetStart,
        Vector3d targetDisplacement,
        Fixed64 targetRadius,
        out Fixed64 normalizedTime,
        out Vector3d normalForSource,
        out Fixed64 closingSpeed)
    {
        normalizedTime = Fixed64.Zero;
        normalForSource = Vector3d.Zero;
        closingSpeed = Fixed64.Zero;

        Vector3d relativeDisplacement = ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            sourceDisplacement,
            targetDisplacement,
            out _);
        Vector3d sourceEnd = GetSweepEnd(sourceStart, sourceDisplacement);
        Vector3d targetEnd = GetSweepEnd(targetStart, targetDisplacement);
        if (relativeDisplacement.MagnitudeSquared <= Fixed64.Epsilon
            || !HasUsableCombinedRadius(sourceRadius, targetRadius))
        {
            return false;
        }

        if (!RadialSweepAdmission.TryIntersect(
                sourceStart,
                relativeDisplacement,
                Fixed64.One,
                targetStart,
                targetRadius,
                sourceRadius,
                sourceEnd,
                targetEnd,
                out Fixed64 time))
            return false;

        Vector3d sourceImpact = Vector3d.Lerp(sourceStart, sourceEnd, time);
        Vector3d targetImpact = Vector3d.Lerp(targetStart, targetEnd, time);
        normalForSource = ResolveNormal(targetImpact, sourceImpact, relativeDisplacement);
        closingSpeed = -Vector3d.Dot(relativeDisplacement, normalForSource);
        if (closingSpeed <= Fixed64.Epsilon)
            return false;

        normalizedTime = time;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TrySweepRelativeCircles(
        Vector2d sourceStart,
        Vector2d sourceDisplacement,
        Fixed64 sourceRadius,
        Vector2d targetStart,
        Vector2d targetDisplacement,
        Fixed64 targetRadius,
        out Fixed64 normalizedTime,
        out Vector2d normalForSource,
        out Fixed64 closingSpeed)
    {
        normalizedTime = Fixed64.Zero;
        normalForSource = Vector2d.Zero;
        closingSpeed = Fixed64.Zero;

        Vector2d relativeDisplacement = ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            sourceDisplacement,
            targetDisplacement,
            out _);
        Vector2d sourceEnd = GetSweepEnd(sourceStart, sourceDisplacement);
        Vector2d targetEnd = GetSweepEnd(targetStart, targetDisplacement);
        if (relativeDisplacement.MagnitudeSquared <= Fixed64.Epsilon
            || !HasUsableCombinedRadius(sourceRadius, targetRadius))
        {
            return false;
        }

        if (!RadialSweepAdmission.TryIntersect(
                sourceStart,
                relativeDisplacement,
                Fixed64.One,
                targetStart,
                targetRadius,
                sourceRadius,
                sourceEnd,
                targetEnd,
                out Fixed64 time))
            return false;

        Vector2d sourceImpact = Vector2d.Lerp(sourceStart, sourceEnd, time);
        Vector2d targetImpact = Vector2d.Lerp(targetStart, targetEnd, time);
        normalForSource = ResolveNormal(targetImpact, sourceImpact, relativeDisplacement);
        closingSpeed = -Vector2d.Dot(relativeDisplacement, normalForSource);
        if (closingSpeed <= Fixed64.Epsilon)
            return false;

        normalizedTime = time;
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
