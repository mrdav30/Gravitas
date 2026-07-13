//=======================================================================
// ContinuousCollisionMath.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionMath
{
    private const int MaxRotationalSubsteps = 16;
    public const int RotationalToiRefinementIterations = 12;
    private static readonly Fixed64 MaxRotationalStepRadians = FixedMath.DegToRad((Fixed64)15);

    public static int ResolveRotationalSubstepCount(Fixed64 angularDisplacement)
    {
        Fixed64 angularDistance = angularDisplacement.Abs();
        if (angularDistance <= Fixed64.Epsilon)
            return 0;

        int steps = 1;
        Fixed64 covered = MaxRotationalStepRadians;
        while (covered < angularDistance && steps < MaxRotationalSubsteps)
        {
            covered += MaxRotationalStepRadians;
            steps++;
        }

        return steps;
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
        Vector3d delta = sourceStart - targetStart;
        Vector3d relativeDisplacement = sourceDisplacement - targetDisplacement;
        Fixed64 combinedRadius = sourceRadius + targetRadius;
        return TrySweepRelative(
            delta,
            relativeDisplacement,
            combinedRadius,
            out normalizedTime,
            out normalForSource,
            out closingSpeed);
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
        Vector2d delta = sourceStart - targetStart;
        Vector2d relativeDisplacement = sourceDisplacement - targetDisplacement;
        Fixed64 combinedRadius = sourceRadius + targetRadius;
        return TrySweepRelative(
            delta,
            relativeDisplacement,
            combinedRadius,
            out normalizedTime,
            out normalForSource,
            out closingSpeed);
    }

    private static bool TrySweepRelative(
        Vector3d delta,
        Vector3d relativeDisplacement,
        Fixed64 combinedRadius,
        out Fixed64 normalizedTime,
        out Vector3d normalForSource,
        out Fixed64 closingSpeed)
    {
        normalizedTime = Fixed64.Zero;
        normalForSource = Vector3d.Zero;
        closingSpeed = Fixed64.Zero;

        Fixed64 relativeMagnitudeSquared = relativeDisplacement.MagnitudeSquared;
        if (relativeMagnitudeSquared <= Fixed64.Epsilon || combinedRadius <= Fixed64.Epsilon)
            return false;

        Fixed64 combinedRadiusSquared = combinedRadius * combinedRadius;
        Fixed64 c = delta.MagnitudeSquared - combinedRadiusSquared;
        if (c <= Fixed64.Zero)
        {
            normalForSource = ResolveNormal(delta, relativeDisplacement);
            closingSpeed = -Vector3d.Dot(relativeDisplacement, normalForSource);
            return closingSpeed > Fixed64.Epsilon;
        }

        Fixed64 b = (Fixed64)2 * Vector3d.Dot(delta, relativeDisplacement);
        if (b >= Fixed64.Zero)
            return false;

        Fixed64 a = relativeMagnitudeSquared;
        Fixed64 discriminant = b * b - (Fixed64)4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 time = (-b - FixedMath.Sqrt(discriminant)) / ((Fixed64)2 * a);
        if (time > Fixed64.One)
            return false;

        Vector3d impactDelta = delta + relativeDisplacement * time;
        normalForSource = ResolveNormal(impactDelta, relativeDisplacement);
        closingSpeed = -Vector3d.Dot(relativeDisplacement, normalForSource);
        if (closingSpeed <= Fixed64.Epsilon)
            return false;

        normalizedTime = time;
        return true;
    }

    private static bool TrySweepRelative(
        Vector2d delta,
        Vector2d relativeDisplacement,
        Fixed64 combinedRadius,
        out Fixed64 normalizedTime,
        out Vector2d normalForSource,
        out Fixed64 closingSpeed)
    {
        normalizedTime = Fixed64.Zero;
        normalForSource = Vector2d.Zero;
        closingSpeed = Fixed64.Zero;

        Fixed64 relativeMagnitudeSquared = relativeDisplacement.MagnitudeSquared;
        if (relativeMagnitudeSquared <= Fixed64.Epsilon || combinedRadius <= Fixed64.Epsilon)
            return false;

        Fixed64 combinedRadiusSquared = combinedRadius * combinedRadius;
        Fixed64 c = delta.MagnitudeSquared - combinedRadiusSquared;
        if (c <= Fixed64.Zero)
        {
            normalForSource = ResolveNormal(delta, relativeDisplacement);
            closingSpeed = -Vector2d.Dot(relativeDisplacement, normalForSource);
            return closingSpeed > Fixed64.Epsilon;
        }

        Fixed64 b = (Fixed64)2 * Vector2d.Dot(delta, relativeDisplacement);
        if (b >= Fixed64.Zero)
            return false;

        Fixed64 a = relativeMagnitudeSquared;
        Fixed64 discriminant = b * b - (Fixed64)4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 time = (-b - FixedMath.Sqrt(discriminant)) / ((Fixed64)2 * a);
        if (time > Fixed64.One)
            return false;

        Vector2d impactDelta = delta + relativeDisplacement * time;
        normalForSource = ResolveNormal(impactDelta, relativeDisplacement);
        closingSpeed = -Vector2d.Dot(relativeDisplacement, normalForSource);
        if (closingSpeed <= Fixed64.Epsilon)
            return false;

        normalizedTime = time;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveNormal(Vector3d delta, Vector3d relativeDisplacement)
    {
        if (delta != Vector3d.Zero)
        {
            Fixed64 scale = FixedMath.Max(
                delta.X.Abs(),
                FixedMath.Max(delta.Y.Abs(), delta.Z.Abs()));
            return new Vector3d(delta.X / scale, delta.Y / scale, delta.Z / scale).Normalized;
        }

        return -relativeDisplacement.Normalized;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveNormal(Vector2d delta, Vector2d relativeDisplacement)
    {
        if (delta != Vector2d.Zero)
        {
            Fixed64 scale = FixedMath.Max(delta.X.Abs(), delta.Y.Abs());
            return new Vector2d(delta.X / scale, delta.Y / scale).Normalized;
        }

        return -relativeDisplacement.Normalized;
    }
}
