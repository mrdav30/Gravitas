using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionMath
{
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
        if (time < Fixed64.Zero || time > Fixed64.One)
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
        if (time < Fixed64.Zero || time > Fixed64.One)
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
        if (delta.MagnitudeSquared > Fixed64.Epsilon)
            return delta.Normalized;

        return relativeDisplacement.MagnitudeSquared > Fixed64.Epsilon
            ? -relativeDisplacement.Normalized
            : Vector3d.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveNormal(Vector2d delta, Vector2d relativeDisplacement)
    {
        if (delta.MagnitudeSquared > Fixed64.Epsilon)
            return delta.Normalized;

        return relativeDisplacement.MagnitudeSquared > Fixed64.Epsilon
            ? -relativeDisplacement.Normalized
            : Vector2d.Zero;
    }
}
