//=======================================================================
// ContinuousCollisionImpulsePolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionImpulsePolicy
{
    internal static bool IsResolvableMobility(
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass) =>
        bodyInverseMass <= Fixed64.Zero
        || (constrainedInverseMass > Fixed64.Zero
            && constrainedInverseMass / bodyInverseMass > Fixed64.Epsilon);

    internal static Vector3d ResolveVelocityDelta(
        Vector3d normal,
        Fixed64 responseSpeed,
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass) =>
        (normal * (bodyInverseMass / constrainedInverseMass)) * responseSpeed;

    internal static Vector2d ResolveVelocityDelta(
        Vector2d normal,
        Fixed64 responseSpeed,
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass) =>
        (normal * (bodyInverseMass / constrainedInverseMass)) * responseSpeed;

    internal static bool TryResolveSourceNormal(
        Vector3d normalForSource,
        Vector3d sourceDisplacement,
        out Vector3d normal)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normalForSource.Normalized;
            return true;
        }

        if (sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = -sourceDisplacement.Normalized;
            return true;
        }

        normal = Vector3d.Zero;
        return false;
    }

    internal static bool TryResolveSourceNormal(
        Vector2d normalForSource,
        Vector2d sourceDisplacement,
        out Vector2d normal)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normalForSource.Normalized;
            return true;
        }

        if (sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = -sourceDisplacement.Normalized;
            return true;
        }

        normal = Vector2d.Zero;
        return false;
    }

    internal static bool TryResolveImpactNormal(Vector3d normalForSource, out Vector3d normal)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normalForSource.Normalized;
            return true;
        }

        normal = Vector3d.Zero;
        return false;
    }

    internal static bool TryResolveImpactNormal(Vector2d normalForSource, out Vector2d normal)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normalForSource.Normalized;
            return true;
        }

        normal = Vector2d.Zero;
        return false;
    }
}
