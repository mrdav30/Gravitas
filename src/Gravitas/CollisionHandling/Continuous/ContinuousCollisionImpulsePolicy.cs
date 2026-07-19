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
    internal static bool TryResolveVelocityDelta(
        Vector3d signedNormal,
        Fixed64 normalVelocity,
        Fixed64 responseFactor,
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass,
        out Vector3d velocityDelta)
    {
        if (bodyInverseMass == Fixed64.Zero)
        {
            velocityDelta = default;
            return true;
        }

        bool resolved = Fixed64.TryMultiplyDivide(
            normalVelocity,
            responseFactor,
            bodyInverseMass,
            constrainedInverseMass,
            out Fixed64 speedDelta);
        velocityDelta = signedNormal * speedDelta;
        return resolved;
    }

    internal static bool TryResolveVelocityDelta(
        Vector2d signedNormal,
        Fixed64 normalVelocity,
        Fixed64 responseFactor,
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass,
        out Vector2d velocityDelta)
    {
        if (bodyInverseMass == Fixed64.Zero)
        {
            velocityDelta = default;
            return true;
        }

        bool resolved = Fixed64.TryMultiplyDivide(
            normalVelocity,
            responseFactor,
            bodyInverseMass,
            constrainedInverseMass,
            out Fixed64 speedDelta);
        velocityDelta = signedNormal * speedDelta;
        return resolved;
    }

    internal static bool TryResolveVelocityDelta(
        Vector3d normal,
        Fixed64 responseSpeed,
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass,
        out Vector3d velocityDelta)
    {
        if (bodyInverseMass == Fixed64.Zero)
        {
            velocityDelta = default;
            return true;
        }

        if (!Fixed64.TryMultiplyDivide(normal.X, responseSpeed, bodyInverseMass, constrainedInverseMass, out Fixed64 x)
            || !Fixed64.TryMultiplyDivide(normal.Y, responseSpeed, bodyInverseMass, constrainedInverseMass, out Fixed64 y)
            || !Fixed64.TryMultiplyDivide(normal.Z, responseSpeed, bodyInverseMass, constrainedInverseMass, out Fixed64 z))
        {
            velocityDelta = default;
            return false;
        }

        velocityDelta = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryResolveVelocityDelta(
        Vector2d normal,
        Fixed64 responseSpeed,
        Fixed64 bodyInverseMass,
        Fixed64 constrainedInverseMass,
        out Vector2d velocityDelta)
    {
        if (bodyInverseMass == Fixed64.Zero)
        {
            velocityDelta = default;
            return true;
        }

        if (!Fixed64.TryMultiplyDivide(normal.X, responseSpeed, bodyInverseMass, constrainedInverseMass, out Fixed64 x)
            || !Fixed64.TryMultiplyDivide(normal.Y, responseSpeed, bodyInverseMass, constrainedInverseMass, out Fixed64 y))
        {
            velocityDelta = default;
            return false;
        }

        velocityDelta = new Vector2d(x, y);
        return true;
    }

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
