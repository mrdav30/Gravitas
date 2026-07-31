//=======================================================================
// RadialSweepAdmission.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Owns exact radial sweep roots plus Gravitas's inclusive frame-end contract.
/// </summary>
internal static class RadialSweepAdmission
{
    internal static bool TryIntersect(
        Vector2d rayStart,
        Vector2d rayDirection,
        Fixed64 maxParameter,
        Vector2d targetCenter,
        Fixed64 targetRadius,
        Fixed64 radiusExpansion,
        Vector2d sourceEndpoint,
        Vector2d targetEndpoint,
        out Fixed64 parameter)
    {
        Fixed64? exact = new FixedRay2d(rayStart, rayDirection).Intersects(
            new FixedBoundCircle(targetCenter, targetRadius),
            radiusExpansion,
            maxParameter);
        if (exact.HasValue)
        {
            parameter = exact.Value;
            return true;
        }

        if (maxParameter < Fixed64.Zero
            || !IsRepresentableEndpointContact(sourceEndpoint, targetEndpoint, targetRadius, radiusExpansion))
        {
            parameter = default;
            return false;
        }

        parameter = maxParameter;
        return true;
    }

    private static bool IsRepresentableEndpointContact(
        Vector2d sourceEndpoint,
        Vector2d targetEndpoint,
        Fixed64 targetRadius,
        Fixed64 radiusExpansion)
    {
        if (!Vector2d.TryGetDistance(sourceEndpoint, targetEndpoint, out Fixed64 endpointDistance))
            return false;

        targetRadius = targetRadius.Abs();
        return !Fixed64.TryAdd(targetRadius, radiusExpansion, out Fixed64 combinedRadius)
            || endpointDistance <= combinedRadius;
    }

}
