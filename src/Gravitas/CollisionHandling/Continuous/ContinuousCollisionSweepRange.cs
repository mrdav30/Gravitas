//=======================================================================
// ContinuousCollisionSweepRange.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Validates the scalar-distance contract required by continuous sweep queries.
/// </summary>
internal static class ContinuousCollisionSweepRange
{
    private const string RangeMessage =
        "Continuous collision motion must have a component-exact difference and a representable Euclidean length.";

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ValidateEndpoint(Vector2d start, Vector2d end, out Fixed64 length)
    {
        if (!Vector2d.TrySubtract(end, start, out Vector2d displacement)
            || !Vector2d.TryGetMagnitude(displacement, out length))
        {
            throw new ArgumentOutOfRangeException(nameof(end), RangeMessage);
        }

        return displacement;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ValidateEndpoint(Vector3d start, Vector3d end, out Fixed64 length)
    {
        if (!Vector3d.TrySubtract(end, start, out Vector3d displacement)
            || !Vector3d.TryGetMagnitude(displacement, out length))
        {
            throw new ArgumentOutOfRangeException(nameof(end), RangeMessage);
        }

        return displacement;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ValidateEndpoint(
        Vector2d start,
        Vector2d end,
        Vector2d requestedDisplacement,
        out Fixed64 length)
    {
        Vector2d displacement = ValidateEndpoint(start, end, out length);
        if (displacement != requestedDisplacement)
            throw new ArgumentOutOfRangeException(nameof(end), RangeMessage);

        return displacement;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ValidateEndpoint(
        Vector3d start,
        Vector3d end,
        Vector3d requestedDisplacement,
        out Fixed64 length)
    {
        Vector3d displacement = ValidateEndpoint(start, end, out length);
        if (displacement != requestedDisplacement)
            throw new ArgumentOutOfRangeException(nameof(end), RangeMessage);

        return displacement;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ValidateRelativeDisplacement(
        Vector2d sourceDisplacement,
        Vector2d targetDisplacement,
        out Fixed64 length)
    {
        if (!Vector2d.TrySubtract(sourceDisplacement, targetDisplacement, out Vector2d relativeDisplacement)
            || !Vector2d.TryGetMagnitude(relativeDisplacement, out length))
        {
            throw new ArgumentOutOfRangeException(nameof(sourceDisplacement), RangeMessage);
        }

        return relativeDisplacement;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ValidateRelativeDisplacement(
        Vector3d sourceDisplacement,
        Vector3d targetDisplacement,
        out Fixed64 length)
    {
        if (!Vector3d.TrySubtract(sourceDisplacement, targetDisplacement, out Vector3d relativeDisplacement)
            || !Vector3d.TryGetMagnitude(relativeDisplacement, out length))
        {
            throw new ArgumentOutOfRangeException(nameof(sourceDisplacement), RangeMessage);
        }

        return relativeDisplacement;
    }
}
