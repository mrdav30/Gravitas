//=======================================================================
// PlanarSegmentGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Shared deterministic helpers for finite 2D line segments used by collision and query code.
/// </summary>
internal static class PlanarSegmentGeometry
{
    /// <summary>
    /// Finds the closest point on a finite segment, treating near-zero segments as collapsed.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ClosestPoint(Vector2d point, Vector2d start, Vector2d end)
    {
        Vector2d segment = end - start;
        Fixed64 lengthSquared = segment.MagnitudeSquared;
        if (lengthSquared <= Fixed64.Epsilon)
            return start;

        Fixed64 t = Vector2d.Dot(point - start, segment) / lengthSquared;
        if (t <= Fixed64.Zero)
            return start;

        if (t >= Fixed64.One)
            return end;

        return start + segment * t;
    }

    /// <summary>
    /// Computes the squared distance from a point to a finite segment.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(Vector2d point, Vector2d start, Vector2d end) =>
        Vector2d.DistanceSquared(point, ClosestPoint(point, start, end));
}
