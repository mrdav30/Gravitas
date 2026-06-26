//=======================================================================
// SweepBoundsUtility.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal static class SweepBoundsUtility
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CreateSweptBounds(
        Vector3d min,
        Vector3d max,
        Vector3d displacement,
        Fixed64 padding,
        out Vector3d sweptMin,
        out Vector3d sweptMax)
    {
        Vector3d endMin = min + displacement;
        Vector3d endMax = max + displacement;
        Vector3d extents = Vector3d.One * padding;
        sweptMin = Vector3d.Min(min, endMin) - extents;
        sweptMax = Vector3d.Max(max, endMax) + extents;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CreateSweptSphereBounds(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Fixed64 padding,
        out Vector3d sweptMin,
        out Vector3d sweptMax)
    {
        Vector3d extents = Vector3d.One * (radius + padding);
        sweptMin = Vector3d.Min(start, end) - extents;
        sweptMax = Vector3d.Max(start, end) + extents;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool OverlapsInclusive(
        Vector3d firstMin,
        Vector3d firstMax,
        Vector3d secondMin,
        Vector3d secondMax)
    {
        return firstMax.X >= secondMin.X
            && firstMin.X <= secondMax.X
            && firstMax.Y >= secondMin.Y
            && firstMin.Y <= secondMax.Y
            && firstMax.Z >= secondMin.Z
            && firstMin.Z <= secondMax.Z;
    }
}
