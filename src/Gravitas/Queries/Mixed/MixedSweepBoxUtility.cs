//=======================================================================
// MixedSweepBoxUtility.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Provides deterministic swept-segment clipping against axis-aligned 3D bounds.
/// </summary>
internal static class MixedSweepBoxUtility
{
    internal static bool TrySweepBox(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Vector3d min,
        Vector3d max,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (IsInsideBox(start, min, max))
            return true;

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = length;
        if (!ClipSegmentAxis(start.X, direction.X, min.X, max.X, ref entry, ref exit)
            || !ClipSegmentAxis(start.Y, direction.Y, min.Y, max.Y, ref entry, ref exit)
            || !ClipSegmentAxis(start.Z, direction.Z, min.Z, max.Z, ref entry, ref exit))
        {
            return false;
        }

        distance = entry;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsInsideBox(Vector3d point, Vector3d min, Vector3d max) =>
        point.X >= min.X && point.X <= max.X
        && point.Y >= min.Y && point.Y <= max.Y
        && point.Z >= min.Z && point.Z <= max.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ClipSegmentAxis(
        Fixed64 position,
        Fixed64 direction,
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        if (direction.Abs() <= Fixed64.Epsilon)
            return position >= min && position <= max;

        Fixed64 first = (min - position) / direction;
        Fixed64 second = (max - position) / direction;
        if (first > second)
            (first, second) = (second, first);

        if (first > entry)
            entry = first;
        if (second < exit)
            exit = second;
        return entry <= exit;
    }
}
