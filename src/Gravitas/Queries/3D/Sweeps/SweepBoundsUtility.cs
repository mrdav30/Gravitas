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
    internal static bool TryTransformOrientedBoxSegmentToLocal(
        Vector3d worldStart,
        Vector3d worldEnd,
        Vector3d boxCenter,
        FixedQuaternion rotation,
        FixedQuaternion inverseRotation,
        out Vector3d localStart,
        out Vector3d localEnd)
    {
        localStart = Vector3d.Zero;
        localEnd = Vector3d.Zero;
        return Vector3d.TrySubtract(worldStart, boxCenter, out Vector3d startOffset)
            && Vector3d.TrySubtract(worldEnd, boxCenter, out Vector3d endOffset)
            && TryTransformOrientedBoxOffsetToLocal(startOffset, rotation, inverseRotation, out localStart)
            && TryTransformOrientedBoxOffsetToLocal(endOffset, rotation, inverseRotation, out localEnd);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryTransformOrientedBoxOffsetToLocal(
        Vector3d worldOffset,
        FixedQuaternion rotation,
        FixedQuaternion inverseRotation,
        out Vector3d localOffset)
    {
        localOffset = inverseRotation * worldOffset;

        // One deterministic refinement step corrects Q32.32 matrix rounding
        // without widening the represented box.
        Vector3d residual = worldOffset - rotation * localOffset;
        return residual == Vector3d.Zero
            || Vector3d.TryAdd(localOffset, inverseRotation * residual, out localOffset);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryClipSegment(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Vector3d min,
        Vector3d max,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        entry = Fixed64.Zero;
        exit = length;
        bool overlaps = ClipSegmentAxis(start.X, direction.X, min.X, max.X, ref entry, ref exit)
            && ClipSegmentAxis(start.Y, direction.Y, min.Y, max.Y, ref entry, ref exit)
            && ClipSegmentAxis(start.Z, direction.Z, min.Z, max.Z, ref entry, ref exit);
        if (!overlaps)
            return false;

        if (entry > exit)
            entry = exit = FixedMath.Midpoint(entry, exit);

        return true;
    }

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ClipSegmentAxis(
        Fixed64 position,
        Fixed64 direction,
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        if (direction == Fixed64.Zero)
            return position >= min && position <= max;

        Fixed64 first = (min - position) / direction;
        Fixed64 second = (max - position) / direction;
        if (first > second)
            (first, second) = (second, first);

        if (first > entry)
            entry = first;
        if (second < exit)
            exit = second;
        return entry <= exit || entry - exit <= Fixed64.Epsilon;
    }
}
