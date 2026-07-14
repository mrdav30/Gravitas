//=======================================================================
// FixedVectorDifference.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Support;

/// <summary>
/// Forms fixed-point vector differences while rejecting component saturation.
/// </summary>
internal static class FixedVectorDifference
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryCreate(Vector2d start, Vector2d end, out Vector2d difference)
    {
        difference = end - start;
        return start + difference == end;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryCreate(Vector3d start, Vector3d end, out Vector3d difference)
    {
        difference = end - start;
        return start + difference == end;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryTranslate(Vector2d start, Vector2d translation, out Vector2d end)
    {
        end = start + translation;
        return end - start == translation;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryTranslate(Vector3d start, Vector3d translation, out Vector3d end)
    {
        end = start + translation;
        return end - start == translation;
    }
}
