//=======================================================================
// PlanarRotation.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal static class PlanarRotation
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 Canonicalize(Fixed64 rotation)
    {
        if (rotation >= -Fixed64.Pi && rotation < Fixed64.Pi)
            return rotation;

        rotation %= Fixed64.TwoPi;
        if (rotation >= Fixed64.Pi)
            return rotation - Fixed64.TwoPi;
        if (rotation < -Fixed64.Pi)
            return rotation + Fixed64.TwoPi;
        return rotation;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 Combine(Fixed64 first, Fixed64 second) =>
        Canonicalize(Canonicalize(first) + Canonicalize(second));
}
