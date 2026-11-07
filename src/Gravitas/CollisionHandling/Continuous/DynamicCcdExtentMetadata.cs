//=======================================================================
// DynamicCcdExtentMetadata.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;
using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal static class DynamicCcdExtentMetadata
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsEquivalent(
        Fixed64 previousMinX,
        Fixed64 previousMaxX,
        Fixed64 currentMinX,
        Fixed64 currentMaxX)
    {
        bool previousRepresentable = Fixed64.TrySubtract(previousMaxX, previousMinX, out Fixed64 previousExtentX);
        bool currentRepresentable = Fixed64.TrySubtract(currentMaxX, currentMinX, out Fixed64 currentExtentX);
        return previousRepresentable == currentRepresentable
            && (!previousRepresentable || previousExtentX == currentExtentX);
    }
}
