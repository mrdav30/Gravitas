//=======================================================================
// CollisionPositionCorrection.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Diagnostics;

namespace Gravitas.CollisionHandling;

/// <summary>Preserves compact correction rounding without overflowing a shared mass scale.</summary>
internal static class CollisionPositionCorrection
{
    internal static bool TryGetCompactScale(
        Fixed64 depth,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        out Fixed64 scale)
    {
        // Callers provide nonnegative inverse masses with a positive total.
        if (!Fixed64.TryAdd(inverseMassA, inverseMassB, out Fixed64 totalInverseMass)
            || Fixed64.CompareProducts(depth, Fixed64.One, totalInverseMass, Fixed64.MaxValue) > 0)
        {
            scale = default;
            return false;
        }

        scale = depth / totalInverseMass;
        // Rounded common scales must also fit when multiplied back by mass.
        return Fixed64.CompareProducts(scale, totalInverseMass, Fixed64.MaxValue, Fixed64.One) <= 0;
    }

    internal static Fixed64 GetWeightedDistance(
        Fixed64 depth,
        Fixed64 inverseMass,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB)
    {
        // A nonnegative mass fraction cannot exceed one, so the weighted depth fits
        // even when the shared scale or the sum of inverse masses does not.
        bool fits = Fixed64.TryMultiplyDivideBySum(
            depth, inverseMass, Fixed64.One, Fixed64.One,
            inverseMassA, inverseMassB, Fixed64.Zero, Fixed64.Zero,
            out Fixed64 distance);
        Debug.Assert(fits);
        return distance;
    }
}
