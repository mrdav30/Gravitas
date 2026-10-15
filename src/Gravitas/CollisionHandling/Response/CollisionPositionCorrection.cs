//=======================================================================
// CollisionPositionCorrection.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Diagnostics;

namespace Gravitas.CollisionHandling;

/// <summary>Retains compact correction rounding only when mass weighting cannot amplify it.</summary>
internal static class CollisionPositionCorrection
{
    internal static bool TryGetCompactScale(
        Fixed64 depth,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        out Fixed64 scale)
    {
        // Callers provide nonnegative inverse masses with a positive total.
        // Later mass multiplication must not amplify common-scale or normal rounding.
        if (!Fixed64.TryAdd(inverseMassA, inverseMassB, out Fixed64 totalInverseMass)
            || inverseMassA > Fixed64.One || inverseMassB > Fixed64.One
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
        // Callers select one of the two masses; these exact fractions need no wide division.
        if (inverseMass == Fixed64.Zero) return Fixed64.Zero;
        if (inverseMassA == Fixed64.Zero || inverseMassB == Fixed64.Zero) return depth;
        if (inverseMassA == inverseMassB) return depth * Fixed64.Half;

        // A nonnegative mass fraction cannot exceed one, so the weighted depth fits
        // even when the shared scale or the sum of inverse masses does not.
        bool fits = Fixed64.TryAdd(inverseMassA, inverseMassB, out Fixed64 totalInverseMass)
            ? Fixed64.TryMultiplyDivide(depth, inverseMass, totalInverseMass, out Fixed64 distance)
            : Fixed64.TryMultiplyDivideBySum(
                depth, inverseMass, Fixed64.One, Fixed64.One,
                inverseMassA, inverseMassB, Fixed64.Zero, Fixed64.Zero,
                out distance);
        Debug.Assert(fits);
        return distance;
    }
}
