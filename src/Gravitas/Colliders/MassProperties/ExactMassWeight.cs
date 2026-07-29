//=======================================================================
// ExactMassWeight.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Represents a nonnegative relative mass-property measure without narrowing
/// products or aggregate sums to <see cref="Fixed64"/>.
/// </summary>
internal readonly struct ExactMassWeight
{
    internal readonly Signed320 Numerator;

    internal ExactMassWeight(Signed320 numerator)
    {
        Numerator = numerator;
    }

    /// <summary>Gets the zero relative weight.</summary>
    internal static ExactMassWeight Zero => default;

    /// <summary>Gets a unit relative weight.</summary>
    internal static ExactMassWeight One =>
        ExactMassProperties.CreateWeight(Fixed64.One);

    /// <summary>Gets whether this weight is exactly zero.</summary>
    internal bool IsZero => Numerator.IsZero;

    /// <summary>Creates a weight from one nonnegative measure.</summary>
    internal static ExactMassWeight FromMeasure(Fixed64 measure) =>
        ExactMassProperties.CreateWeight(measure);

    /// <summary>Creates a weight from an exact product of nonnegative factors.</summary>
    internal static ExactMassWeight FromProduct(
        Fixed64 first,
        Fixed64 second) =>
        ExactMassProperties.CreateWeight(first, second);

    /// <summary>Creates a weight from an exact product of nonnegative factors.</summary>
    internal static ExactMassWeight FromProduct(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third) =>
        ExactMassProperties.CreateWeight(first, second, third);

    /// <summary>Creates a weight from an exact product of nonnegative factors.</summary>
    internal static ExactMassWeight FromProduct(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 fourth) =>
        ExactMassProperties.CreateWeight(first, second, third, fourth);

    /// <summary>Adds another nonnegative weight without scalar narrowing.</summary>
    /// <exception cref="OverflowException">
    /// The exact aggregate exceeds the semantic weight domain.
    /// </exception>
    internal ExactMassWeight Add(ExactMassWeight other)
    {
        if (!TryAdd(other, out ExactMassWeight result))
        {
            throw new OverflowException(
                "The aggregate mass-property weight is outside the semantic weight domain.");
        }

        return result;
    }

    /// <summary>
    /// Attempts to add another nonnegative weight without scalar narrowing.
    /// </summary>
    internal bool TryAdd(
        ExactMassWeight other,
        out ExactMassWeight result) =>
        ExactMassProperties.TryAdd(this, other, out result);

    /// <summary>
    /// Attempts to materialize the represented measure as <see cref="Fixed64"/>.
    /// </summary>
    internal bool TryGetMeasure(out Fixed64 measure) =>
        ExactMassProperties.TryGetMeasure(this, out measure);

    /// <summary>
    /// Attempts to distribute <paramref name="total"/> by this weight's exact
    /// share of <paramref name="totalWeight"/>.
    /// </summary>
    internal bool TryGetProportionalShare(
        Fixed64 total,
        ExactMassWeight totalWeight,
        out Fixed64 share) =>
        ExactMassProperties.TryGetProportionalShare(
            this,
            total,
            totalWeight,
            out share);
}
