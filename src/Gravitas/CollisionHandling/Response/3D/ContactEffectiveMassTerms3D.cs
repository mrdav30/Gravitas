//=======================================================================
// ContactEffectiveMassTerms3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal readonly struct ContactEffectiveMassTerms3D
{
    internal ContactEffectiveMassTerms3D(
        Fixed64 linearA,
        Fixed64 linearB,
        Fixed64 angularA,
        Fixed64 angularB)
    {
        LinearA = linearA;
        LinearB = linearB;
        AngularA = angularA;
        AngularB = angularB;
    }

    internal Fixed64 LinearA { get; }

    internal Fixed64 LinearB { get; }

    internal Fixed64 AngularA { get; }

    internal Fixed64 AngularB { get; }

    internal Fixed64 SaturatedSum =>
        LinearA + LinearB + AngularA + AngularB;

    internal bool TryGetValue(out Fixed64 value)
    {
        bool resolved =
            Fixed64.TryAdd(LinearA, LinearB, out Fixed64 linear);
        resolved &= Fixed64.TryAdd(
            linear,
            AngularA,
            out Fixed64 first);
        resolved &= Fixed64.TryAdd(first, AngularB, out value);
        if (!resolved)
            value = default;
        return resolved;
    }
}
