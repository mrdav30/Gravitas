//=======================================================================
// CompoundInertiaTensor.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>Stores an aggregate's unmaterialized weighted tensor about its requested reference.</summary>
// Full-domain shape weights fit 256 bits and semantic displacements fit 162.
// Squared shifts, factor sixty and an int-bounded part sum stay below 650 bits;
// applying full-domain mass fits 704. Intrinsic/rotation terms fit 576 bits.
internal struct CompoundInertiaTensor
{
    internal Signed704 Xx, Yy, Zz, Xy, Xz, Yz;

    internal Fixed3x3 Materialize(Fixed64 mass, ExactMassWeight weight)
    {
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            CompoundInertiaMath3D.ScaleWeight(weight), CompoundInertiaMath3D.Q192);
        bool representable = TryMoment(Xx, mass, denominator, out Fixed64 xx)
            & TryMoment(Yy, mass, denominator, out Fixed64 yy)
            & TryMoment(Zz, mass, denominator, out Fixed64 zz)
            & TryMoment(Xy, mass, denominator, out Fixed64 xy)
            & TryMoment(Xz, mass, denominator, out Fixed64 xz)
            & TryMoment(Yz, mass, denominator, out Fixed64 yz);
        if (!representable)
            throw new InvalidOperationException("The compound collider's inertia tensor is outside the Fixed64 scalar domain.");
        return new Fixed3x3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    }

    private static bool TryMoment(Signed704 geometry, Fixed64 mass, Signed576 denominator, out Fixed64 moment) =>
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned704ToSigned832(geometry, Signed192.Raw(mass)),
            Signed832.ExtendValue(denominator), numeratorLeftShift: 0, out moment);

    internal Fixed3x3 MaterializeSaturated(Fixed64 mass, ExactMassWeight weight)
    {
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            CompoundInertiaMath3D.ScaleWeight(weight), CompoundInertiaMath3D.Q192);
        Fixed64 xx = SaturatedMoment(Xx, mass, denominator), yy = SaturatedMoment(Yy, mass, denominator);
        Fixed64 zz = SaturatedMoment(Zz, mass, denominator), xy = SaturatedMoment(Xy, mass, denominator);
        Fixed64 xz = SaturatedMoment(Xz, mass, denominator), yz = SaturatedMoment(Yz, mass, denominator);
        return new Fixed3x3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    }

    private static Fixed64 SaturatedMoment(Signed704 geometry, Fixed64 mass, Signed576 denominator) =>
        TryMoment(geometry, mass, denominator, out Fixed64 moment) ? moment
            : (geometry.Sign < 0) != (mass < Fixed64.Zero) ? Fixed64.MinValue : Fixed64.MaxValue;
}
