//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Retains oversized impulses for one frozen contact row during one solver step.
/// Normal numerators share the row's fixed denominator. Tangents retain rounded
/// Q32.32 integer magnitudes, so repeated disk projections cannot grow radicals.
/// </summary>
internal sealed class ExactContactImpulseState
{
    internal const int Words = 48;
    private ulong[]? _normal;
    private ulong[]? _tangents;
    internal bool HasNormal { get; private set; }
    internal bool HasTangents { get; private set; }
    internal int PrimarySign { get; private set; }
    internal int SecondarySign { get; private set; }
    internal ReadOnlySpan<ulong> NormalNumerator => HasNormal ? _normal!.AsSpan(0, Words) : ReadOnlySpan<ulong>.Empty;
    internal ReadOnlySpan<ulong> NormalDenominator => HasNormal ? _normal!.AsSpan(Words, Words) : ReadOnlySpan<ulong>.Empty;
    internal ReadOnlySpan<ulong> PrimaryMagnitude => HasTangents ? _tangents!.AsSpan(0, Words) : ReadOnlySpan<ulong>.Empty;
    internal ReadOnlySpan<ulong> SecondaryMagnitude => HasTangents ? _tangents!.AsSpan(Words, Words) : ReadOnlySpan<ulong>.Empty;

    internal void Clear()
    {
        HasNormal = HasTangents = false;
        PrimarySign = SecondarySign = 0;
    }

    internal void SetNormal(ReadOnlySpan<ulong> numerator, ReadOnlySpan<ulong> denominator)
    {
        _normal ??= new ulong[Words * 2];
        numerator.CopyTo(_normal.AsSpan(0, Words));
        denominator.CopyTo(_normal.AsSpan(Words, Words));
        HasNormal = true;
    }

    internal void SetTangents(ReadOnlySpan<ulong> primary, int primarySign,
        ReadOnlySpan<ulong> secondary, int secondarySign)
    {
        _tangents ??= new ulong[Words * 2];
        primary.CopyTo(_tangents.AsSpan(0, Words));
        secondary.CopyTo(_tangents.AsSpan(Words, Words));
        PrimarySign = primarySign;
        SecondarySign = secondarySign;
        HasTangents = true;
    }
}
