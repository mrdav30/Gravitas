//=======================================================================
// GravitasReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;

namespace Gravitas;

/// <summary>
/// Fixed-width deterministic hash of a Gravitas authoritative replay state.
/// </summary>
public readonly struct GravitasReplayHash : IEquatable<GravitasReplayHash>
{
    public GravitasReplayHash(ulong low, ulong high)
    {
        Low = low;
        High = high;
    }

    /// <summary>
    /// Gets the low 64-bit lane.
    /// </summary>
    public ulong Low { get; }

    /// <summary>
    /// Gets the high 64-bit lane.
    /// </summary>
    public ulong High { get; }

    public bool Equals(GravitasReplayHash other) => Low == other.Low && High == other.High;

    public override bool Equals(object? obj) => obj is GravitasReplayHash other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            ulong mixed = Low ^ (High * 0x9e3779b97f4a7c15UL);
            mixed ^= mixed >> 33;
            mixed *= 0xff51afd7ed558ccdUL;
            mixed ^= mixed >> 33;
            return (int)(mixed ^ (mixed >> 32));
        }
    }

    public override string ToString() =>
        High.ToString("x16", CultureInfo.InvariantCulture)
        + Low.ToString("x16", CultureInfo.InvariantCulture);

    public static bool operator ==(GravitasReplayHash left, GravitasReplayHash right) => left.Equals(right);

    public static bool operator !=(GravitasReplayHash left, GravitasReplayHash right) => !left.Equals(right);
}
