//=======================================================================
// ContinuousCollisionModeSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>Validates continuous collision modes at settings and body boundaries.</summary>
internal static class ContinuousCollisionModeSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(this ContinuousCollisionMode mode) =>
        (byte)mode <= (byte)ContinuousCollisionMode.Auto;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfInvalid(this ContinuousCollisionMode mode, string parameterName)
    {
        ChronicleGuard.ThrowIfArgumentOutOfRange(
            !mode.IsValid(),
            (int)mode,
            parameterName,
            "Continuous collision mode must be Inherit, Discrete, Continuous, or Auto.");
    }
}
