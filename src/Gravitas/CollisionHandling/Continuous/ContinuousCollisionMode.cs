//=======================================================================
// ContinuousCollisionMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Selects how a body guards its frame movement against tunneling.
/// </summary>
public enum ContinuousCollisionMode : byte
{
    /// <summary>
    /// Uses the owning context's default continuous-collision mode.
    /// </summary>
    Inherit = 0,

    /// <summary>
    /// Uses the existing discrete integration path without a movement sweep.
    /// </summary>
    Discrete = 1,

    /// <summary>
    /// Sweeps the body through its intended frame displacement before committing position.
    /// </summary>
    Continuous = 2,

    /// <summary>
    /// Sweeps only when the intended frame displacement is larger than the body proxy radius.
    /// </summary>
    Auto = 3
}

internal static class ContinuousCollisionModeSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(this ContinuousCollisionMode mode) =>
        (byte)mode <= (byte)ContinuousCollisionMode.Auto;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfInvalid(this ContinuousCollisionMode mode, string parameterName)
    {
        SwiftThrowHelper.ThrowIfArgumentOutOfRange(
            !mode.IsValid(),
            (int)mode,
            parameterName,
            "Continuous collision mode must be Inherit, Discrete, Continuous, or Auto.");
    }
}
