//=======================================================================
// PhysicsRuntimeMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Selects which dimensional runtime path a context advances.
/// </summary>
[Flags]
public enum PhysicsRuntimeMode : byte
{
    /// <summary>
    /// No physics runtime path. This is not a valid settings value.
    /// </summary>
    None = 0,

    /// <summary>
    /// Advance only the pure 2D runtime path.
    /// </summary>
    TwoD = 1 << 0,

    /// <summary>
    /// Advance only the 3D runtime path.
    /// </summary>
    ThreeD = 1 << 1,

    /// <summary>
    /// Advance pure 2D and pure 3D runtime paths without cross-dimensional contacts.
    /// </summary>
    Both = TwoD | ThreeD,

    /// <summary>
    /// Advance pure 2D, pure 3D, and explicit mixed 2D/3D collision paths.
    /// </summary>
    Mixed = Both | (1 << 2)
}

internal static class PhysicsRuntimeModeSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(this PhysicsRuntimeMode mode)
    {
        return mode == PhysicsRuntimeMode.TwoD
            || mode == PhysicsRuntimeMode.ThreeD
            || mode == PhysicsRuntimeMode.Both
            || mode == PhysicsRuntimeMode.Mixed;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Runs2D(this PhysicsRuntimeMode mode)
    {
        return (mode & PhysicsRuntimeMode.TwoD) != 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Runs3D(this PhysicsRuntimeMode mode)
    {
        return (mode & PhysicsRuntimeMode.ThreeD) != 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool RunsMixedContacts(this PhysicsRuntimeMode mode)
    {
        return mode == PhysicsRuntimeMode.Mixed;
    }
}
