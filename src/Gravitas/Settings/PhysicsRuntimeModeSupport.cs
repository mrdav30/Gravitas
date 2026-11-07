//=======================================================================
// PhysicsRuntimeModeSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas;

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
