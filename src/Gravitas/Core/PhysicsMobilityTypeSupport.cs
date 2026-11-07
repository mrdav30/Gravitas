//=======================================================================
// BodyMotionTypeSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Validates body mobility values from configuration and motion updates.
/// </summary>
internal static class PhysicsMobilityTypeSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(this PhysicsMobilityType motionType) =>
        (byte)motionType <= (byte)PhysicsMobilityType.Static;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfInvalid(this PhysicsMobilityType motionType, string parameterName)
    {
        ChronicleGuard.ThrowIfArgumentOutOfRange(
            !motionType.IsValid(),
            (int)motionType,
            parameterName,
            "Body motion type must be Dynamic, Kinematic, or Static.");
    }
}
