//=======================================================================
// BodyMotionTypeSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas;

internal static class BodyMotionTypeSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(this BodyMotionType motionType) =>
        (byte)motionType <= (byte)BodyMotionType.Static;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfInvalid(this BodyMotionType motionType, string parameterName)
    {
        ChronicleGuard.ThrowIfArgumentOutOfRange(
            !motionType.IsValid(),
            (int)motionType,
            parameterName,
            "Body motion type must be Dynamic, Kinematic, or Static.");
    }
}
