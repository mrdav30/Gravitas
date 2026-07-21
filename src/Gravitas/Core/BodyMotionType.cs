//=======================================================================
// BodyMotionType.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Selects whether a body is solver controlled, host controlled, or immobile.
/// </summary>
public enum BodyMotionType : byte
{
    /// <summary>
    /// The solver controls the body subject to its frozen degrees of freedom.
    /// </summary>
    Dynamic = 0,

    /// <summary>
    /// The host controls the body while the solver treats it as infinite mass.
    /// </summary>
    Kinematic = 1,

    /// <summary>
    /// The body is excluded from per-frame motion and treated as infinite mass.
    /// </summary>
    Static = 2
}

internal static class BodyMotionTypeSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(this BodyMotionType motionType) =>
        (byte)motionType <= (byte)BodyMotionType.Static;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfInvalid(this BodyMotionType motionType, string parameterName)
    {
        SwiftThrowHelper.ThrowIfArgumentOutOfRange(
            !motionType.IsValid(),
            (int)motionType,
            parameterName,
            "Body motion type must be Dynamic, Kinematic, or Static.");
    }
}
