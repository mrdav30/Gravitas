//=======================================================================
// BodyMotionType.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>
/// Selects whether a body is solver controlled, host controlled, or immobile.
/// </summary>
public enum PhysicsMobilityType : byte
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
