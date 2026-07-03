//=======================================================================
// JointMotorKind2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Identifies the deterministic pure 2D motor family.
/// </summary>
public enum JointMotorKind2D : byte
{
    /// <summary>
    /// No motor row is emitted.
    /// </summary>
    Disabled = 0,

    /// <summary>
    /// Drives scalar relative yaw toward a target angle.
    /// </summary>
    Angular = 1,

    /// <summary>
    /// Drives prismatic translation along the joint axis toward a target offset.
    /// </summary>
    Linear = 2
}
