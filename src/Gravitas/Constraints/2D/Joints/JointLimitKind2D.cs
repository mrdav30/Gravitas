//=======================================================================
// JointLimitKind2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Identifies the active deterministic pure 2D joint limit payload.
/// </summary>
public enum JointLimitKind2D : byte
{
    /// <summary>
    /// No additional scalar limit is active.
    /// </summary>
    Unrestricted = 0,

    /// <summary>
    /// A distance joint target length.
    /// </summary>
    Distance = 1,

    /// <summary>
    /// A prismatic slider translation range.
    /// </summary>
    Slider = 2,

    /// <summary>
    /// A scalar relative-angle range.
    /// </summary>
    Angular = 3
}
