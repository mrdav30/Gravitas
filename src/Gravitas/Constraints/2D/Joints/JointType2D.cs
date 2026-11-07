//=======================================================================
// JointType2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Identifies the deterministic pure 2D joint model solved by Gravitas.
/// </summary>
public enum JointType2D : byte
{
    /// <summary>
    /// Keeps two anchors at a configured distance while allowing free relative yaw.
    /// </summary>
    Distance = 1,

    /// <summary>
    /// Keeps two anchors coincident while allowing free relative yaw.
    /// </summary>
    Pin = 2,

    /// <summary>
    /// Keeps two anchors coincident and constrains scalar relative yaw.
    /// </summary>
    Weld = 3,

    /// <summary>
    /// Keeps one link on a local slider axis with optional scalar translation limits.
    /// </summary>
    Prismatic = 4
}
