//=======================================================================
// JointLimitKind3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Identifies optional angular limit data for a 3D joint.
/// </summary>
public enum JointLimitKind3D : byte
{
    /// <summary>
    /// No angular limit is applied beyond the joint type's base rows.
    /// </summary>
    Unrestricted = 0,

    /// <summary>
    /// Limits hinge twist around the local X axis.
    /// </summary>
    Hinge = 1,

    /// <summary>
    /// Limits swing around the local Z axis and twist around that same axis.
    /// </summary>
    ConeTwist = 2
}
