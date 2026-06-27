//=======================================================================
// JointType3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Identifies the deterministic 3D joint model solved by Gravitas.
/// </summary>
public enum JointType3D : byte
{
    /// <summary>
    /// Keeps two local anchor points together while allowing free relative rotation.
    /// </summary>
    BallSocket = 1,

    /// <summary>
    /// Keeps two anchors together and aligns the local X hinge axes.
    /// </summary>
    Hinge = 2,

    /// <summary>
    /// Keeps two anchors together and constrains swing around the local Z forward axis with optional twist limits.
    /// </summary>
    ConeTwist = 3,

    /// <summary>
    /// Keeps two anchors together and removes all relative angular freedom.
    /// </summary>
    Fixed = 4
}
