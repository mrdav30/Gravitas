//=======================================================================
// BodyFreezeAxes3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace Gravitas;

/// <summary>
/// Identifies the translational and rotational degrees of freedom frozen on a
/// 3D <see cref="SolidBody"/>.
/// </summary>
[Flags]
public enum BodyFreezeAxes3D
{
    /// <summary>
    /// No position or rotation axes are frozen.
    /// </summary>
    None = 0,

    /// <summary>
    /// Freezes world-space X translation.
    /// </summary>
    PositionX = 1 << 0,

    /// <summary>
    /// Freezes world-space Y translation.
    /// </summary>
    PositionY = 1 << 1,

    /// <summary>
    /// Freezes world-space Z translation.
    /// </summary>
    PositionZ = 1 << 2,

    /// <summary>
    /// Freezes angular velocity around the world-space X axis.
    /// </summary>
    RotationX = 1 << 3,

    /// <summary>
    /// Freezes angular velocity around the world-space Y axis.
    /// </summary>
    RotationY = 1 << 4,

    /// <summary>
    /// Freezes angular velocity around the world-space Z axis.
    /// </summary>
    RotationZ = 1 << 5,

    /// <summary>
    /// Freezes all world-space translation axes.
    /// </summary>
    Position = PositionX | PositionY | PositionZ,

    /// <summary>
    /// Freezes all world-space rotation axes.
    /// </summary>
    Rotation = RotationX | RotationY | RotationZ,

    /// <summary>
    /// Freezes all translational and rotational degrees of freedom.
    /// </summary>
    All = Position | Rotation
}
