//=======================================================================
// BodyFreezeAxes2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace Gravitas;

/// <summary>
/// Identifies the planar translational and yaw rotational degrees of freedom
/// frozen on a pure 2D <see cref="SolidBody2D"/>.
/// </summary>
[Flags]
public enum BodyFreezeAxes2D
{
    /// <summary>
    /// No planar position or yaw rotation axes are frozen.
    /// </summary>
    None = 0,

    /// <summary>
    /// Freezes the first planar coordinate, mapped to world-space X.
    /// </summary>
    PositionX = 1 << 0,

    /// <summary>
    /// Freezes the second planar coordinate, mapped to world-space Z.
    /// </summary>
    PositionY = 1 << 1,

    /// <summary>
    /// Freezes yaw rotation around the embedded world-space Y axis.
    /// </summary>
    Rotation = 1 << 2,

    /// <summary>
    /// Freezes both planar translation axes.
    /// </summary>
    Position = PositionX | PositionY,

    /// <summary>
    /// Freezes planar translation and yaw rotation.
    /// </summary>
    All = Position | Rotation
}
