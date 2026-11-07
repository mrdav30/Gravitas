//=======================================================================
// GroundProbeMode2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>
/// Selects the pure 2D query primitive used by <see cref="SolidBody2D"/> when probing for planar support.
/// </summary>
public enum GroundProbeMode2D : byte
{
    /// <summary>
    /// Chooses the probe from the body's collider shape and configured probe radius.
    /// </summary>
    Auto,

    /// <summary>
    /// Uses a narrow in-plane segment raycast along the support down direction.
    /// </summary>
    Ray,

    /// <summary>
    /// Uses a swept circle along the support down direction.
    /// </summary>
    SweptCircle
}
