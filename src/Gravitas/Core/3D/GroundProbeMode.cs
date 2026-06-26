//=======================================================================
// GroundProbeMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>
/// Selects the query primitive used by <see cref="SolidBody"/> when probing for ground.
/// </summary>
public enum GroundProbeMode : byte
{
    /// <summary>
    /// Chooses the probe from the body's collider shape and size.
    /// </summary>
    Auto,

    /// <summary>
    /// Uses a narrow downward ray probe.
    /// </summary>
    Ray,

    /// <summary>
    /// Uses a swept sphere probe with an explicit or shape-derived radius.
    /// </summary>
    SweptSphere
}
