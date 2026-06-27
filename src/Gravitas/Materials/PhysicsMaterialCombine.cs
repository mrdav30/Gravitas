//=======================================================================
// PhysicsMaterialCombine.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Materials;

/// <summary>
/// Defines how two deterministic scalar material coefficients are combined for
/// one contact pair.
/// </summary>
public enum PhysicsMaterialCombine
{
    /// <summary>
    /// Uses the lower coefficient from the two contacting surfaces.
    /// </summary>
    Minimum,

    /// <summary>
    /// Uses the higher coefficient from the two contacting surfaces.
    /// </summary>
    Maximum,

    /// <summary>
    /// Uses the arithmetic mean of the two contacting surfaces.
    /// </summary>
    Average,

    /// <summary>
    /// Multiplies the two contacting surface coefficients.
    /// </summary>
    Multiply,

    /// <summary>
    /// Uses the square root of the product of the two contacting surfaces.
    /// </summary>
    GeometricMean
}
