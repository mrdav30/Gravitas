//=======================================================================
// GroundingMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>
/// Selects whether a <see cref="StiffBody"/> owns ground detection through probes or receives grounding from the host.
/// </summary>
public enum GroundingMode : byte
{
    /// <summary>
    /// Gravitas updates grounding from deterministic ray or swept-sphere probes.
    /// </summary>
    Automatic,

    /// <summary>
    /// The host owns grounded state and must call manual grounding methods when the state changes.
    /// </summary>
    Manual
}
