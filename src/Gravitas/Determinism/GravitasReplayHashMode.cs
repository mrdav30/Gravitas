//=======================================================================
// GravitasReplayHashMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>
/// Selects which deterministic runtime state is included in replay hashes.
/// </summary>
public enum GravitasReplayHashMode
{
    /// <summary>
    /// Includes authoritative state that determines deterministic continuation.
    /// </summary>
    Authoritative = 0,

    /// <summary>
    /// Includes authoritative state plus stable solver/cache state useful for drift RCA.
    /// </summary>
    AuthoritativeWithSolverCaches = 1
}
