//=======================================================================
// IntervalSearchStatus.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>
/// Distinguishes a certified no-hit result, an exact hit, and an unresolved search.
/// </summary>
internal enum IntervalSearchStatus : byte
{
    /// <summary>
    /// Indicates that the search has determined there is no collision within the interval.
    /// </summary>
    CertifiedNoHit,
    /// <summary>
    /// Indicates that the search has found an exact collision within the interval.
    /// </summary>
    ExactHit,
    /// <summary>
    /// Indicates that the search could not determine the collision status within the interval.
    /// </summary>
    Unresolved,
}
