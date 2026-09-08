//=======================================================================
// ColliderReconfigurationStatus.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>Describes the result of an exact registered-collider reconfiguration attempt.</summary>
public enum ColliderReconfigurationStatus
{
    /// <summary>The requested geometry and root pose were published.</summary>
    Applied,
    /// <summary>The requested geometry, offset, and root pose already matched.</summary>
    Unchanged,
    /// <summary>A physically eligible collider had positive penetration with the candidate shape.</summary>
    Blocked
}
