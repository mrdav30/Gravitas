//=======================================================================
// GravitasColliderDimension.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Identifies the dimensional runtime surface for diagnostic collider payloads.
/// </summary>
public enum GravitasColliderDimension : byte
{
    /// <summary>No collider dimension is associated with the payload.</summary>
    None = 0,

    /// <summary>The payload describes a 3D collider.</summary>
    ThreeD = 1,

    /// <summary>The payload describes a 2D collider.</summary>
    TwoD = 2
}
