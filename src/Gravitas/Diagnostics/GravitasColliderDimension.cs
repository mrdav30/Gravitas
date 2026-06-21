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
    None = 0,
    ThreeD = 1,
    TwoD = 2
}
