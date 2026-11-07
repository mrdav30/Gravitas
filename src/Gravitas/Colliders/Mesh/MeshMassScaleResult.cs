//=======================================================================
// MeshMassScaleResult.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Distinguishes valid scaled mass properties from unrepresentable volume or inertia data.
/// </summary>
internal enum MeshMassScaleResult : byte
{
    Valid,
    NonRepresentableVolume,
    NonRepresentableMassProperties
}
