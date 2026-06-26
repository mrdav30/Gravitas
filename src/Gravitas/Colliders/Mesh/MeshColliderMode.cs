//=======================================================================
// MeshColliderMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Declares how a mesh collider should be treated by runtime collision policy.
/// </summary>
public enum MeshColliderMode : byte
{
    /// <summary>
    /// Mesh is intended to behave as a convex collision shape.
    /// </summary>
    Convex,

    /// <summary>
    /// Mesh is allowed to be concave and should be treated as triangle collision data.
    /// </summary>
    Concave
}
