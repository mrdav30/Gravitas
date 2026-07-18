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
    /// Mesh is validated as either one closed convex manifold shell or one open,
    /// coplanar triangulation that fills a single convex polygon.
    /// </summary>
    Convex,

    /// <summary>
    /// Mesh may contain arbitrary open or closed triangle surfaces and is treated as
    /// authored triangle collision data rather than a support-mapped convex hull.
    /// </summary>
    Concave
}
