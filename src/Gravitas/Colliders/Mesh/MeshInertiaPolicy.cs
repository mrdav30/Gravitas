//=======================================================================
// MeshInertiaPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Selects how mesh inertia is derived when a mesh body participates in angular dynamics.
/// </summary>
public enum MeshInertiaPolicy
{
    /// <summary>
    /// Require a validated closed triangle volume and compute solid mass properties.
    /// </summary>
    RequireClosedVolume = 0,

    /// <summary>
    /// Use a surface-area-weighted approximation for explicitly open or surface-only meshes.
    /// </summary>
    SurfaceApproximation = 1
}
