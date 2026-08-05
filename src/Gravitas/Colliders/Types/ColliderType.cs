//=======================================================================
// ColliderType.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>Identifies runtime 3D collider shapes.</summary>
public enum ColliderType : byte
{
    /// <summary>No collider shape.</summary>
    None,
    /// <summary>A sphere.</summary>
    Sphere,
    /// <summary>An axis-aligned box.</summary>
    AABox,
    /// <summary>An oriented box.</summary>
    OBBox,
    /// <summary>A capsule.</summary>
    Capsule,
    /// <summary>A finite cylinder.</summary>
    Cylinder,
    /// <summary>A finite cone.</summary>
    Cone,
    /// <summary>A triangle mesh.</summary>
    Mesh,
    /// <summary>A compound collider.</summary>
    Compound
}
