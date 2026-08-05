//=======================================================================
// CollisionType.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>Identifies the resolved 3D narrow-phase path for an ordered collider pair.</summary>
public enum CollisionType : byte
{
    /// <summary>No supported collision path.</summary>
    None,
    /// <summary>Sphere against sphere.</summary>
    Sphere_Sphere,
    /// <summary>Capsule against sphere.</summary>
    Capsule_Sphere,
    /// <summary>Capsule against capsule.</summary>
    Capsule_Capsule,
    /// <summary>Cuboid against sphere.</summary>
    Cuboid_Sphere,
    /// <summary>Axis-aligned box against capsule.</summary>
    AABox_Capsule,
    /// <summary>Oriented box against capsule.</summary>
    OBBox_Capsule,
    /// <summary>Cuboid against cuboid.</summary>
    Cuboid_Cuboid,
    /// <summary>Cylinder against sphere.</summary>
    Cylinder_Sphere,
    /// <summary>Cylinder against capsule.</summary>
    Cylinder_Capsule,
    /// <summary>Cylinder against cylinder.</summary>
    Cylinder_Cylinder,
    /// <summary>Cuboid against cylinder.</summary>
    Cuboid_Cylinder,
    /// <summary>Cone against sphere.</summary>
    Cone_Sphere,
    /// <summary>Cone against a convex shape.</summary>
    Cone_Convex,
    /// <summary>Mesh against sphere.</summary>
    Mesh_Sphere,
    /// <summary>Mesh against capsule.</summary>
    Mesh_Capsule,
    /// <summary>Mesh against cuboid.</summary>
    Mesh_Cuboid,
    /// <summary>Mesh against cylinder.</summary>
    Mesh_Cylinder,
    /// <summary>Mesh against cone.</summary>
    Mesh_Cone,
    /// <summary>Mesh against mesh.</summary>
    Mesh_Mesh,
    /// <summary>A pair involving at least one compound collider.</summary>
    Compound
}
