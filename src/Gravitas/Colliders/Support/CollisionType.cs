//=======================================================================
// CollisionType.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

public enum CollisionType : byte
{
    None,
    Sphere_Sphere,
    Capsule_Sphere,
    Capsule_Capsule,
    Cuboid_Sphere,
    AABox_Capsule,
    OBBox_Capsule,
    Cuboid_Cuboid,
    Cylinder_Sphere,
    Cylinder_Capsule,
    Cylinder_Cylinder,
    Cuboid_Cylinder,
    Mesh_Sphere,
    Mesh_Capsule,
    Mesh_Cuboid,
    Mesh_Cylinder,
    Mesh_Mesh,
    Compound
}
