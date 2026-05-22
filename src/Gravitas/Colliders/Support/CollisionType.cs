namespace Gravitas.Colliders
{
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
        Mesh_Sphere,
        Mesh_Capsule,
        Mesh_Cuboid,
        Mesh_Mesh
        // ... Add all other combinations here
    }
}
