using SwiftCollections;

namespace Gravitas.Colliders;

public static class ColliderSettings
{
    public static int GetPriority(ColliderType type) =>
        type switch
        {
            ColliderType.Sphere => 0,
            ColliderType.Capsule => 1,
            ColliderType.AABox => 2,
            ColliderType.OBBox => 2,
            ColliderType.Mesh => 3,
            _ => -1
        };

    private static readonly SwiftDictionary<(ColliderType, ColliderType), CollisionType> CollisionTypeMapping = new()
    {
            // Sphere
            {(ColliderType.Sphere, ColliderType.Sphere), CollisionType.Sphere_Sphere},
            {(ColliderType.Sphere, ColliderType.AABox), CollisionType.Cuboid_Sphere},
            {(ColliderType.Sphere, ColliderType.OBBox), CollisionType.Cuboid_Sphere},
            {(ColliderType.Sphere, ColliderType.Capsule), CollisionType.Capsule_Sphere},
            {(ColliderType.Sphere, ColliderType.Mesh), CollisionType.Mesh_Sphere},
            // Capsule
            {(ColliderType.Capsule, ColliderType.Sphere), CollisionType.Capsule_Sphere},
            {(ColliderType.Capsule, ColliderType.AABox), CollisionType.AABox_Capsule},
            {(ColliderType.Capsule, ColliderType.OBBox), CollisionType.OBBox_Capsule},
            {(ColliderType.Capsule, ColliderType.Capsule), CollisionType.Capsule_Capsule},
            {(ColliderType.Capsule, ColliderType.Mesh), CollisionType.Mesh_Capsule},
            // AABox
            {(ColliderType.AABox, ColliderType.Sphere), CollisionType.Cuboid_Sphere},
            {(ColliderType.AABox, ColliderType.AABox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.AABox, ColliderType.OBBox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.AABox, ColliderType.Capsule), CollisionType.AABox_Capsule},
            {(ColliderType.AABox, ColliderType.Mesh), CollisionType.Mesh_Cuboid},
            // OBBox
            {(ColliderType.OBBox, ColliderType.Sphere), CollisionType.Cuboid_Sphere},
            {(ColliderType.OBBox, ColliderType.AABox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.OBBox, ColliderType.OBBox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.OBBox, ColliderType.Capsule), CollisionType.OBBox_Capsule},
            {(ColliderType.OBBox, ColliderType.Mesh), CollisionType.Mesh_Cuboid},
            // Mesh
            {(ColliderType.Mesh, ColliderType.Sphere), CollisionType.Mesh_Sphere},
            {(ColliderType.Mesh, ColliderType.AABox), CollisionType.Mesh_Cuboid},
            {(ColliderType.Mesh, ColliderType.OBBox), CollisionType.Mesh_Cuboid},
            {(ColliderType.Mesh, ColliderType.Capsule), CollisionType.Mesh_Capsule},
            {(ColliderType.Mesh, ColliderType.Mesh), CollisionType.Mesh_Mesh},
            // ... Add all other combinations here
        };

    public static CollisionType GetCollisionType(ColliderType type1, ColliderType type2)
    {
        if (CollisionTypeMapping.TryGetValue((type1, type2), out CollisionType collisionType))
            return collisionType;
        else
            return CollisionType.None;
    }
}
