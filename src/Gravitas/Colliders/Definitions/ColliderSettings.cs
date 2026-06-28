//=======================================================================
// ColliderSettings.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;

namespace Gravitas.Colliders;

public static class ColliderSettings
{
    public static int GetPriority(ColliderType type) =>
        type switch
        {
            ColliderType.Sphere => 0,
            ColliderType.Capsule => 1,
            ColliderType.Cylinder => 1,
            ColliderType.Cone => 1,
            ColliderType.AABox => 2,
            ColliderType.OBBox => 2,
            ColliderType.Mesh => 3,
            ColliderType.Compound => 4,
            _ => -1
        };

    private static readonly SwiftDictionary<(ColliderType, ColliderType), CollisionType> CollisionTypeMapping = new()
    {
            // Sphere
            {(ColliderType.Sphere, ColliderType.Sphere), CollisionType.Sphere_Sphere},
            {(ColliderType.Sphere, ColliderType.AABox), CollisionType.Cuboid_Sphere},
            {(ColliderType.Sphere, ColliderType.OBBox), CollisionType.Cuboid_Sphere},
            {(ColliderType.Sphere, ColliderType.Capsule), CollisionType.Capsule_Sphere},
            {(ColliderType.Sphere, ColliderType.Cylinder), CollisionType.Cylinder_Sphere},
            {(ColliderType.Sphere, ColliderType.Cone), CollisionType.Cone_Sphere},
            {(ColliderType.Sphere, ColliderType.Mesh), CollisionType.Mesh_Sphere},
            {(ColliderType.Sphere, ColliderType.Compound), CollisionType.Compound},
            // Capsule
            {(ColliderType.Capsule, ColliderType.Sphere), CollisionType.Capsule_Sphere},
            {(ColliderType.Capsule, ColliderType.AABox), CollisionType.AABox_Capsule},
            {(ColliderType.Capsule, ColliderType.OBBox), CollisionType.OBBox_Capsule},
            {(ColliderType.Capsule, ColliderType.Capsule), CollisionType.Capsule_Capsule},
            {(ColliderType.Capsule, ColliderType.Cylinder), CollisionType.Cylinder_Capsule},
            {(ColliderType.Capsule, ColliderType.Cone), CollisionType.Cone_Convex},
            {(ColliderType.Capsule, ColliderType.Mesh), CollisionType.Mesh_Capsule},
            {(ColliderType.Capsule, ColliderType.Compound), CollisionType.Compound},
            // AABox
            {(ColliderType.AABox, ColliderType.Sphere), CollisionType.Cuboid_Sphere},
            {(ColliderType.AABox, ColliderType.AABox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.AABox, ColliderType.OBBox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.AABox, ColliderType.Capsule), CollisionType.AABox_Capsule},
            {(ColliderType.AABox, ColliderType.Cylinder), CollisionType.Cuboid_Cylinder},
            {(ColliderType.AABox, ColliderType.Cone), CollisionType.Cone_Convex},
            {(ColliderType.AABox, ColliderType.Mesh), CollisionType.Mesh_Cuboid},
            {(ColliderType.AABox, ColliderType.Compound), CollisionType.Compound},
            // OBBox
            {(ColliderType.OBBox, ColliderType.Sphere), CollisionType.Cuboid_Sphere},
            {(ColliderType.OBBox, ColliderType.AABox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.OBBox, ColliderType.OBBox), CollisionType.Cuboid_Cuboid},
            {(ColliderType.OBBox, ColliderType.Capsule), CollisionType.OBBox_Capsule},
            {(ColliderType.OBBox, ColliderType.Cylinder), CollisionType.Cuboid_Cylinder},
            {(ColliderType.OBBox, ColliderType.Cone), CollisionType.Cone_Convex},
            {(ColliderType.OBBox, ColliderType.Mesh), CollisionType.Mesh_Cuboid},
            {(ColliderType.OBBox, ColliderType.Compound), CollisionType.Compound},
            // Cylinder
            {(ColliderType.Cylinder, ColliderType.Sphere), CollisionType.Cylinder_Sphere},
            {(ColliderType.Cylinder, ColliderType.Capsule), CollisionType.Cylinder_Capsule},
            {(ColliderType.Cylinder, ColliderType.AABox), CollisionType.Cuboid_Cylinder},
            {(ColliderType.Cylinder, ColliderType.OBBox), CollisionType.Cuboid_Cylinder},
            {(ColliderType.Cylinder, ColliderType.Cylinder), CollisionType.Cylinder_Cylinder},
            {(ColliderType.Cylinder, ColliderType.Cone), CollisionType.Cone_Convex},
            {(ColliderType.Cylinder, ColliderType.Mesh), CollisionType.Mesh_Cylinder},
            {(ColliderType.Cylinder, ColliderType.Compound), CollisionType.Compound},
            // Cone
            {(ColliderType.Cone, ColliderType.Sphere), CollisionType.Cone_Sphere},
            {(ColliderType.Cone, ColliderType.Capsule), CollisionType.Cone_Convex},
            {(ColliderType.Cone, ColliderType.AABox), CollisionType.Cone_Convex},
            {(ColliderType.Cone, ColliderType.OBBox), CollisionType.Cone_Convex},
            {(ColliderType.Cone, ColliderType.Cylinder), CollisionType.Cone_Convex},
            {(ColliderType.Cone, ColliderType.Cone), CollisionType.Cone_Convex},
            {(ColliderType.Cone, ColliderType.Mesh), CollisionType.Mesh_Cone},
            {(ColliderType.Cone, ColliderType.Compound), CollisionType.Compound},
            // Mesh
            {(ColliderType.Mesh, ColliderType.Sphere), CollisionType.Mesh_Sphere},
            {(ColliderType.Mesh, ColliderType.AABox), CollisionType.Mesh_Cuboid},
            {(ColliderType.Mesh, ColliderType.OBBox), CollisionType.Mesh_Cuboid},
            {(ColliderType.Mesh, ColliderType.Capsule), CollisionType.Mesh_Capsule},
            {(ColliderType.Mesh, ColliderType.Cylinder), CollisionType.Mesh_Cylinder},
            {(ColliderType.Mesh, ColliderType.Cone), CollisionType.Mesh_Cone},
            {(ColliderType.Mesh, ColliderType.Mesh), CollisionType.Mesh_Mesh},
            {(ColliderType.Mesh, ColliderType.Compound), CollisionType.Compound},
            // Compound
            {(ColliderType.Compound, ColliderType.Sphere), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.Capsule), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.AABox), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.OBBox), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.Cylinder), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.Cone), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.Mesh), CollisionType.Compound},
            {(ColliderType.Compound, ColliderType.Compound), CollisionType.Compound},
        };

    public static CollisionType GetCollisionType(ColliderType type1, ColliderType type2)
    {
        if (CollisionTypeMapping.TryGetValue((type1, type2), out CollisionType collisionType))
            return collisionType;
        else
            return CollisionType.None;
    }
}
