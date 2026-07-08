//=======================================================================
// ColliderSettings.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public static class ColliderSettings
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CollisionType GetCollisionType(ColliderType type1, ColliderType type2) =>
        (type1, type2) switch
        {
            (ColliderType.Sphere, ColliderType.Sphere) => CollisionType.Sphere_Sphere,
            (ColliderType.Sphere, ColliderType.AABox or ColliderType.OBBox) => CollisionType.Cuboid_Sphere,
            (ColliderType.Sphere, ColliderType.Capsule) => CollisionType.Capsule_Sphere,
            (ColliderType.Sphere, ColliderType.Cylinder) => CollisionType.Cylinder_Sphere,
            (ColliderType.Sphere, ColliderType.Cone) => CollisionType.Cone_Sphere,
            (ColliderType.Sphere, ColliderType.Mesh) => CollisionType.Mesh_Sphere,
            (ColliderType.Sphere, ColliderType.Compound) => CollisionType.Compound,

            (ColliderType.Capsule, ColliderType.Sphere) => CollisionType.Capsule_Sphere,
            (ColliderType.Capsule, ColliderType.AABox) => CollisionType.AABox_Capsule,
            (ColliderType.Capsule, ColliderType.OBBox) => CollisionType.OBBox_Capsule,
            (ColliderType.Capsule, ColliderType.Capsule) => CollisionType.Capsule_Capsule,
            (ColliderType.Capsule, ColliderType.Cylinder) => CollisionType.Cylinder_Capsule,
            (ColliderType.Capsule, ColliderType.Cone) => CollisionType.Cone_Convex,
            (ColliderType.Capsule, ColliderType.Mesh) => CollisionType.Mesh_Capsule,
            (ColliderType.Capsule, ColliderType.Compound) => CollisionType.Compound,

            (ColliderType.AABox or ColliderType.OBBox, ColliderType.Sphere) => CollisionType.Cuboid_Sphere,
            (ColliderType.AABox, ColliderType.Capsule) => CollisionType.AABox_Capsule,
            (ColliderType.OBBox, ColliderType.Capsule) => CollisionType.OBBox_Capsule,
            (ColliderType.AABox or ColliderType.OBBox, ColliderType.AABox or ColliderType.OBBox) =>
                CollisionType.Cuboid_Cuboid,
            (ColliderType.AABox or ColliderType.OBBox, ColliderType.Cylinder) =>
                CollisionType.Cuboid_Cylinder,
            (ColliderType.AABox or ColliderType.OBBox, ColliderType.Cone) => CollisionType.Cone_Convex,
            (ColliderType.AABox or ColliderType.OBBox, ColliderType.Mesh) => CollisionType.Mesh_Cuboid,
            (ColliderType.AABox or ColliderType.OBBox, ColliderType.Compound) => CollisionType.Compound,

            (ColliderType.Cylinder, ColliderType.Sphere) => CollisionType.Cylinder_Sphere,
            (ColliderType.Cylinder, ColliderType.Capsule) => CollisionType.Cylinder_Capsule,
            (ColliderType.Cylinder, ColliderType.AABox or ColliderType.OBBox) =>
                CollisionType.Cuboid_Cylinder,
            (ColliderType.Cylinder, ColliderType.Cylinder) => CollisionType.Cylinder_Cylinder,
            (ColliderType.Cylinder, ColliderType.Cone) => CollisionType.Cone_Convex,
            (ColliderType.Cylinder, ColliderType.Mesh) => CollisionType.Mesh_Cylinder,
            (ColliderType.Cylinder, ColliderType.Compound) => CollisionType.Compound,

            (
                ColliderType.Cone,
                ColliderType.Capsule
                    or ColliderType.AABox
                    or ColliderType.OBBox
                    or ColliderType.Cylinder
                    or ColliderType.Cone) => CollisionType.Cone_Convex,
            (ColliderType.Cone, ColliderType.Sphere) => CollisionType.Cone_Sphere,
            (ColliderType.Cone, ColliderType.Mesh) => CollisionType.Mesh_Cone,
            (ColliderType.Cone, ColliderType.Compound) => CollisionType.Compound,

            (ColliderType.Mesh, ColliderType.Sphere) => CollisionType.Mesh_Sphere,
            (ColliderType.Mesh, ColliderType.Capsule) => CollisionType.Mesh_Capsule,
            (ColliderType.Mesh, ColliderType.AABox or ColliderType.OBBox) => CollisionType.Mesh_Cuboid,
            (ColliderType.Mesh, ColliderType.Cylinder) => CollisionType.Mesh_Cylinder,
            (ColliderType.Mesh, ColliderType.Cone) => CollisionType.Mesh_Cone,
            (ColliderType.Mesh, ColliderType.Mesh) => CollisionType.Mesh_Mesh,
            (ColliderType.Mesh, ColliderType.Compound) => CollisionType.Compound,

            (
                ColliderType.Compound,
                ColliderType.Sphere
                    or ColliderType.Capsule
                    or ColliderType.AABox
                    or ColliderType.OBBox
                    or ColliderType.Cylinder
                    or ColliderType.Cone
                    or ColliderType.Mesh
                    or ColliderType.Compound) => CollisionType.Compound,

            _ => CollisionType.None
        };
}
