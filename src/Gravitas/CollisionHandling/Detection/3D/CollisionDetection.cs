//=======================================================================
// CollisionDetection.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    public static bool DoCollisionCheck(CollisionPair pair)
    {
        pair.Manifold.BeginUpdate(pair.Context.FrameCount);
        return DoCollisionCheck(CollisionWorkItem.Create(pair));
    }

    internal static bool DoCollisionCheck(CollisionWorkItem pair)
    {
        return pair.CollisionType switch
        {
            CollisionType.Sphere_Sphere => DoSpheresCheck(pair),
            CollisionType.Capsule_Sphere => DoCapsuleSphereCheck(pair),
            CollisionType.Capsule_Capsule => DoCapsulesCheck(pair),
            CollisionType.Cuboid_Sphere => DoCuboidSphereCheck(pair),
            CollisionType.AABox_Capsule => DoCuboidCapsuleCheck(pair),
            CollisionType.OBBox_Capsule => DoCuboidCapsuleCheck(pair),
            CollisionType.Cuboid_Cuboid => DoCuboidsCheck(pair),
            CollisionType.Cylinder_Sphere => DoCylinderSphereCheck(pair),
            CollisionType.Cylinder_Capsule => DoCylinderCapsuleCheck(pair),
            CollisionType.Cylinder_Cylinder => DoCylindersCheck(pair),
            CollisionType.Cuboid_Cylinder => DoCuboidCylinderCheck(pair),
            CollisionType.Cone_Sphere => DoConeSphereCheck(pair),
            CollisionType.Cone_Convex => DoConeConvexCheck(pair),
            CollisionType.Mesh_Sphere => DoMeshSphereCheck(pair),
            CollisionType.Mesh_Capsule => DoMeshCapsuleCheck(pair),
            CollisionType.Mesh_Cuboid => DoMeshCuboidCheck(pair),
            CollisionType.Mesh_Cylinder => DoMeshCylinderCheck(pair),
            CollisionType.Mesh_Cone => DoMeshConeCheck(pair),
            CollisionType.Mesh_Mesh => DoMeshesCheck(pair),
            CollisionType.Compound => DoCompoundCheck(pair),
            _ => false,
        };
    }

    private static void KeepDirectionalPenetration(
        Vector3d axis,
        FixedRange firstProjection,
        FixedRange secondProjection,
        ref AxisPenetration penetration)
    {
        Fixed64 positiveDepth = firstProjection.Max - secondProjection.Min;
        Fixed64 negativeDepth = secondProjection.Max - firstProjection.Min;
        Vector3d candidateAxis = axis;
        Fixed64 candidateDepth = positiveDepth;
        if (negativeDepth < positiveDepth)
        {
            candidateAxis = -axis;
            candidateDepth = negativeDepth;
        }

        if (!penetration.HasValue || candidateDepth < penetration.Depth)
            penetration = new AxisPenetration(candidateAxis, candidateDepth);
    }

}
