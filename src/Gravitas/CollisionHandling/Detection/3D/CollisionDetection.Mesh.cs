//=======================================================================
// CollisionDetection.Mesh.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Mesh

    private static bool DoMeshSphereCheck(CollisionWorkItem pair)
    {
        var meshCollider = (LSMeshCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        return MeshTriangleContactGenerator.TryBuildMeshSphereManifold(
            pair,
            meshCollider,
            sphere,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA);
    }

    private static bool DoMeshCapsuleCheck(CollisionWorkItem pair)
    {
        var mesh = (LSMeshCollider)pair.ColliderA;
        var capsule = (LSCapsuleCollider)pair.ColliderB;

        if (mesh.Mode == MeshColliderMode.Convex)
        {
            PhysicsMesh physicsMesh = mesh.Mesh;
            bool hasContact =
                FixedConvexHullRelations
                    .TryGetCenteredCapsuleContact(
                        physicsMesh.Origin,
                        physicsMesh.Rotation,
                        physicsMesh.ScaledLocalVertices,
                        physicsMesh.Triangles,
                        physicsMesh.ConvexSatEdgeVertexPairs,
                        capsule.Center,
                        capsule.Rotation,
                        Vector3d.Up,
                        capsule.AxisLength,
                        capsule.ScaledRadius,
                        out FixedContactAnchors contact);
            if (!hasContact)
                return false;
            pair.Manifold.SetContact(
                new ContactAnchor(contact.FirstAnchor),
                new ContactAnchor(contact.SecondAnchor),
                contact.Depth,
                contact.Normal,
                contact.DepthIsClamped);
            return true;
        }

        return MeshTriangleContactGenerator.TryBuildMeshCapsuleManifold(
            pair,
            mesh,
            capsule,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA);
    }

    private static bool DoMeshCuboidCheck(CollisionWorkItem pair)
    {
        var mesh = (LSMeshCollider)pair.ColliderA;
        var cuboid = (LSCuboidCollider)pair.ColliderB;

        if (MeshTriangleContactGenerator.TryBuildMeshCuboidManifold(
            pair,
            mesh,
            cuboid,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA))
        {
            return true;
        }

        if (mesh.Mode == MeshColliderMode.Concave)
            return false;

        PhysicsMesh physicsMesh = mesh.Mesh;
        if (!cuboid.OrientedBox.TryGetConvexHullContact(
                mesh.Center,
                physicsMesh.Rotation,
                physicsMesh.ScaledLocalVertices,
                physicsMesh.Triangles,
                physicsMesh.ConvexSatEdgeVertexPairs,
                out FixedContactAnchors contact))
        {
            return false;
        }

        pair.Manifold.SetContact(
            new ContactAnchor(contact.SecondAnchor),
            new ContactAnchor(contact.FirstAnchor),
            contact.Depth,
            -contact.Normal,
            contact.DepthIsClamped);

        return true;
    }

    private static bool DoMeshCylinderCheck(CollisionWorkItem pair)
    {
        var mesh = (LSMeshCollider)pair.ColliderA;
        var cylinder = (LSCylinderCollider)pair.ColliderB;

        return MeshTriangleContactGenerator.TryBuildMeshCylinderManifold(
            pair,
            mesh,
            cylinder,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA);
    }

    /// <summary>
    /// Checks if two mesh colliders intersect and sets the contact point if they do.
    /// </summary>
    /// <param name="pair">The pair of colliders to test for collision.</param>
    /// <returns>true if the colliders intersect; otherwise, false.</returns>
    private static bool DoMeshesCheck(CollisionWorkItem pair)
    {
        var meshA = (LSMeshCollider)pair.ColliderA;
        var meshB = (LSMeshCollider)pair.ColliderB;
        if (meshA.Mode == MeshColliderMode.Concave
            || meshB.Mode == MeshColliderMode.Concave)
        {
            return MeshTriangleContactGenerator.TryBuildMeshMeshManifold(
                pair,
                meshA,
                meshB,
                pair.Context.CollisionScratch.MeshTriangleCandidatesA,
                pair.Context.CollisionScratch.MeshTriangleCandidatesB);
        }

        PhysicsMesh physicsMeshA = meshA.Mesh;
        PhysicsMesh physicsMeshB = meshB.Mesh;
        bool hasContact = FixedConvexHullRelations.TryGetContact(
            meshA.Center,
            physicsMeshA.Rotation,
            physicsMeshA.ScaledLocalVertices,
            physicsMeshA.Triangles,
            physicsMeshA.ConvexSatEdgeVertexPairs,
            meshB.Center,
            physicsMeshB.Rotation,
            physicsMeshB.ScaledLocalVertices,
            physicsMeshB.Triangles,
            physicsMeshB.ConvexSatEdgeVertexPairs,
            out FixedContactAnchors contact);
        if (!hasContact)
            return false;
        if (contact.Depth <= Fixed64.Zero)
            return false;

        pair.Manifold.SetContact(
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);

        return true;
    }

    #endregion
}
