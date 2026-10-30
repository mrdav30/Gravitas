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

/// <content>Dispatches whole-hull volume and finite-surface mesh contacts.</content>
public static partial class CollisionDetection
{
    #region Mesh

    private static bool DoMeshSphereCheck(CollisionWorkItem pair)
    {
        var meshCollider = (LSMeshCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        // Only closed convex meshes fill their volume; open meshes stay surfaces.
        if (meshCollider.Mode == MeshColliderMode.Convex && meshCollider.Mesh.IsClosedSurface)
            return TrySetConvexMeshRoundedContact(pair, meshCollider.Mesh, sphere.Center,
                sphere.Rotation, Fixed64.Zero, sphere.ScaledRadius);

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
            return TrySetConvexMeshRoundedContact(pair, mesh.Mesh, capsule.Center,
                capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius);

        return MeshTriangleContactGenerator.TryBuildMeshCapsuleManifold(
            pair,
            mesh,
            capsule,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA);
    }

    private static bool TrySetConvexMeshRoundedContact(CollisionWorkItem pair, PhysicsMesh mesh,
        Vector3d center, FixedQuaternion rotation, Fixed64 axisLength, Fixed64 radius)
    {
        if (!WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(mesh.GeometryFrame, mesh.Origin,
            mesh.GeometryVertices, mesh.Triangles, mesh.ConvexSatEdgeVertexPairs,
            center, rotation, Vector3d.Up, axisLength, radius, out FixedContactAnchors contact))
            return false;
        pair.Manifold.SetContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
            contact.Depth, contact.Normal, contact.DepthIsClamped);
        return true;
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
        if (!WideOrientedBox.TryGetConvexHullContact(
                cuboid.Center, cuboid.Rotation, cuboid.OrientedBox.HalfExtents,
                physicsMesh.GeometryFrame, physicsMesh.Origin,
                physicsMesh.GeometryVertices,
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

        // An interior/boundary center needs the filled hull's exit, even when
        // it already reaches a wall whose two-sided surface would push inward.
        if (mesh.Mode == MeshColliderMode.Convex && mesh.Mesh.IsClosedSurface
            && ConvexHullCylinderContact.TryGetContactWithContainedCenter(mesh.Mesh.GeometryFrame,
                mesh.Mesh.GeometryVertices, mesh.Mesh.Triangles, cylinder.Center, cylinder.Rotation,
                cylinder.Height, cylinder.ScaledRadius, out FixedContactAnchors contact))
        {
            pair.Manifold.SetContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, contact.DepthIsClamped);
            return true;
        }
        return MeshTriangleContactGenerator.TryBuildMeshCylinderManifold(pair, mesh, cylinder,
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
        if (meshA.Mesh.IsClosedSurface != meshB.Mesh.IsClosedSurface
            || meshA.Mode == MeshColliderMode.Concave
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
        bool hasContact = physicsMeshA.GeometryFrame.IsRigid && physicsMeshB.GeometryFrame.IsRigid
            ? FixedConvexHullRelations.TryGetContact(physicsMeshA.Origin, physicsMeshA.Rotation,
                physicsMeshA.GeometryVertices, physicsMeshA.Triangles, physicsMeshA.ConvexSatEdgeVertexPairs,
                physicsMeshB.Origin, physicsMeshB.Rotation, physicsMeshB.GeometryVertices,
                physicsMeshB.Triangles, physicsMeshB.ConvexSatEdgeVertexPairs, out FixedContactAnchors contact)
            : WideOrientedBox.TryGetConvexHullContact(physicsMeshA.GeometryFrame, physicsMeshA.Origin,
                physicsMeshA.GeometryVertices, physicsMeshA.Triangles, physicsMeshA.ConvexSatEdgeVertexPairs,
                physicsMeshB.GeometryFrame, physicsMeshB.Origin, physicsMeshB.GeometryVertices,
                physicsMeshB.Triangles, physicsMeshB.ConvexSatEdgeVertexPairs, out contact);
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
