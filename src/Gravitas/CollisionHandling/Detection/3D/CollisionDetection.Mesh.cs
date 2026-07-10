//=======================================================================
// CollisionDetection.Mesh.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Mesh

    /// <summary>
    /// Assumes collider A is the mesh collider and collider B is the sphere collider.
    /// </summary>
    /// <param name="pair"></param>
    /// <returns>True if colliders intersect, otherwise false</returns>
    public static bool DoMeshSphereCheck(CollisionPair pair)
    {
        pair.Manifold.BeginUpdate(pair.Context.FrameCount);
        return DoMeshSphereCheck(CollisionWorkItem.Create(pair));
    }

    private static bool DoMeshSphereCheck(CollisionWorkItem pair)
    {
        var meshCollider = (LSMeshCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        if (meshCollider.Mode == MeshColliderMode.Concave)
            return MeshTriangleContactGenerator.TryBuildMeshSphereManifold(
                pair,
                meshCollider,
                sphere,
                pair.Context.CollisionScratch.MeshTriangleCandidatesA);

        Vector3d closestPointOnMesh = meshCollider.TryFindClosestPointOnSurface(
            sphere.Center,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA,
            out Vector3d closest,
            out Vector3d surfaceNormal)
                ? closest
                : meshCollider.Bounds.ClosestPointOnSurface(sphere.Center);
        Vector3d penetrationVector = sphere.Center - closestPointOnMesh;
        Fixed64 distanceSquared = penetrationVector.MagnitudeSquared;
        if (distanceSquared > sphere.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove sphere's radius to find the actual depth

        Fixed64 distance = distanceSquared <= Fixed64.Epsilon
            ? Fixed64.Zero
            : FixedMath.Sqrt(distanceSquared);
        Vector3d penetrationNormal = distance > Fixed64.Epsilon
            ? penetrationVector / distance
            : OrientNormal(surfaceNormal, sphere.Center - meshCollider.Center);
        pair.Manifold.SetContact(
            closestPointOnMesh,
            sphere.Center - penetrationNormal * sphere.ScaledRadius,
            sphere.ScaledRadius - distance,
            penetrationNormal
        );

        return true;
    }

    private static bool DoMeshCapsuleCheck(CollisionWorkItem pair)
    {
        var mesh = (LSMeshCollider)pair.ColliderA;
        var capsule = (LSCapsuleCollider)pair.ColliderB;

        if (mesh.Mode == MeshColliderMode.Concave)
            return MeshTriangleContactGenerator.TryBuildMeshCapsuleManifold(
                pair,
                mesh,
                capsule,
                pair.Context.CollisionScratch.MeshTriangleCandidatesA);

        // Calculate the closest point on the capsule line segment to the mesh center
        Vector3d closestPointOnCapsuleLine = Vector3d.ClosestPointOnLineSegment(capsule.LineSegmentStart, capsule.LineSegmentEnd, mesh.Center);
        Vector3d closetPointOnMesh = mesh.ClosestPointOnSurface(closestPointOnCapsuleLine);
        // Check if the distance from the closest point to the mesh is less than the capsule radius
        if ((closetPointOnMesh - closestPointOnCapsuleLine).MagnitudeSquared > capsule.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove capsule's radius to find the actual depth

        // Use the normal of the triangle where the collision occurs.
        Vector3d penetrationNormal = (closestPointOnCapsuleLine - closetPointOnMesh).Normalized;
        // penetration vector should be along the normal direction
        Vector3d penetrationVector = penetrationNormal * (capsule.ScaledRadius - Vector3d.Distance(closestPointOnCapsuleLine, closetPointOnMesh));
        // find collision point on the capsule
        Vector3d collisionPointCapsule = closestPointOnCapsuleLine - penetrationNormal * capsule.ScaledRadius;
        pair.Manifold.SetContact(
            closetPointOnMesh,
            collisionPointCapsule,
            penetrationVector.Magnitude,
            penetrationNormal
        );

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

        if (!TestMeshCuboidColliders(pair, mesh, cuboid, out CollisionResult output))
            return false;

        pair.Manifold.SetContact(
            output.PointsOfContact.Point1,
            output.PointsOfContact.Point2,
            output.AxisPenetration.Depth,
            output.AxisPenetration.Vector.Normalized);

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
    /// Tests if there are any separating axes between a cuboid and a mesh using the given axis vectors.
    /// </summary>
    /// <returns>true if no separating axis is found, false otherwise.</returns>
    private static bool TestMeshCuboidColliders(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCuboidCollider cuboid,
        out CollisionResult output)
    {
        output = default;

        (Vector3d PointA, Vector3d PointB) = FindInitialPointsOfContact(mesh, cuboid);
        if (!TryFindConvexMeshCuboidPenetration(mesh, cuboid, out AxisPenetration penetration))
            return false;

        output = new CollisionResult(
            (PointA, PointB),
            (penetration.Axis, penetration.Depth));

        return true;
    }

    private static bool TryFindConvexMeshCuboidPenetration(
        LSMeshCollider mesh,
        LSCuboidCollider cuboid,
        out AxisPenetration penetration)
    {
        penetration = default;
        PhysicsMesh physicsMesh = mesh.Mesh;
        Vector3d[] meshVertices = physicsMesh.Vertices;
        int triangleCount = physicsMesh.TriangleCount;
        Vector3d[] cuboidVertices = cuboid.Vertices;
        Vector3d meshToCuboid = cuboid.Center - mesh.Center;

        for (int i = 0; i < triangleCount; i++)
        {
            if (!CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, physicsMesh.GetFaceNormalWorld(i), meshToCuboid, ref penetration))
                return false;
        }

        // Cuboid face/edge arrays contain opposite or parallel duplicates; one representative per axis is enough for SAT.
        if (!CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, cuboid.FaceNormals[0], meshToCuboid, ref penetration)
            || !CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, cuboid.FaceNormals[2], meshToCuboid, ref penetration)
            || !CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, cuboid.FaceNormals[4], meshToCuboid, ref penetration))
            return false;

        int[] edgeVertexPairs = physicsMesh.ConvexSatEdgeVertexPairs;
        for (int i = 0; i < edgeVertexPairs.Length; i += 2)
        {
            Vector3d meshEdge = meshVertices[edgeVertexPairs[i + 1]] - meshVertices[edgeVertexPairs[i]];
            if (!CheckConvexMeshCuboidEdgeAxes(meshVertices, cuboidVertices, cuboid, meshEdge, meshToCuboid, ref penetration))
                return false;
        }

        return penetration.HasValue;
    }

    private static bool CheckConvexMeshCuboidEdgeAxes(
        Vector3d[] meshVertices,
        Vector3d[] cuboidVertices,
        LSCuboidCollider cuboid,
        Vector3d meshEdge,
        Vector3d meshToCuboid,
        ref AxisPenetration penetration)
    {
        return CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, Vector3d.Cross(meshEdge, cuboid.EdgeDirections[0]), meshToCuboid, ref penetration)
            && CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, Vector3d.Cross(meshEdge, cuboid.EdgeDirections[2]), meshToCuboid, ref penetration)
            && CheckConvexMeshCuboidAxis(meshVertices, cuboidVertices, Vector3d.Cross(meshEdge, cuboid.EdgeDirections[8]), meshToCuboid, ref penetration);
    }

    private static bool CheckConvexMeshCuboidAxis(
        Vector3d[] meshVertices,
        Vector3d[] cuboidVertices,
        Vector3d axis,
        Vector3d meshToCuboid,
        ref AxisPenetration penetration)
        => CheckVertexProjectionAxis(meshVertices, cuboidVertices, axis, meshToCuboid, ref penetration);

    private static (Vector3d Point1, Vector3d Point2) FindInitialPointsOfContact(LSMeshCollider mesh, LSCuboidCollider cuboid)
    {
        Vector3d cuboidBoundPoint = cuboid.Bounds.GetPointOnSurfaceTowardsObject(mesh.Position);
        if (cuboid.CurrentState == CuboidState.OOBox)
            cuboidBoundPoint = cuboid.ClosestPointOnSurface(cuboidBoundPoint);

        Vector3d meshBoundPoint = mesh.Bounds.GetPointOnSurfaceTowardsObject(cuboidBoundPoint);
        Vector3d pointOfContactA = mesh.ClosestPointOnSurface(meshBoundPoint);
        if (cuboid.Bounds.Contains(pointOfContactA))
            pointOfContactA = cuboidBoundPoint;

        return (pointOfContactA, cuboidBoundPoint);
    }

    /// <summary>
    /// Checks if two mesh colliders intersect and sets the contact point if they do.
    /// </summary>
    /// <param name="pair">The pair of colliders to test for collision.</param>
    /// <returns>true if the colliders intersect; otherwise, false.</returns>
    private static bool DoMeshesCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is LSMeshCollider meshA
            && pair.ColliderB is LSMeshCollider meshB
            && (meshA.Mode == MeshColliderMode.Concave || meshB.Mode == MeshColliderMode.Concave))
        {
            return MeshTriangleContactGenerator.TryBuildMeshMeshManifold(
                pair,
                meshA,
                meshB,
                pair.Context.CollisionScratch.MeshTriangleCandidatesA,
                pair.Context.CollisionScratch.MeshTriangleCandidatesB);
        }

        // Test for intersection between the meshes using separating axis theorem
        if (!TestMeshColliders(pair, out CollisionResult output))
            return false;

        // Set the contact point information if collision detected
        pair.Manifold.SetContact(
            output.PointsOfContact.Point1,
            output.PointsOfContact.Point2,
            output.AxisPenetration.Depth,
            output.AxisPenetration.Vector.Normalized
        );

        return true;
    }

    private static bool TestMeshColliders(CollisionWorkItem pair, out CollisionResult output)
    {
        output = default;

        var mesh1 = (LSMeshCollider)pair.ColliderA;
        var mesh2 = (LSMeshCollider)pair.ColliderB;

        (Vector3d Point1, Vector3d Point2) = FindInitialPointsOfContact(mesh1, mesh2);
        if (!TryFindConvexMeshMeshPenetration(mesh1, mesh2, out AxisPenetration penetration))
            return false;

        output = new CollisionResult(
            (Point1, Point2),
            (penetration.Axis, penetration.Depth));

        return true;
    }

    private static bool TryFindConvexMeshMeshPenetration(
        LSMeshCollider meshA,
        LSMeshCollider meshB,
        out AxisPenetration penetration)
    {
        penetration = default;
        PhysicsMesh physicsMeshA = meshA.Mesh;
        PhysicsMesh physicsMeshB = meshB.Mesh;
        Vector3d[] verticesA = physicsMeshA.Vertices;
        Vector3d[] verticesB = physicsMeshB.Vertices;
        Vector3d displacementAtoB = meshB.Center - meshA.Center;

        if (!CheckConvexMeshFaceAxes(physicsMeshA, verticesA, verticesB, displacementAtoB, ref penetration)
            || !CheckConvexMeshFaceAxes(physicsMeshB, verticesA, verticesB, displacementAtoB, ref penetration))
        {
            return false;
        }

        int[] edgeVertexPairsA = physicsMeshA.ConvexSatEdgeVertexPairs;
        int[] edgeVertexPairsB = physicsMeshB.ConvexSatEdgeVertexPairs;
        for (int a = 0; a < edgeVertexPairsA.Length; a += 2)
        {
            Vector3d edgeA = verticesA[edgeVertexPairsA[a + 1]] - verticesA[edgeVertexPairsA[a]];
            for (int b = 0; b < edgeVertexPairsB.Length; b += 2)
            {
                Vector3d edgeB = verticesB[edgeVertexPairsB[b + 1]] - verticesB[edgeVertexPairsB[b]];
                if (!CheckVertexProjectionAxis(verticesA, verticesB, Vector3d.Cross(edgeA, edgeB), displacementAtoB, ref penetration))
                    return false;
            }
        }

        return penetration.HasValue;
    }

    private static bool CheckConvexMeshFaceAxes(
        PhysicsMesh axisSource,
        Vector3d[] verticesA,
        Vector3d[] verticesB,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        for (int i = 0; i < axisSource.TriangleCount; i++)
        {
            if (!CheckVertexProjectionAxis(verticesA, verticesB, axisSource.GetFaceNormalWorld(i), displacementAtoB, ref penetration))
                return false;
        }

        return true;
    }

    private static (Vector3d Point1, Vector3d Point2) FindInitialPointsOfContact(LSMeshCollider mesh1, LSMeshCollider mesh2)
    {
        // Find the closest points on the actual mesh surface, not just the bounding box
        Vector3d support1 = mesh1.GetSupportPoint(mesh2.Center);
        Vector3d pointOfContactA = mesh1.ClosestPointOnSurface(support1);
        return (pointOfContactA, mesh2.ClosestPointOnSurface(pointOfContactA));
    }

    #endregion
}
