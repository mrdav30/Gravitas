//=======================================================================
// CollisionDetection.Mesh.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

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
        if (pair.ColliderA is not LSMeshCollider meshCollider || pair.ColliderB is not LSSphereCollider sphere)
            return false;

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
        if (pair.ColliderA is not LSMeshCollider mesh || pair.ColliderB is not LSCapsuleCollider capsule)
            return false;

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
        if (!TryGetPairColliders(pair, out LSMeshCollider mesh, out LSCuboidCollider cuboid))
            return false;

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

        if (!TestMeshCuboidColliders(pair, mesh, cuboid, out CollisionResult? output))
            return false;

        if (!output.HasValue) return false; // Check if axisPenetration was found
        SetContactInPairOrder(
            pair,
            mesh,
            output.Value.PointsOfContact.Point1,
            cuboid,
            output.Value.PointsOfContact.Point2,
            output.Value.AxisPenetration.Depth,
            output.Value.AxisPenetration.Vector.Normalized);

        return true;
    }

    private static bool DoMeshCylinderCheck(CollisionWorkItem pair)
    {
        if (!TryGetPairColliders(pair, out LSMeshCollider mesh, out LSCylinderCollider cylinder))
            return false;

        if (MeshTriangleContactGenerator.TryBuildMeshCylinderManifold(
            pair,
            mesh,
            cylinder,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA))
        {
            return true;
        }

        if (mesh.Mode == MeshColliderMode.Concave)
            return false;

        SwiftList<int> nearbyTriangles = pair.Context.CollisionScratch.MeshCylinderTriangles;
        if (!TryFindMeshCylinderContact(
            mesh,
            cylinder,
            nearbyTriangles,
            out Vector3d pointOnMesh,
            out Vector3d pointOnCylinder,
            out Vector3d normalMeshToCylinder,
            out Fixed64 depth))
        {
            return false;
        }

        SetContactInPairOrder(pair, mesh, pointOnMesh, cylinder, pointOnCylinder, depth, normalMeshToCylinder);
        return true;
    }

    private static bool TryFindMeshCylinderContact(
        LSMeshCollider mesh,
        LSCylinderCollider cylinder,
        SwiftList<int> triangleBuffer,
        out Vector3d pointOnMesh,
        out Vector3d pointOnCylinder,
        out Vector3d normalMeshToCylinder,
        out Fixed64 depth)
    {
        pointOnMesh = Vector3d.Zero;
        pointOnCylinder = Vector3d.Zero;
        normalMeshToCylinder = Vector3d.Zero;
        depth = Fixed64.Zero;

        mesh.GetTrianglesInBounds(new FixedBoundVolume(cylinder.BoundsMin, cylinder.BoundsMax), triangleBuffer);
        bool found = false;
        Fixed64 bestDepth = Fixed64.MaxValue;

        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            int triangleIndex = triangleBuffer[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d faceNormal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            Vector3d candidatePointOnMesh = MeshUtils.ClosestPointOnTriangle(first, second, third, faceNormal, cylinder.Center);
            if (!IsPointInsideCylinder(cylinder, candidatePointOnMesh))
                continue;

            Vector3d candidatePointOnCylinder = cylinder.ClosestPointOnSurface(candidatePointOnMesh);
            Fixed64 candidateDepth = Vector3d.Distance(candidatePointOnMesh, candidatePointOnCylinder);
            if (found && candidateDepth >= bestDepth)
                continue;

            found = true;
            bestDepth = candidateDepth;
            pointOnMesh = candidatePointOnMesh;
            pointOnCylinder = candidatePointOnCylinder;
            normalMeshToCylinder = OrientNormal(faceNormal, cylinder.Center - candidatePointOnMesh);
            depth = candidateDepth;
        }

        return found;
    }

    /// <summary>
    /// Tests if there are any separating axes between a cuboid and a mesh using the given axis vectors.
    /// </summary>
    /// <returns>true if no separating axis is found, false otherwise.</returns>
    private static bool TestMeshCuboidColliders(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCuboidCollider cuboid,
        out CollisionResult? output)
    {
        output = null;

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
        int[] meshTriangles = physicsMesh.Triangles;
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

        for (int triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++)
        {
            int triangleOffset = triangleIndex * 3;
            Vector3d first = meshVertices[meshTriangles[triangleOffset]];
            Vector3d second = meshVertices[meshTriangles[triangleOffset + 1]];
            Vector3d third = meshVertices[meshTriangles[triangleOffset + 2]];
            if (!CheckConvexMeshCuboidEdgeAxes(meshVertices, cuboidVertices, cuboid, second - first, meshToCuboid, ref penetration))
                return false;
            if (!CheckConvexMeshCuboidEdgeAxes(meshVertices, cuboidVertices, cuboid, third - second, meshToCuboid, ref penetration))
                return false;
            if (!CheckConvexMeshCuboidEdgeAxes(meshVertices, cuboidVertices, cuboid, first - third, meshToCuboid, ref penetration))
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
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange meshProjection = FixedRange.MinRange;
        FixedRange cuboidProjection = FixedRange.MinRange;
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, meshVertices, ref meshProjection);
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, cuboidVertices, ref cuboidProjection);
        return CheckProjectedAxis(meshProjection, cuboidProjection, normalizedAxis, meshToCuboid, ref penetration);
    }

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
        if (!TestMeshColliders(pair, out CollisionResult? output))
            return false;

        // Check if axisPenetration was found
        if (!output.HasValue) return false;
        // Set the contact point information if collision detected
        pair.Manifold.SetContact(
            output.Value.PointsOfContact.Point1,
            output.Value.PointsOfContact.Point2,
            output.Value.AxisPenetration.Depth,
            output.Value.AxisPenetration.Vector.Normalized
        );

        return true;
    }

    private static bool TestMeshColliders(CollisionWorkItem pair, out CollisionResult? output)
    {
        output = null;

        if (pair.ColliderA is not LSMeshCollider mesh1 || pair.ColliderB is not LSMeshCollider mesh2)
            return false;

        (Vector3d Point1, Vector3d Point2) = FindInitialPointsOfContact(mesh1, mesh2);
        if (!pair.Context.CollisionScratch.TryPrepareMeshes(mesh1, Point1, mesh2, Point2, out CollisionContext data))
        {
            return false;
        }

        if (!PerformSeparatingAxisTest(data, out (Vector3d Vector, Fixed64 Depth)? axisPenetration))
            return false;

        output = new CollisionResult(
            data.PointsOfContact,
            axisPenetration!.Value);

        return true;
    }

    private static (Vector3d Point1, Vector3d Point2) FindInitialPointsOfContact(LSMeshCollider mesh1, LSMeshCollider mesh2)
    {
        // Find the closest points on the actual mesh surface, not just the bounding box
        Vector3d support1 = mesh1.GetSupportPoint(mesh2.Center);
        Vector3d pointOfContactA = mesh1.ClosestPointOnSurface(support1);
        return (pointOfContactA, mesh2.ClosestPointOnSurface(pointOfContactA));
    }

    private static bool PerformSeparatingAxisTest(
        CollisionContext data,
        out (Vector3d Vector, Fixed64 Depth)? axisPenetration)
    {
        axisPenetration = null;
        if (data.AxisVectors.Count <= 0)
            return false;

        FixedRange projectionA = FixedRange.MinRange, projectionB = FixedRange.MinRange;
        foreach (Vector3d axis in data.AxisVectors)
        {
            AxisProjectionHelper.ProjectPolygonOntoAxis(axis, data.CollisionInfoA.UniqueVertices, ref projectionA);
            AxisProjectionHelper.ProjectPolygonOntoAxis(axis, data.CollisionInfoB.UniqueVertices, ref projectionB);
            if (!projectionA.Overlaps(projectionB))
                return false;  //  Found seperating axis, no collision.
            Fixed64 overlap = FixedRange.ComputeOverlapDepth(projectionA, projectionB);
            // Update if this axis is the axis of minimum penetration
            if (!axisPenetration.HasValue || overlap < axisPenetration.Value.Depth)
                axisPenetration = (Vector3d.Dot(data.Displacement, axis) < Fixed64.Zero ? -axis : axis, overlap);  // Flip the direction if it's pointing the wrong way.
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointInsideCylinder(LSCylinderCollider cylinder, Vector3d point)
    {
        Vector3d local = cylinder.Rotation.Inverse() * (point - cylinder.Center);
        Fixed64 radialSqr = local.X * local.X + local.Z * local.Z;
        return radialSqr <= cylinder.ScaledRadiusSqr + Fixed64.Epsilon
            && local.Y >= -cylinder.HalfHeight - Fixed64.Epsilon
            && local.Y <= cylinder.HalfHeight + Fixed64.Epsilon;
    }

    #endregion
}
