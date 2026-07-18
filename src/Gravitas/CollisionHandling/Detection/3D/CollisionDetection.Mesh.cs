//=======================================================================
// CollisionDetection.Mesh.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Mesh

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

        meshCollider.FindClosestPointOnSurface(
            sphere.Center,
            pair.Context.CollisionScratch.MeshTriangleCandidatesA,
            out Vector3d closest,
            out Vector3d surfaceNormal);
        Vector3d closestPointOnMesh = closest;
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

        Vector3d closestPointOnCapsuleLine = Vector3d.ClosestPointOnLineSegment(
            mesh.Center,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd);
        Vector3d closestPointOnMesh = mesh.ClosestPointOnSurface(closestPointOnCapsuleLine);
        Vector3d separation = closestPointOnCapsuleLine - closestPointOnMesh;
        Fixed64 distanceSquared = separation.MagnitudeSquared;
        bool representativeInside = IsPointInsideClosedConvexMesh(mesh, closestPointOnCapsuleLine);
        if (representativeInside)
        {
            FindConvexMeshCapsulePenetration(mesh, capsule, out AxisPenetration penetration);
            Vector3d containedNormal = penetration.Axis;
            Vector3d capsulePoint = ConvexColliderSupport.Support(capsule, -containedNormal);
            pair.Manifold.SetContact(
                FindMeshSupportFeaturePoint(mesh, containedNormal, capsulePoint),
                capsulePoint,
                penetration.Depth,
                containedNormal);
            return true;
        }

        if (distanceSquared > capsule.ScaledRadiusSqr)
            return MeshTriangleContactGenerator.TryBuildMeshCapsuleManifold(
                pair,
                mesh,
                capsule,
                pair.Context.CollisionScratch.MeshTriangleCandidatesA);

        Fixed64 distance = separation.Magnitude;
        Vector3d normal = ResolveNormal(separation, capsule.Center - mesh.Center);
        pair.Manifold.SetContact(
            closestPointOnMesh,
            closestPointOnCapsuleLine - normal * capsule.ScaledRadius,
            capsule.ScaledRadius - distance,
            normal
        );

        return true;
    }

    private static void FindConvexMeshCapsulePenetration(
        LSMeshCollider mesh,
        LSCapsuleCollider capsule,
        out AxisPenetration penetration)
    {
        penetration = default;
        PhysicsMesh physicsMesh = mesh.Mesh;
        for (int i = 0; i < physicsMesh.TriangleCount; i++)
        {
            Vector3d axis = physicsMesh.GetFaceNormalWorld(i);
            FixedRange meshProjection = ConvexColliderSupport.ProjectOntoAxis(mesh, axis);
            FixedRange capsuleProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(
                axis,
                capsule.LineSegmentStart,
                capsule.LineSegmentEnd,
                capsule.ScaledRadius);
            KeepDirectionalPenetration(axis, meshProjection, capsuleProjection, ref penetration);
        }

        ReadOnlySpan<int> edgeVertexPairs = physicsMesh.ConvexSatEdgeVertexPairs;
        ReadOnlySpan<Vector3d> vertices = physicsMesh.Vertices;
        for (int i = 0; i < edgeVertexPairs.Length; i += 2)
        {
            Vector3d edge = vertices[edgeVertexPairs[i + 1]] - vertices[edgeVertexPairs[i]];
            Vector3d cross = Vector3d.Cross(edge, capsule.LineDirection);
            if (cross.MagnitudeSquared == Fixed64.Zero)
                continue;

            Vector3d axis = cross.Normalized;
            FixedRange meshProjection = ConvexColliderSupport.ProjectOntoAxis(mesh, axis);
            FixedRange capsuleProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(
                axis,
                capsule.LineSegmentStart,
                capsule.LineSegmentEnd,
                capsule.ScaledRadius);
            KeepDirectionalPenetration(axis, meshProjection, capsuleProjection, ref penetration);
        }
    }

    private static Vector3d FindMeshSupportFeaturePoint(
        LSMeshCollider mesh,
        Vector3d direction,
        Vector3d target)
    {
        PhysicsMesh physicsMesh = mesh.Mesh;
        Vector3d supportVertex = ConvexColliderSupport.Support(mesh, direction);
        Fixed64 supportProjection = Vector3d.Dot(supportVertex, direction);
        Vector3d closest = supportVertex;
        Fixed64 closestDistanceSquared = Vector3d.DistanceSquared(target, closest);

        ReadOnlySpan<int> edgeVertexPairs = physicsMesh.ConvexSatEdgeVertexPairs;
        ReadOnlySpan<Vector3d> vertices = physicsMesh.Vertices;
        for (int i = 0; i < edgeVertexPairs.Length; i += 2)
        {
            Vector3d start = vertices[edgeVertexPairs[i]];
            Vector3d end = vertices[edgeVertexPairs[i + 1]];
            if (!IsOnSupportPlane(start, direction, supportProjection)
                || !IsOnSupportPlane(end, direction, supportProjection))
            {
                continue;
            }

            KeepClosestFeaturePoint(
                Vector3d.ClosestPointOnLineSegment(target, start, end),
                target,
                ref closest,
                ref closestDistanceSquared);
        }

        for (int i = 0; i < physicsMesh.TriangleCount; i++)
        {
            physicsMesh.GetTriangleVertices(i, out Vector3d first, out Vector3d second, out Vector3d third);
            if (!IsOnSupportPlane(first, direction, supportProjection)
                || !IsOnSupportPlane(second, direction, supportProjection)
                || !IsOnSupportPlane(third, direction, supportProjection))
            {
                continue;
            }

            KeepClosestFeaturePoint(
                MeshUtils.ClosestPointOnTriangle(
                    first,
                    second,
                    third,
                    physicsMesh.GetFaceNormalWorld(i),
                    target),
                target,
                ref closest,
                ref closestDistanceSquared);
        }

        return closest;
    }

    private static bool IsOnSupportPlane(
        Vector3d point,
        Vector3d direction,
        Fixed64 supportProjection) =>
        (Vector3d.Dot(point, direction) - supportProjection).Abs() <= Fixed64.Epsilon;

    private static void KeepClosestFeaturePoint(
        Vector3d candidate,
        Vector3d target,
        ref Vector3d closest,
        ref Fixed64 closestDistanceSquared)
    {
        Fixed64 candidateDistanceSquared = Vector3d.DistanceSquared(target, candidate);
        if (candidateDistanceSquared >= closestDistanceSquared)
            return;

        closest = candidate;
        closestDistanceSquared = candidateDistanceSquared;
    }

    private static bool IsPointInsideClosedConvexMesh(LSMeshCollider mesh, Vector3d point)
    {
        PhysicsMesh physicsMesh = mesh.Mesh;
        if (!physicsMesh.TryGetClosedVolumeMassProperties(out MeshMassProperties massProperties, out _))
            return false;

        Vector3d interiorPoint = physicsMesh.ConvertScaledLocalToWorld(massProperties.CenterOfMass);
        for (int i = 0; i < physicsMesh.TriangleCount; i++)
        {
            physicsMesh.GetTriangleVertices(i, out Vector3d first, out _, out _);
            Vector3d outwardNormal = OrientNormal(
                physicsMesh.GetFaceNormalWorld(i),
                first - interiorPoint);

            if (Vector3d.Dot(outwardNormal, point - first) > Fixed64.Epsilon)
                return false;
        }

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
        ReadOnlySpan<Vector3d> meshVertices = physicsMesh.Vertices;
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

        ReadOnlySpan<int> edgeVertexPairs = physicsMesh.ConvexSatEdgeVertexPairs;
        for (int i = 0; i < edgeVertexPairs.Length; i += 2)
        {
            Vector3d meshEdge = meshVertices[edgeVertexPairs[i + 1]] - meshVertices[edgeVertexPairs[i]];
            if (!CheckConvexMeshCuboidEdgeAxes(meshVertices, cuboidVertices, cuboid, meshEdge, meshToCuboid, ref penetration))
                return false;
        }

        return penetration.HasValue;
    }

    private static bool CheckConvexMeshCuboidEdgeAxes(
        ReadOnlySpan<Vector3d> meshVertices,
        ReadOnlySpan<Vector3d> cuboidVertices,
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
        ReadOnlySpan<Vector3d> meshVertices,
        ReadOnlySpan<Vector3d> cuboidVertices,
        Vector3d axis,
        Vector3d meshToCuboid,
        ref AxisPenetration penetration)
        => CheckVertexProjectionAxis(meshVertices, cuboidVertices, axis, meshToCuboid, ref penetration);

    private static (Vector3d Point1, Vector3d Point2) FindInitialPointsOfContact(LSMeshCollider mesh, LSCuboidCollider cuboid)
    {
        Vector3d cuboidBoundPoint = cuboid.Bounds.GetPointOnSurfaceTowardsObject(mesh.Position);
        if (cuboid.Shape == ColliderType.OBBox)
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
        var meshA = (LSMeshCollider)pair.ColliderA;
        var meshB = (LSMeshCollider)pair.ColliderB;
        if (meshA.Mode == MeshColliderMode.Concave || meshB.Mode == MeshColliderMode.Concave)
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
        ReadOnlySpan<Vector3d> verticesA = physicsMeshA.Vertices;
        ReadOnlySpan<Vector3d> verticesB = physicsMeshB.Vertices;
        Vector3d displacementAtoB = meshB.Center - meshA.Center;

        if (!CheckConvexMeshFaceAxes(physicsMeshA, verticesA, verticesB, displacementAtoB, ref penetration)
            || !CheckConvexMeshFaceAxes(physicsMeshB, verticesA, verticesB, displacementAtoB, ref penetration))
        {
            return false;
        }

        ReadOnlySpan<int> edgeVertexPairsA = physicsMeshA.ConvexSatEdgeVertexPairs;
        ReadOnlySpan<int> edgeVertexPairsB = physicsMeshB.ConvexSatEdgeVertexPairs;
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
        ReadOnlySpan<Vector3d> verticesA,
        ReadOnlySpan<Vector3d> verticesB,
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
        Vector3d pointOfContactA = mesh1.ClosestPointOnSurface(mesh2.Center);
        return (pointOfContactA, mesh2.ClosestPointOnSurface(pointOfContactA));
    }

    #endregion
}
