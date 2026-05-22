using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;

namespace Gravitas.CollisionHandling;

public static class CollisionDetection
{
    public static bool DoCollisionCheck(CollisionPair pair)
    {
        return pair.CollisionType switch
        {
            CollisionType.None => false,
            CollisionType.Sphere_Sphere => DoSpheresCheck(pair),
            CollisionType.Capsule_Sphere => DoCapsuleSphereCheck(pair),
            CollisionType.Capsule_Capsule => DoCapsulesCheck(pair),
            CollisionType.Cuboid_Sphere => DoCuboidSphereCheck(pair),
            CollisionType.AABox_Capsule => DoAABoxCapsuleCheck(pair),
            CollisionType.OBBox_Capsule => DoOBBoxCapsuleCheck(pair),
            CollisionType.Cuboid_Cuboid => DoCuboidsCheck(pair),
            CollisionType.Mesh_Sphere => DoMeshSphereCheck(pair),
            CollisionType.Mesh_Capsule => DoMeshCapsuleCheck(pair),
            CollisionType.Mesh_Cuboid => DoMeshCuboidCheck(pair),
            CollisionType.Mesh_Mesh => DoMeshesCheck(pair),
            _ => false,
        };
    }

    #region Sphere

    private static bool DoSpheresCheck(CollisionPair pair)
    {
        Vector3d penetrationVector = pair.ColliderB.Center - pair.ColliderA.Center;
        if (penetrationVector.SqrMagnitude > (pair.ColliderA.ScaledRadius + pair.ColliderB.ScaledRadius) * (pair.ColliderA.ScaledRadius + pair.ColliderB.ScaledRadius))
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        // Calculate the penetration depths and normals
        Vector3d penetrationNormal = penetrationVector.Normal;
        pair.ContactPoint.SetContactPoint(
            pair.ColliderA.Center + penetrationNormal * pair.ColliderA.ScaledRadius,
            pair.ColliderB.Center - penetrationNormal * pair.ColliderB.ScaledRadius,
            penetrationVector.Magnitude - (pair.ColliderA.ScaledRadius + pair.ColliderB.ScaledRadius),
            penetrationNormal
        );
        return true;
    }

    #endregion

    #region Capsule

    private static bool DoCapsuleSphereCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCapsuleCollider capsule || pair.ColliderB is not LSSphereCollider sphere)
            return false;

        Vector3d closestPointOnCapsule = capsule.ClosestPointOnSurface(sphere.Center);
        Vector3d penetrationVector = sphere.Center - closestPointOnCapsule;
        // Check if the distance from the sphere center to the closest point is less than the sum of the radii
        if (penetrationVector.SqrMagnitude > sphere.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        Vector3d penetrationNormal = penetrationVector.Normal;
        pair.ContactPoint.SetContactPoint(
            closestPointOnCapsule,
            sphere.Center - penetrationNormal * sphere.ScaledRadius,
            penetrationVector.Magnitude - sphere.ScaledRadius,
            penetrationNormal
        );
        return true;
    }

    private static bool DoCapsulesCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCapsuleCollider capsule1 || pair.ColliderB is not LSCapsuleCollider capsule2)
            return false;

        (Vector3d, Vector3d) closestPointsOnCapsules = Vector3d.ClosestPointsOnTwoLines(
            capsule1.LineSegmentStart,
            capsule1.LineSegmentEnd,
            capsule2.LineSegmentStart,
            capsule2.LineSegmentEnd);
        // Check if the distance between these two points is less than the sum of the radii (indicating a collision)
        if ((closestPointsOnCapsules.Item1 - closestPointsOnCapsules.Item2).SqrMagnitude > (capsule1.ScaledRadius + capsule2.ScaledRadius) * (capsule1.ScaledRadius + capsule2.ScaledRadius))
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        // Calculate the points on the capsule surfaces in the direction of the closest points
        Vector3d collisionPointCapsule1 = capsule1.ClosestPointOnSurface(closestPointsOnCapsules.Item2);
        Vector3d collisionPointCapsule2 = capsule2.ClosestPointOnSurface(closestPointsOnCapsules.Item1);
        // Compute the penetration depth and normal
        Vector3d penetrationVector = collisionPointCapsule1 - collisionPointCapsule2;
        pair.ContactPoint.SetContactPoint(
            collisionPointCapsule1,
            collisionPointCapsule2,
            penetrationVector.Magnitude,
            penetrationVector.Normal
        );
        return true;
    }

    #endregion

    #region Cuboid

    private static bool DoCuboidSphereCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCuboidCollider cuboid || pair.ColliderB is not LSSphereCollider)
            return false;

        // Calculate the closest point the AABB to the sphere center
        Vector3d closetPointOnBox = cuboid.ClosestPointOnSurface(pair.ColliderB.Center);

        // TODO: agnostic implemetation for all checks
        // DebugShapes.DrawDot(closetPointOnBox.ToVector3(), 0.1f, UnityEngine.Color.yellow);

        Vector3d penetrationVector = pair.ColliderB.Center - closetPointOnBox;
        // Check if the distance from the closest point to the AABB is less than the capsule radius
        if (penetrationVector.SqrMagnitude > pair.ColliderB.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove sphere's radius to find the actual depth

        Vector3d penetrationNormal = penetrationVector.Normal;
        // get sphere's contact point by subtracting normal scaled by sphere's radius
        pair.ContactPoint.SetContactPoint(
            closetPointOnBox,
            pair.ColliderB.Center - penetrationNormal * pair.ColliderB.ScaledRadius,
            penetrationVector.Magnitude - pair.ColliderB.ScaledRadius,
            penetrationNormal
        );

        return true;
    }

    private static bool DoAABoxCapsuleCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCuboidCollider aabb || pair.ColliderB is not LSCapsuleCollider capsule)
            return false;

        // Calculate the closest point on the capsule line segment to the AABB center
        Vector3d closestPointOnCapsuleLine = Vector3d.ClosestPointOnLineSegment(capsule.LineSegmentStart, capsule.LineSegmentEnd, aabb.Center);
        Vector3d closestPointOnBox = aabb.ClosestPointOnSurface(closestPointOnCapsuleLine);
        Vector3d penetrationVector = closestPointOnCapsuleLine - closestPointOnBox;
        // Check if the distance from the closest point to the AABB is less than the capsule radius
        if (penetrationVector.SqrMagnitude > capsule.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove capsule's radius to find the actual depth

        // remove the capsule's radius to get the actual penetration depth 
        Vector3d penetrationNormal = penetrationVector.Normal;
        pair.ContactPoint.SetContactPoint(
            closestPointOnBox,
            closestPointOnCapsuleLine - penetrationNormal * capsule.ScaledRadius,
            penetrationVector.Magnitude - capsule.ScaledRadius,
            penetrationNormal
        );

        return true;
    }

    private static bool DoOBBoxCapsuleCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCuboidCollider obb || pair.ColliderB is not LSCapsuleCollider capsule)
            return false;

        if (!TestOBBoxCapsuleSeparatingAxes(obb, capsule, out (Vector3d Vector, Fixed64 Depth)? axisPenetration))
            return false;

        if (!axisPenetration.HasValue) return false;
        // find closest point on capsules inner line segment relative to poly center
        Vector3d closestPointOnCapsuleLine = Vector3d.ClosestPointOnLineSegment(capsule.LineSegmentStart, capsule.LineSegmentEnd, obb.Center);
        // find the closest point on the poly relative to the closest point on capsule's line segment
        Vector3d collisionPointOBBox = obb.ClosestPointOnSurface(closestPointOnCapsuleLine);
        Vector3d collisionPointCapsule = capsule.ClosestPointOnSurface(collisionPointOBBox);
        pair.ContactPoint.SetContactPoint(
            collisionPointOBBox,
            collisionPointCapsule,
            axisPenetration.Value.Depth,
            axisPenetration.Value.Vector.Normal
        );

        return true;
    }

    private static bool TestOBBoxCapsuleSeparatingAxes(
        LSCuboidCollider obb,
        LSCapsuleCollider capsule,
        out (Vector3d Vector, Fixed64 Depth)? output)
    {
        output = null;
        bool overlaps = false;

        SwiftHashSet<Vector3d> axes = SwiftHashSetPool<Vector3d>.Shared.Rent();
        AxisProjectionHelper.GetCuboidAndCapsuleAxisVectors(obb, capsule, ref axes);
        if (axes.Count <= 0)
        {
            SwiftHashSetPool<Vector3d>.Shared.Release(axes);
            return false;
        }

        int ndx = 0;
        Vector3d representativePointB = Vector3d.ClosestPointOnLineSegment(capsule.LineSegmentStart, capsule.LineSegmentEnd, obb.Center);
        // Check for a separating axis
        FixedRange obbProjection = FixedRange.MinRange, capProjection;
        foreach (Vector3d axis in axes)
        {
            AxisProjectionHelper.ProjectPolygonOntoAxis(axis, obb.Vertices, ref obbProjection);
            capProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(axis, capsule.LineSegmentStart, capsule.LineSegmentEnd, capsule.ScaledRadius);
            if (!obbProjection.Overlaps(capProjection))
            {
                overlaps = false;
                break;
            }
            else overlaps = true;

            Vector3d representativePointA = obb.ClosestPointOnSurface(representativePointB);
            // Determine the direction of the overlap
            Fixed64 sign = Vector3d.Dot(axis, representativePointB - representativePointA) < Fixed64.Zero ? -Fixed64.One : Fixed64.One;
            Fixed64 checkDepth = output.HasValue ? output.Value.Depth : Fixed64.MAX_VALUE;
            if (FixedRange.CheckOverlap(axis, obbProjection, capProjection, checkDepth, sign, out (Vector3d Vector, Fixed64 Depth)? axisOverlap))
                output = axisOverlap;
            ndx++;
        }

        SwiftHashSetPool<Vector3d>.Shared.Release(axes);
        return overlaps;
    }

    /// <summary>
    /// Checks for collisions between two poly-poly colliders.
    /// </summary>
    /// <returns>true if a collision is detected, false otherwise.</returns>
    private static bool DoCuboidsCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCuboidCollider cuboidA || pair.ColliderB is not LSCuboidCollider cuboidB)
            return false;

        if (!TestCuboidsSeperatingAxes(cuboidA, cuboidB, out CollisionResult? output))
            return false;

        if (!output.HasValue) return false;
        pair.ContactPoint.SetContactPoint(
            output.Value.PointsOfContact.Point1,
            output.Value.PointsOfContact.Point2,
            output.Value.AxisPenetration.Depth,
            output.Value.AxisPenetration.Vector
        ); ;
        return true;
    }

    /// <summary>
    /// Tests if there are any separating axes between two polygons using the given axis vectors.
    /// </summary>
    /// <param name="cuboidA">The first collider.</param>
    /// <param name="cuboidB">The second collider.</param>
    /// <param name="output">The resulting collision information if a collision is detected.</param>
    /// <returns>true if no separating axis is found, false otherwise.</returns>
    private static bool TestCuboidsSeperatingAxes(
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB,
        out CollisionResult? output)
    {
        output = null;

        (Vector3d PointA, Vector3d PointB) = FindInitialPointsOfContact(cuboidA, cuboidB);
        (CuboidObjectInfo CollisionInfoA, CuboidObjectInfo CollisionInfoB) = (
            new CuboidObjectInfo(cuboidA, PointA),
            new CuboidObjectInfo(cuboidB, PointB)
        );

        // check if we're dealing with 2 AABoxes
        if (cuboidA.CurrentState == CuboidState.AABox && cuboidB.CurrentState == CuboidState.AABox)
        {
            // Calculate the displacement vector and distance between box centers
            Vector3d penetrationVector = cuboidB.Center - cuboidA.Center;
            // Calculate overlap on x, y and z axes
            Fixed64 overlapX = cuboidA.Bounds.Scope.x + cuboidB.Bounds.Scope.x - penetrationVector.x.Abs();
            Fixed64 overlapY = cuboidA.Bounds.Scope.y + cuboidB.Bounds.Scope.y - penetrationVector.y.Abs();
            Fixed64 overlapZ = cuboidA.Bounds.Scope.z + cuboidB.Bounds.Scope.z - penetrationVector.z.Abs();

            // If there's no overlap on an axis, there's no collision
            if (overlapX > Fixed64.Zero && overlapY > Fixed64.Zero && overlapZ > Fixed64.Zero)
            {
                (Vector3d Vector, Fixed64 Depth) boxPenetration;
                // The smallest overlap is the minimum penetration vector
                if (overlapX < overlapY && overlapX < overlapZ)
                    boxPenetration = (new Vector3d(penetrationVector.x.Sign(), 0, 0), overlapX);
                else if (overlapY < overlapZ)
                    boxPenetration = (new Vector3d(0, penetrationVector.y.Sign(), 0), overlapY);
                else
                    boxPenetration = (new Vector3d(0, 0, penetrationVector.z.Sign()), overlapZ);

                output = new CollisionResult(
                    (PointA, PointB),
                    boxPenetration);

                return true;
            }

            // no overlap found
            return false;
        }

        using CollisionContext data = new(CollisionInfoA, CollisionInfoB);
        data.PrepareDataForSAT();
        if (!PerformSeparatingAxisTest(data, out (Vector3d Vector, Fixed64 Depth)? axisPenetration))
            return false;

        output = new CollisionResult(
            data.PointsOfContact,
            axisPenetration!.Value);

        return true;
    }

    private static (Vector3d Point1, Vector3d Point2) FindInitialPointsOfContact(LSCuboidCollider cuboidA, LSCuboidCollider cuboidB)
    {
        return (
            cuboidA.ClosestPointOnSurface(cuboidB.Center),
            cuboidB.ClosestPointOnSurface(cuboidA.Center)
        );
    }

    #endregion

    #region Mesh

    /// <summary>
    /// Assumes collider A is the mesh collider and collider B is the sphere collider.
    /// </summary>
    /// <param name="pair"></param>
    /// <returns>True if colliders intersect, otherwise false</returns>
    public static bool DoMeshSphereCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSMeshCollider meshCollider || pair.ColliderB is not LSSphereCollider)
            return false;

        Vector3d closestPointOnMesh = meshCollider.ClosestPointOnSurface(pair.ColliderB.Center);
        Vector3d penetrationVector = pair.ColliderB.Center - closestPointOnMesh;
        if (penetrationVector.SqrMagnitude > pair.ColliderB.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove sphere's radius to find the actual depth

        Vector3d penetrationNormal = penetrationVector.Normal;
        pair.ContactPoint.SetContactPoint(
            closestPointOnMesh,
            pair.ColliderB.Center - penetrationNormal * pair.ColliderB.ScaledRadius,
            penetrationVector.Magnitude - pair.ColliderB.ScaledRadius,
            penetrationNormal
        );

        return true;
    }

    private static bool DoMeshCapsuleCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSMeshCollider mesh || pair.ColliderB is not LSCapsuleCollider capsule)
            return false;

        // Calculate the closest point on the capsule line segment to the mesh center
        Vector3d closestPointOnCapsuleLine = Vector3d.ClosestPointOnLineSegment(capsule.LineSegmentStart, capsule.LineSegmentEnd, mesh.Center);
        Vector3d closetPointOnMesh = mesh.ClosestPointOnSurface(closestPointOnCapsuleLine);
        // Check if the distance from the closest point to the mesh is less than the capsule radius
        if ((closetPointOnMesh - closestPointOnCapsuleLine).SqrMagnitude > capsule.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove capsule's radius to find the actual depth

        // Use the normal of the triangle where the collision occurs.
        Vector3d penetrationNormal = (closestPointOnCapsuleLine - closetPointOnMesh).Normal;
        // penetration vector should be along the normal direction
        Vector3d penetrationVector = penetrationNormal * (capsule.ScaledRadius - Vector3d.Distance(closestPointOnCapsuleLine, closetPointOnMesh));
        // find collision point on the capsule
        Vector3d collisionPointCapsule = closestPointOnCapsuleLine - penetrationNormal * capsule.ScaledRadius;
        pair.ContactPoint.SetContactPoint(
            closetPointOnMesh,
            collisionPointCapsule,
            penetrationVector.Magnitude,
            penetrationNormal
        );

        return true;
    }

    private static bool DoMeshCuboidCheck(CollisionPair pair)
    {
        if (!TestMeshCuboidColliders(pair, out CollisionResult? output))
            return false;

        if (!output.HasValue) return false; // Check if axisPenetration was found
        pair.ContactPoint.SetContactPoint(
            output.Value.PointsOfContact.Point1,
            output.Value.PointsOfContact.Point2,
            output.Value.AxisPenetration.Depth,
            output.Value.AxisPenetration.Vector.Normal
        );

        return true;
    }

    /// <summary>
    /// Tests if there are any separating axes between a cuboid and a mesh using the given axis vectors.
    /// </summary>
    /// <returns>true if no separating axis is found, false otherwise.</returns>
    private static bool TestMeshCuboidColliders(CollisionPair pair, out CollisionResult? output)
    {
        output = null;

        if (pair.ColliderA is not LSMeshCollider mesh || pair.ColliderB is not LSCuboidCollider cuboid)
            return false;

        if (!GetCollisionInfo(mesh, cuboid,
            out (MeshObjectInfo CollisionInfoA, CuboidObjectInfo CollisionInfoB)? collisionInfo))
        {
            return false;
        }

        using CollisionContext data = new(collisionInfo!.Value.CollisionInfoA, collisionInfo.Value.CollisionInfoB);
        data.PrepareDataForSAT();
        if (!PerformSeparatingAxisTest(data, out (Vector3d Vector, Fixed64 Depth)? axisPenetration))
            return false;

        output = new CollisionResult(
            data.PointsOfContact,
            axisPenetration!.Value);

        return true;
    }

    private static bool GetCollisionInfo(
        LSMeshCollider mesh,
        LSCuboidCollider cuboid,
        out (MeshObjectInfo MeshCollisionInfoA, CuboidObjectInfo MeshCollisionInfoB)? collisionInfo)
    {
        collisionInfo = null;

        (Vector3d PointA, Vector3d PointB) = FindInitialPointsOfContact(mesh, cuboid);
        SwiftList<int> nearbyMeshTriangles = mesh.GetNearbyTriangles(PointA);
        if (nearbyMeshTriangles.Count == 0)
            return false;

        collisionInfo = (
            new MeshObjectInfo(mesh, PointA, nearbyMeshTriangles),
            new CuboidObjectInfo(cuboid, PointB)
        );
        return true;
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
    private static bool DoMeshesCheck(CollisionPair pair)
    {
        // Test for intersection between the meshes using separating axis theorem
        if (!TestMeshColliders(pair, out CollisionResult? output))
            return false;

        // Check if axisPenetration was found
        if (!output.HasValue) return false;
        // Set the contact point information if collision detected
        pair.ContactPoint.SetContactPoint(
            output.Value.PointsOfContact.Point1,
            output.Value.PointsOfContact.Point2,
            output.Value.AxisPenetration.Depth,
            output.Value.AxisPenetration.Vector.Normal
        );

        return true;
    }

    private static bool TestMeshColliders(CollisionPair pair, out CollisionResult? output)
    {
        output = null;

        if (pair.ColliderA is not LSMeshCollider mesh1 || pair.ColliderB is not LSMeshCollider mesh2)
            return false;

        if (!GetCollisionInfo(mesh1, mesh2,
            out (MeshObjectInfo CollisionInfoA, MeshObjectInfo CollisionInfoB)? CollisionInfo))
        {
            return false;
        }

        using CollisionContext data = new(CollisionInfo!.Value.CollisionInfoA, CollisionInfo.Value.CollisionInfoB);
        data.PrepareDataForSAT();
        if (!PerformSeparatingAxisTest(data, out (Vector3d Vector, Fixed64 Depth)? axisPenetration))
            return false;

        output = new CollisionResult(
            data.PointsOfContact,
            axisPenetration!.Value);

        return true;
    }

    private static bool GetCollisionInfo(
        LSMeshCollider mesh1,
        LSMeshCollider mesh2,
        out (MeshObjectInfo CollisionInfoA, MeshObjectInfo CollisionInfoB)? collisionInfo)
    {
        collisionInfo = null;
        (Vector3d Point1, Vector3d Point2) = FindInitialPointsOfContact(mesh1, mesh2);

        // Gather nearby triangles for each point of contact
        SwiftList<int> nearbyTriangles1 = mesh1.GetNearbyTriangles(Point1);
        if (nearbyTriangles1.Count <= 0)
            return false;
        SwiftList<int> nearbyTriangles2 = mesh2.GetNearbyTriangles(Point2);
        if (nearbyTriangles2.Count <= 0)
            return false;

        collisionInfo = (
            new MeshObjectInfo(mesh1, Point1, nearbyTriangles1),
            new MeshObjectInfo(mesh2, Point2, nearbyTriangles2)
        );

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

    #endregion
}
