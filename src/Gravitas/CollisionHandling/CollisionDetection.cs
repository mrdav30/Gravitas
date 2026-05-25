using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;
using SwiftCollections.Query;

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
            CollisionType.Cylinder_Sphere => DoCylinderSphereCheck(pair),
            CollisionType.Cylinder_Capsule => DoCylinderCapsuleCheck(pair),
            CollisionType.Cylinder_Cylinder => DoCylindersCheck(pair),
            CollisionType.Cuboid_Cylinder => DoCuboidCylinderCheck(pair),
            CollisionType.Mesh_Sphere => DoMeshSphereCheck(pair),
            CollisionType.Mesh_Capsule => DoMeshCapsuleCheck(pair),
            CollisionType.Mesh_Cuboid => DoMeshCuboidCheck(pair),
            CollisionType.Mesh_Cylinder => DoMeshCylinderCheck(pair),
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

        Vector3d penetrationNormal = ResolveNormal(penetrationVector, sphere.Center - capsule.Center);
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

        (Vector3d, Vector3d) closestPointsOnCapsules = ClosestPointsOnSegments(
            capsule1.LineSegmentStart,
            capsule1.LineSegmentEnd,
            capsule2.LineSegmentStart,
            capsule2.LineSegmentEnd);
        Vector3d centerDelta = closestPointsOnCapsules.Item2 - closestPointsOnCapsules.Item1;
        Fixed64 radiusSum = capsule1.ScaledRadius + capsule2.ScaledRadius;
        if (centerDelta.SqrMagnitude > radiusSum * radiusSum)
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        Fixed64 distance = centerDelta.Magnitude;
        Vector3d penetrationNormal = distance > Fixed64.Epsilon
            ? centerDelta / distance
            : Vector3d.Right;
        Vector3d collisionPointCapsule1 = closestPointsOnCapsules.Item1 + penetrationNormal * capsule1.ScaledRadius;
        Vector3d collisionPointCapsule2 = closestPointsOnCapsules.Item2 - penetrationNormal * capsule2.ScaledRadius;
        pair.ContactPoint.SetContactPoint(
            collisionPointCapsule1,
            collisionPointCapsule2,
            radiusSum - distance,
            penetrationNormal
        );
        return true;
    }

    private static (Vector3d First, Vector3d Second) ClosestPointsOnSegments(
        Vector3d firstStart,
        Vector3d firstEnd,
        Vector3d secondStart,
        Vector3d secondEnd)
    {
        bool firstDegenerate = (firstEnd - firstStart).SqrMagnitude <= Fixed64.Epsilon;
        bool secondDegenerate = (secondEnd - secondStart).SqrMagnitude <= Fixed64.Epsilon;

        if (firstDegenerate && secondDegenerate)
            return (firstStart, secondStart);

        if (firstDegenerate)
            return (firstStart, Vector3d.ClosestPointOnLineSegment(firstStart, secondStart, secondEnd));

        if (secondDegenerate)
            return (Vector3d.ClosestPointOnLineSegment(secondStart, firstStart, firstEnd), secondStart);

        return Vector3d.ClosestPointsOnTwoLines(firstStart, firstEnd, secondStart, secondEnd);
    }

    #endregion

    #region Cuboid

    private static bool DoCuboidSphereCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCuboidCollider cuboid || pair.ColliderB is not LSSphereCollider)
            return false;

        // Calculate the closest point the AABB to the sphere center
        Vector3d closetPointOnBox = cuboid.ClosestPointOnSurface(pair.ColliderB.Center);

        Vector3d penetrationVector = pair.ColliderB.Center - closetPointOnBox;
        // Check if the distance from the closest point to the AABB is less than the capsule radius
        if (penetrationVector.SqrMagnitude > pair.ColliderB.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove sphere's radius to find the actual depth

        Vector3d penetrationNormal = ResolveNormal(penetrationVector, pair.ColliderB.Center - cuboid.Center);
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
        Vector3d penetrationNormal = ResolveNormal(penetrationVector, capsule.Center - aabb.Center);
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
            OrientNormal(axisPenetration.Value.Vector, capsule.Center - obb.Center)
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

        if (!TestCuboidsSeperatingAxes(pair.Context.CollisionScratch, cuboidA, cuboidB, out CollisionResult? output))
            return false;

        if (!output.HasValue) return false;
        pair.ContactPoint.SetContactPoint(
            output.Value.PointsOfContact.Point1,
            output.Value.PointsOfContact.Point2,
            output.Value.AxisPenetration.Depth,
            OrientNormal(output.Value.AxisPenetration.Vector, cuboidB.Center - cuboidA.Center)
        ); ;
        return true;
    }

    /// <summary>
    /// Tests if there are any separating axes between two polygons using the given axis vectors.
    /// </summary>
    /// <param name="scratch">The context-owned SAT scratch buffers.</param>
    /// <param name="cuboidA">The first collider.</param>
    /// <param name="cuboidB">The second collider.</param>
    /// <param name="output">The resulting collision information if a collision is detected.</param>
    /// <returns>true if no separating axis is found, false otherwise.</returns>
    private static bool TestCuboidsSeperatingAxes(
        CollisionSatScratch scratch,
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB,
        out CollisionResult? output)
    {
        output = null;

        (Vector3d PointA, Vector3d PointB) = FindInitialPointsOfContact(cuboidA, cuboidB);

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

        CollisionContext data = scratch.PrepareCuboids(cuboidA, PointA, cuboidB, PointB);
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

    #region Cylinder

    private static bool DoCylinderSphereCheck(CollisionPair pair)
    {
        if (!TryGetPairColliders(pair, out LSCylinderCollider cylinder, out LSSphereCollider sphere))
            return false;

        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(sphere.Center);
        Vector3d delta = sphere.Center - cylinderPoint;
        if (delta.SqrMagnitude > sphere.ScaledRadiusSqr)
            return false;

        Fixed64 distance = delta.Magnitude;
        Vector3d normal = ResolveNormal(delta, sphere.Center - cylinder.Center);
        Vector3d spherePoint = sphere.Center - normal * sphere.ScaledRadius;
        SetContactPointInPairOrder(
            pair,
            cylinder,
            cylinderPoint,
            sphere,
            spherePoint,
            sphere.ScaledRadius - distance,
            normal);

        return true;
    }

    private static bool DoCylinderCapsuleCheck(CollisionPair pair)
    {
        if (!TryGetPairColliders(pair, out LSCylinderCollider cylinder, out LSCapsuleCollider capsule))
            return false;

        if (!TestCylinderCapsuleSeparatingAxes(cylinder, capsule, out AxisPenetration penetration))
            return false;

        Vector3d capsuleLinePoint = Vector3d.ClosestPointOnLineSegment(
            cylinder.Center,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd);
        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(capsuleLinePoint);
        Vector3d capsulePoint = capsule.ClosestPointOnSurface(cylinderPoint);
        SetContactPointInPairOrder(
            pair,
            cylinder,
            cylinderPoint,
            capsule,
            capsulePoint,
            penetration.Depth,
            penetration.Axis);

        return true;
    }

    private static bool DoCylindersCheck(CollisionPair pair)
    {
        if (pair.ColliderA is not LSCylinderCollider cylinderA || pair.ColliderB is not LSCylinderCollider cylinderB)
            return false;

        if (!TestCylinderCylinderSeparatingAxes(cylinderA, cylinderB, out AxisPenetration penetration))
            return false;

        Vector3d cylinderAPoint = cylinderA.ClosestPointOnSurface(cylinderB.Center);
        Vector3d cylinderBPoint = cylinderB.ClosestPointOnSurface(cylinderAPoint);
        pair.ContactPoint.SetContactPoint(
            cylinderAPoint,
            cylinderBPoint,
            penetration.Depth,
            penetration.Axis);

        return true;
    }

    private static bool DoCuboidCylinderCheck(CollisionPair pair)
    {
        if (!TryGetPairColliders(pair, out LSCuboidCollider cuboid, out LSCylinderCollider cylinder))
            return false;

        if (!TestCuboidCylinderSeparatingAxes(cuboid, cylinder, out AxisPenetration penetration))
            return false;

        Vector3d cuboidPoint = cuboid.ClosestPointOnSurface(cylinder.Center);
        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(cuboidPoint);
        SetContactPointInPairOrder(
            pair,
            cuboid,
            cuboidPoint,
            cylinder,
            cylinderPoint,
            penetration.Depth,
            penetration.Axis);

        return true;
    }

    private static bool TestCylinderCapsuleSeparatingAxes(
        LSCylinderCollider cylinder,
        LSCapsuleCollider capsule,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCylinderCapsuleAxis(cylinder, capsule, cylinder.LineDirection, ref penetration))
            return false;

        if (!CheckCylinderCapsuleAxis(cylinder, capsule, capsule.LineDirection, ref penetration))
            return false;

        Vector3d crossAxis = Vector3d.Cross(cylinder.LineDirection, capsule.LineDirection);
        if (!CheckCylinderCapsuleAxis(cylinder, capsule, crossAxis, ref penetration))
            return false;

        (Vector3d CylinderPoint, Vector3d CapsulePoint) closestPoints = ClosestPointsOnSegments(
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd);
        if (!CheckCylinderCapsuleAxis(cylinder, capsule, closestPoints.CapsulePoint - closestPoints.CylinderPoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TestCylinderCylinderSeparatingAxes(
        LSCylinderCollider cylinderA,
        LSCylinderCollider cylinderB,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCylinderCylinderAxis(cylinderA, cylinderB, cylinderA.LineDirection, ref penetration))
            return false;

        if (!CheckCylinderCylinderAxis(cylinderA, cylinderB, cylinderB.LineDirection, ref penetration))
            return false;

        Vector3d crossAxis = Vector3d.Cross(cylinderA.LineDirection, cylinderB.LineDirection);
        if (!CheckCylinderCylinderAxis(cylinderA, cylinderB, crossAxis, ref penetration))
            return false;

        (Vector3d PointA, Vector3d PointB) closestPoints = ClosestPointsOnSegments(
            cylinderA.LineSegmentStart,
            cylinderA.LineSegmentEnd,
            cylinderB.LineSegmentStart,
            cylinderB.LineSegmentEnd);
        if (!CheckCylinderCylinderAxis(cylinderA, cylinderB, closestPoints.PointB - closestPoints.PointA, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TestCuboidCylinderSeparatingAxes(
        LSCuboidCollider cuboid,
        LSCylinderCollider cylinder,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCuboidCylinderAxis(cuboid, cylinder, cylinder.LineDirection, ref penetration))
            return false;

        for (int i = 0; i < cuboid.FaceNormals.Length; i++)
        {
            if (!CheckCuboidCylinderAxis(cuboid, cylinder, cuboid.FaceNormals[i], ref penetration))
                return false;
        }

        for (int i = 0; i < cuboid.EdgeDirections.Length; i++)
        {
            Vector3d crossAxis = Vector3d.Cross(cuboid.EdgeDirections[i], cylinder.LineDirection);
            if (!CheckCuboidCylinderAxis(cuboid, cylinder, crossAxis, ref penetration))
                return false;
        }

        Vector3d linePoint = Vector3d.ClosestPointOnLineSegment(
            cuboid.Center,
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd);
        Vector3d cuboidPoint = cuboid.ClosestPointOnSurface(linePoint);
        if (!CheckCuboidCylinderAxis(cuboid, cylinder, linePoint - cuboidPoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool CheckCylinderCapsuleAxis(
        LSCylinderCollider cylinder,
        LSCapsuleCollider capsule,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange cylinderProjection = AxisProjectionHelper.ProjectCylinderOntoAxis(
            normalizedAxis,
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd,
            cylinder.LineDirection,
            cylinder.ScaledRadius);
        FixedRange capsuleProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(
            normalizedAxis,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd,
            capsule.ScaledRadius);

        return CheckProjectedAxis(
            cylinderProjection,
            capsuleProjection,
            normalizedAxis,
            capsule.Center - cylinder.Center,
            ref penetration);
    }

    private static bool CheckCylinderCylinderAxis(
        LSCylinderCollider cylinderA,
        LSCylinderCollider cylinderB,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange projectionA = AxisProjectionHelper.ProjectCylinderOntoAxis(
            normalizedAxis,
            cylinderA.LineSegmentStart,
            cylinderA.LineSegmentEnd,
            cylinderA.LineDirection,
            cylinderA.ScaledRadius);
        FixedRange projectionB = AxisProjectionHelper.ProjectCylinderOntoAxis(
            normalizedAxis,
            cylinderB.LineSegmentStart,
            cylinderB.LineSegmentEnd,
            cylinderB.LineDirection,
            cylinderB.ScaledRadius);

        return CheckProjectedAxis(
            projectionA,
            projectionB,
            normalizedAxis,
            cylinderB.Center - cylinderA.Center,
            ref penetration);
    }

    private static bool CheckCuboidCylinderAxis(
        LSCuboidCollider cuboid,
        LSCylinderCollider cylinder,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange cuboidProjection = FixedRange.MinRange;
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, cuboid.Vertices, ref cuboidProjection);
        FixedRange cylinderProjection = AxisProjectionHelper.ProjectCylinderOntoAxis(
            normalizedAxis,
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd,
            cylinder.LineDirection,
            cylinder.ScaledRadius);

        return CheckProjectedAxis(
            cuboidProjection,
            cylinderProjection,
            normalizedAxis,
            cylinder.Center - cuboid.Center,
            ref penetration);
    }

    private static bool CheckProjectedAxis(
        FixedRange projectionA,
        FixedRange projectionB,
        Vector3d axis,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        if (!projectionA.Overlaps(projectionB))
            return false;

        Fixed64 depth = ComputeMinimumProjectionOverlap(projectionA, projectionB);
        if (!penetration.HasValue || depth < penetration.Depth)
        {
            Vector3d orientedAxis = Vector3d.Dot(axis, displacementAtoB) < Fixed64.Zero ? -axis : axis;
            penetration = new AxisPenetration(orientedAxis, depth);
        }

        return true;
    }

    private static Fixed64 ComputeMinimumProjectionOverlap(FixedRange projectionA, FixedRange projectionB)
    {
        Fixed64 pushALeft = projectionA.Max - projectionB.Min;
        Fixed64 pushARight = projectionB.Max - projectionA.Min;
        Fixed64 overlap = FixedMath.Min(pushALeft, pushARight);
        return overlap > Fixed64.Zero ? overlap : Fixed64.Zero;
    }

    private static bool TryNormalizeAxis(Vector3d axis, out Vector3d normalizedAxis)
    {
        Fixed64 magnitudeSqr = axis.SqrMagnitude;
        if (magnitudeSqr <= Fixed64.Epsilon)
        {
            normalizedAxis = Vector3d.Zero;
            return false;
        }

        normalizedAxis = axis / FixedMath.Sqrt(magnitudeSqr);
        return true;
    }

    private static Vector3d ResolveNormal(Vector3d delta, Vector3d fallback)
    {
        if (delta.SqrMagnitude > Fixed64.Epsilon)
            return delta.Normal;

        if (fallback.SqrMagnitude > Fixed64.Epsilon)
            return fallback.Normal;

        return Vector3d.Right;
    }

    private static Vector3d OrientNormal(Vector3d normal, Vector3d desiredDirection)
    {
        Vector3d resolved = ResolveNormal(normal, desiredDirection);
        return Vector3d.Dot(resolved, desiredDirection) < Fixed64.Zero ? -resolved : resolved;
    }

    private static bool TryGetPairColliders<TFirst, TSecond>(
        CollisionPair pair,
        out TFirst first,
        out TSecond second)
        where TFirst : LSCollider
        where TSecond : LSCollider
    {
        if (pair.ColliderA is TFirst firstA && pair.ColliderB is TSecond secondB)
        {
            first = firstA;
            second = secondB;
            return true;
        }

        if (pair.ColliderA is TSecond secondA && pair.ColliderB is TFirst firstB)
        {
            first = firstB;
            second = secondA;
            return true;
        }

        first = null!;
        second = null!;
        return false;
    }

    private static void SetContactPointInPairOrder(
        CollisionPair pair,
        LSCollider first,
        Vector3d pointOnFirst,
        LSCollider second,
        Vector3d pointOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond)
    {
        if (ReferenceEquals(pair.ColliderA, first))
        {
            pair.ContactPoint.SetContactPoint(pointOnFirst, pointOnSecond, depth, normalFirstToSecond);
            return;
        }

        if (ReferenceEquals(pair.ColliderA, second))
            pair.ContactPoint.SetContactPoint(pointOnSecond, pointOnFirst, depth, -normalFirstToSecond);
    }

    private readonly struct AxisPenetration
    {
        public AxisPenetration(Vector3d axis, Fixed64 depth)
        {
            Axis = axis;
            Depth = depth;
            HasValue = true;
        }

        public Vector3d Axis { get; }

        public Fixed64 Depth { get; }

        public bool HasValue { get; }
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

    private static bool DoMeshCylinderCheck(CollisionPair pair)
    {
        if (!TryGetPairColliders(pair, out LSMeshCollider mesh, out LSCylinderCollider cylinder))
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

        SetContactPointInPairOrder(pair, mesh, pointOnMesh, cylinder, pointOnCylinder, depth, normalMeshToCylinder);
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
        Fixed64 bestDepth = Fixed64.MAX_VALUE;

        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            int triangleIndex = triangleBuffer[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d faceNormal = mesh.Mesh.FaceNormals[triangleIndex];
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
    private static bool TestMeshCuboidColliders(CollisionPair pair, out CollisionResult? output)
    {
        output = null;

        if (pair.ColliderA is not LSMeshCollider mesh || pair.ColliderB is not LSCuboidCollider cuboid)
            return false;

        (Vector3d PointA, Vector3d PointB) = FindInitialPointsOfContact(mesh, cuboid);
        if (!pair.Context.CollisionScratch.TryPrepareMeshCuboid(mesh, PointA, cuboid, PointB, out CollisionContext data))
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

    private static bool IsPointInsideCylinder(LSCylinderCollider cylinder, Vector3d point)
    {
        Vector3d local = cylinder.Rotation.Inverse() * (point - cylinder.Center);
        Fixed64 radialSqr = local.x * local.x + local.z * local.z;
        return radialSqr <= cylinder.ScaledRadiusSqr + Fixed64.Epsilon
            && local.y >= -cylinder.HalfHeight - Fixed64.Epsilon
            && local.y <= cylinder.HalfHeight + Fixed64.Epsilon;
    }

    #endregion
}
