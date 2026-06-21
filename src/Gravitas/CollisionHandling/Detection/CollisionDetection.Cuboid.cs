//=======================================================================
// CollisionDetection.Cuboid.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Cuboid

    private static bool DoCuboidSphereCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is not LSCuboidCollider cuboid || pair.ColliderB is not LSSphereCollider)
            return false;

        // Calculate the closest point the AABB to the sphere center
        Vector3d closetPointOnBox = cuboid.ClosestPointOnSurface(pair.ColliderB.Center);

        Vector3d penetrationVector = pair.ColliderB.Center - closetPointOnBox;
        // Check if the distance from the closest point to the AABB is less than the capsule radius
        if (penetrationVector.MagnitudeSquared > pair.ColliderB.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove sphere's radius to find the actual depth

        Vector3d penetrationNormal = ResolveNormal(penetrationVector, pair.ColliderB.Center - cuboid.Center);
        // get sphere's contact point by subtracting normal scaled by sphere's radius
        pair.Manifold.SetContact(
            closetPointOnBox,
            pair.ColliderB.Center - penetrationNormal * pair.ColliderB.ScaledRadius,
            penetrationVector.Magnitude - pair.ColliderB.ScaledRadius,
            penetrationNormal
        );

        return true;
    }

    private static bool DoAABoxCapsuleCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is not LSCuboidCollider aabb || pair.ColliderB is not LSCapsuleCollider capsule)
            return false;

        // Calculate the closest point on the capsule line segment to the AABB center
        Vector3d closestPointOnCapsuleLine = Vector3d.ClosestPointOnLineSegment(capsule.LineSegmentStart, capsule.LineSegmentEnd, aabb.Center);
        Vector3d closestPointOnBox = aabb.ClosestPointOnSurface(closestPointOnCapsuleLine);
        Vector3d penetrationVector = closestPointOnCapsuleLine - closestPointOnBox;
        // Check if the distance from the closest point to the AABB is less than the capsule radius
        if (penetrationVector.MagnitudeSquared > capsule.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the radius squared
                          // remove capsule's radius to find the actual depth

        // remove the capsule's radius to get the actual penetration depth 
        Vector3d penetrationNormal = ResolveNormal(penetrationVector, capsule.Center - aabb.Center);
        pair.Manifold.SetContact(
            closestPointOnBox,
            closestPointOnCapsuleLine - penetrationNormal * capsule.ScaledRadius,
            penetrationVector.Magnitude - capsule.ScaledRadius,
            penetrationNormal
        );

        return true;
    }

    private static bool DoOBBoxCapsuleCheck(CollisionWorkItem pair)
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
        pair.Manifold.SetContact(
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
            Fixed64 checkDepth = output.HasValue ? output.Value.Depth : Fixed64.MaxValue;
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
    private static bool DoCuboidsCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is not LSCuboidCollider cuboidA || pair.ColliderB is not LSCuboidCollider cuboidB)
            return false;

        if (cuboidA.CurrentState == CuboidState.AABox && cuboidB.CurrentState == CuboidState.AABox)
            return TryBuildAxisAlignedCuboidManifold(pair, cuboidA, cuboidB);

        if (!TestCuboidsSeperatingAxes(pair.Context.CollisionScratch, cuboidA, cuboidB, out CollisionResult? output))
            return false;

        if (!output.HasValue) return false;
        pair.Manifold.SetContact(
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

        CollisionContext data = scratch.PrepareCuboids(cuboidA, PointA, cuboidB, PointB);
        if (!PerformSeparatingAxisTest(data, out (Vector3d Vector, Fixed64 Depth)? axisPenetration))
            return false;

        output = new CollisionResult(
            data.PointsOfContact,
            axisPenetration!.Value);

        return true;
    }

    private static bool TryBuildAxisAlignedCuboidManifold(
        CollisionWorkItem pair,
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB)
    {
        Fixed64 overlapX = FixedMath.Min(cuboidA.BoundsMax.X, cuboidB.BoundsMax.X) - FixedMath.Max(cuboidA.BoundsMin.X, cuboidB.BoundsMin.X);
        Fixed64 overlapY = FixedMath.Min(cuboidA.BoundsMax.Y, cuboidB.BoundsMax.Y) - FixedMath.Max(cuboidA.BoundsMin.Y, cuboidB.BoundsMin.Y);
        Fixed64 overlapZ = FixedMath.Min(cuboidA.BoundsMax.Z, cuboidB.BoundsMax.Z) - FixedMath.Max(cuboidA.BoundsMin.Z, cuboidB.BoundsMin.Z);

        if (overlapX < Fixed64.Zero || overlapY < Fixed64.Zero || overlapZ < Fixed64.Zero)
            return false;

        Vector3d centerDelta = cuboidB.Center - cuboidA.Center;
        int axis = 0;
        Fixed64 depth = overlapX;
        if (overlapY < depth)
        {
            axis = 1;
            depth = overlapY;
        }

        if (overlapZ < depth)
        {
            axis = 2;
            depth = overlapZ;
        }

        Vector3d normal = axis switch
        {
            0 => new Vector3d(centerDelta.X < Fixed64.Zero ? -Fixed64.One : Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(Fixed64.Zero, centerDelta.Y < Fixed64.Zero ? -Fixed64.One : Fixed64.One, Fixed64.Zero),
            _ => new Vector3d(Fixed64.Zero, Fixed64.Zero, centerDelta.Z < Fixed64.Zero ? -Fixed64.One : Fixed64.One)
        };

        AddAxisAlignedCuboidContacts(pair.Manifold, cuboidA, cuboidB, axis, depth, normal);
        return pair.Manifold.HasContact;
    }

    private static void AddAxisAlignedCuboidContacts(
        ContactManifold manifold,
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB,
        int axis,
        Fixed64 depth,
        Vector3d normal)
    {
        Fixed64 minX = FixedMath.Max(cuboidA.BoundsMin.X, cuboidB.BoundsMin.X);
        Fixed64 maxX = FixedMath.Min(cuboidA.BoundsMax.X, cuboidB.BoundsMax.X);
        Fixed64 minY = FixedMath.Max(cuboidA.BoundsMin.Y, cuboidB.BoundsMin.Y);
        Fixed64 maxY = FixedMath.Min(cuboidA.BoundsMax.Y, cuboidB.BoundsMax.Y);
        Fixed64 minZ = FixedMath.Max(cuboidA.BoundsMin.Z, cuboidB.BoundsMin.Z);
        Fixed64 maxZ = FixedMath.Min(cuboidA.BoundsMax.Z, cuboidB.BoundsMax.Z);

        switch (axis)
        {
            case 0:
                {
                    Fixed64 x = normal.X > Fixed64.Zero ? cuboidA.BoundsMax.X : cuboidA.BoundsMin.X;
                    AddCuboidContact(manifold, new Vector3d(x, minY, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(x, minY, maxZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(x, maxY, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(x, maxY, maxZ), normal, depth);
                    break;
                }
            case 1:
                {
                    Fixed64 y = normal.Y > Fixed64.Zero ? cuboidA.BoundsMax.Y : cuboidA.BoundsMin.Y;
                    AddCuboidContact(manifold, new Vector3d(minX, y, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(minX, y, maxZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, y, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, y, maxZ), normal, depth);
                    break;
                }
            default:
                {
                    Fixed64 z = normal.Z > Fixed64.Zero ? cuboidA.BoundsMax.Z : cuboidA.BoundsMin.Z;
                    AddCuboidContact(manifold, new Vector3d(minX, minY, z), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(minX, maxY, z), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, minY, z), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, maxY, z), normal, depth);
                    break;
                }
        }
    }

    private static void AddCuboidContact(ContactManifold manifold, Vector3d pointA, Vector3d normal, Fixed64 depth)
    {
        Vector3d pointB = pointA - normal * depth;
        manifold.AddContact(pointA, pointB, depth, normal);
    }

    private static (Vector3d Point1, Vector3d Point2) FindInitialPointsOfContact(LSCuboidCollider cuboidA, LSCuboidCollider cuboidB)
    {
        return (
            cuboidA.ClosestPointOnSurface(cuboidB.Center),
            cuboidB.ClosestPointOnSurface(cuboidA.Center)
        );
    }

    #endregion

}
