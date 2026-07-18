//=======================================================================
// CollisionDetection.Cuboid.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    // Candidate distances are already squared. Equivalent closest features can
    // differ by a few Q32.32 output units after local/world projection, so this
    // is deliberately a squared-distance comparison tolerance.
    private static readonly Fixed64 ClosestSegmentBoxSquaredDistanceTolerance = Fixed64.Epsilon;

    #region Cuboid

    private static bool DoCuboidSphereCheck(CollisionWorkItem pair)
    {
        var cuboid = (LSCuboidCollider)pair.ColliderA;

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

    private static bool DoCuboidCapsuleCheck(CollisionWorkItem pair)
    {
        var cuboid = (LSCuboidCollider)pair.ColliderA;
        var capsule = (LSCapsuleCollider)pair.ColliderB;

        FindClosestPointsOnCuboidAndSegment(
            cuboid,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd,
            out Vector3d pointOnCuboid,
            out Vector3d pointOnSegment);
        Vector3d coreSeparation = pointOnSegment - pointOnCuboid;
        Fixed64 coreDistanceSquared = coreSeparation.MagnitudeSquared;
        if (coreDistanceSquared > capsule.ScaledRadiusSqr)
            return false;

        // A segment transformed into an OBB can retain a few raw units of
        // round-trip residue while geometrically inside the box. Treat that as
        // the penetrating-core path instead of fabricating a radial surface hit.
        if (coreSeparation.X.Abs() > Fixed64.Epsilon
            || coreSeparation.Y.Abs() > Fixed64.Epsilon
            || coreSeparation.Z.Abs() > Fixed64.Epsilon)
        {
            if (!Vector3d.TryGetMagnitude(coreSeparation, out Fixed64 coreDistance)
                || coreDistance > capsule.ScaledRadius)
            {
                return false;
            }

            Vector3d radialNormal = coreSeparation / coreDistance;
            pair.Manifold.SetContact(
                pointOnCuboid,
                pointOnSegment - radialNormal * capsule.ScaledRadius,
                capsule.ScaledRadius - coreDistance,
                radialNormal);
            return true;
        }

        FindCuboidCapsulePenetration(cuboid, capsule, out AxisPenetration axisPenetration);

        Vector3d normal = axisPenetration.Axis;
        if (!TryFindCapsuleSupportFeaturePoint(
                capsule,
                -normal,
                cuboid.Center,
                out Vector3d collisionPointCapsule))
        {
            return false;
        }

        Vector3d collisionPointCuboid = FindCuboidSupportFeaturePoint(
            cuboid,
            normal,
            collisionPointCapsule);
        pair.Manifold.SetContact(
            collisionPointCuboid,
            collisionPointCapsule,
            axisPenetration.Depth,
            normal
        );

        return true;
    }

    internal static bool TryFindCapsuleSupportFeaturePoint(
        LSCapsuleCollider capsule,
        Vector3d direction,
        Vector3d target,
        out Vector3d supportPoint)
    {
        if (!Vector3d.TrySubtract(
                capsule.LineSegmentEnd,
                capsule.LineSegmentStart,
                out Vector3d segment))
        {
            supportPoint = default;
            return false;
        }

        Fixed64 segmentProjection = Vector3d.Dot(segment, direction);
        Vector3d segmentPoint;
        if (segmentProjection > Fixed64.Epsilon)
            segmentPoint = capsule.LineSegmentEnd;
        else if (segmentProjection < -Fixed64.Epsilon)
            segmentPoint = capsule.LineSegmentStart;
        else
            segmentPoint = Vector3d.ClosestPointOnLineSegment(
                target,
                capsule.LineSegmentStart,
                capsule.LineSegmentEnd);

        return Vector3d.TryAdd(
            segmentPoint,
            direction * capsule.ScaledRadius,
            out supportPoint);
    }

    private static void FindCuboidCapsulePenetration(
        LSCuboidCollider cuboid,
        LSCapsuleCollider capsule,
        out AxisPenetration penetration)
    {
        penetration = default;

        SwiftList<Vector3d> axes = cuboid.Context.CollisionScratch.CuboidCapsuleAxes;
        axes.FastClear();
        AxisProjectionHelper.GetCuboidAndCapsuleAxisVectors(cuboid, capsule, ref axes);

        FixedRange cuboidProjection = FixedRange.MinRange, capProjection;
        for (int i = 0; i < axes.Count; i++)
        {
            Vector3d axis = axes[i];
            AxisProjectionHelper.ProjectPolygonOntoAxis(axis, cuboid.Vertices, ref cuboidProjection);
            capProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(axis, capsule.LineSegmentStart, capsule.LineSegmentEnd, capsule.ScaledRadius);
            KeepDirectionalPenetration(axis, cuboidProjection, capProjection, ref penetration);
        }
    }

    private static void FindClosestPointsOnCuboidAndSegment(
        LSCuboidCollider cuboid,
        Vector3d segmentStart,
        Vector3d segmentEnd,
        out Vector3d pointOnCuboid,
        out Vector3d pointOnSegment)
    {
        FixedQuaternion inverseRotation = cuboid.Rotation.Inverse();
        Vector3d localStart = ToCuboidLocal(segmentStart, cuboid.Center, inverseRotation);
        Vector3d localEnd = ToCuboidLocal(segmentEnd, cuboid.Center, inverseRotation);
        Vector3d direction = localEnd - localStart;
        Vector3d halfExtents = cuboid.ScaledSize * Fixed64.Half;

        Span<Fixed64> breakpoints = stackalloc Fixed64[8];
        int breakpointCount = 2;
        breakpoints[0] = Fixed64.Zero;
        breakpoints[1] = Fixed64.One;
        AddSegmentBoxBreakpoints(localStart.X, direction.X, halfExtents.X, breakpoints, ref breakpointCount);
        AddSegmentBoxBreakpoints(localStart.Y, direction.Y, halfExtents.Y, breakpoints, ref breakpointCount);
        AddSegmentBoxBreakpoints(localStart.Z, direction.Z, halfExtents.Z, breakpoints, ref breakpointCount);
        SortBreakpoints(breakpoints, breakpointCount);

        Fixed64 directionLengthSquared = direction.MagnitudeSquared;
        Fixed64 representativeParameter = directionLengthSquared == Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Clamp(-Vector3d.Dot(localStart, direction) / directionLengthSquared, Fixed64.Zero, Fixed64.One);
        Fixed64 bestDistanceSquared = Fixed64.MaxValue;
        Fixed64 bestParameter = Fixed64.Zero;
        Vector3d bestSegmentPoint = localStart;
        Vector3d bestBoxPoint = ClampToCuboid(localStart, halfExtents);
        for (int i = 0; i < breakpointCount - 1; i++)
        {
            Fixed64 left = breakpoints[i];
            Fixed64 right = breakpoints[i + 1];
            Fixed64 sample = (left + right) * Fixed64.Half;
            Fixed64 numerator = Fixed64.Zero;
            Fixed64 denominator = Fixed64.Zero;
            AccumulateSegmentBoxDerivative(localStart.X, direction.X, halfExtents.X, sample, ref numerator, ref denominator);
            AccumulateSegmentBoxDerivative(localStart.Y, direction.Y, halfExtents.Y, sample, ref numerator, ref denominator);
            AccumulateSegmentBoxDerivative(localStart.Z, direction.Z, halfExtents.Z, sample, ref numerator, ref denominator);
            Fixed64 candidate = denominator == Fixed64.Zero
                ? FixedMath.Clamp(representativeParameter, left, right)
                : FixedMath.Clamp(-numerator / denominator, left, right);
            KeepClosestSegmentBoxCandidate(
                localStart,
                direction,
                halfExtents,
                candidate,
                representativeParameter,
                ref bestDistanceSquared,
                ref bestParameter,
                ref bestSegmentPoint,
                ref bestBoxPoint);
        }

        pointOnSegment = ToCuboidWorld(bestSegmentPoint, cuboid.Center, cuboid.Rotation);
        pointOnCuboid = ToCuboidWorld(bestBoxPoint, cuboid.Center, cuboid.Rotation);
    }

    private static void AddSegmentBoxBreakpoints(
        Fixed64 start,
        Fixed64 direction,
        Fixed64 halfExtent,
        Span<Fixed64> breakpoints,
        ref int count)
    {
        if (direction == Fixed64.Zero)
            return;

        AddSegmentBoxBreakpoint((-halfExtent - start) / direction, breakpoints, ref count);
        AddSegmentBoxBreakpoint((halfExtent - start) / direction, breakpoints, ref count);
    }

    private static void AddSegmentBoxBreakpoint(Fixed64 value, Span<Fixed64> breakpoints, ref int count)
    {
        if (value > Fixed64.Zero && value < Fixed64.One)
            breakpoints[count++] = value;
    }

    private static void SortBreakpoints(Span<Fixed64> breakpoints, int count)
    {
        for (int i = 1; i < count; i++)
        {
            Fixed64 value = breakpoints[i];
            int insertionIndex = i;
            // Zero is the retained first breakpoint, while every inserted value is positive.
            while (breakpoints[insertionIndex - 1] > value)
            {
                breakpoints[insertionIndex] = breakpoints[insertionIndex - 1];
                insertionIndex--;
            }

            breakpoints[insertionIndex] = value;
        }
    }

    private static void AccumulateSegmentBoxDerivative(
        Fixed64 start,
        Fixed64 direction,
        Fixed64 halfExtent,
        Fixed64 sample,
        ref Fixed64 numerator,
        ref Fixed64 denominator)
    {
        Fixed64 samplePosition = start + direction * sample;
        Fixed64 boundary;
        if (samplePosition < -halfExtent)
            boundary = -halfExtent;
        else if (samplePosition > halfExtent)
            boundary = halfExtent;
        else
            return;

        numerator += direction * (start - boundary);
        denominator += direction * direction;
    }

    private static void KeepClosestSegmentBoxCandidate(
        Vector3d start,
        Vector3d direction,
        Vector3d halfExtents,
        Fixed64 parameter,
        Fixed64 representativeParameter,
        ref Fixed64 bestDistanceSquared,
        ref Fixed64 bestParameter,
        ref Vector3d bestSegmentPoint,
        ref Vector3d bestBoxPoint)
    {
        Vector3d segmentPoint = start + direction * parameter;
        Vector3d boxPoint = ClampToCuboid(segmentPoint, halfExtents);
        Fixed64 distanceSquared = Vector3d.DistanceSquared(segmentPoint, boxPoint);
        if (bestDistanceSquared != Fixed64.MaxValue)
        {
            Fixed64 distanceDelta = distanceSquared - bestDistanceSquared;
            if (distanceDelta > ClosestSegmentBoxSquaredDistanceTolerance
                || (distanceDelta.Abs() <= ClosestSegmentBoxSquaredDistanceTolerance
                    && (parameter - representativeParameter).Abs()
                        >= (bestParameter - representativeParameter).Abs()))
            {
                return;
            }
        }

        bestDistanceSquared = distanceSquared;
        bestParameter = parameter;
        bestSegmentPoint = segmentPoint;
        bestBoxPoint = boxPoint;
    }

    private static Vector3d ClampToCuboid(Vector3d point, Vector3d halfExtents) =>
        new(
            FixedMath.Clamp(point.X, -halfExtents.X, halfExtents.X),
            FixedMath.Clamp(point.Y, -halfExtents.Y, halfExtents.Y),
            FixedMath.Clamp(point.Z, -halfExtents.Z, halfExtents.Z));

    private static Vector3d FindCuboidSupportFeaturePoint(
        LSCuboidCollider cuboid,
        Vector3d direction,
        Vector3d target)
    {
        Vector3d[] vertices = cuboid.Vertices;
        Vector3d supportVertex = ConvexColliderSupport.Support(cuboid, direction);
        Fixed64 supportProjection = Vector3d.Dot(supportVertex, direction);
        Vector3d closest = supportVertex;
        Fixed64 closestDistanceSquared = Vector3d.DistanceSquared(target, closest);

        for (int i = 0; i < LSCuboidCollider.EdgeDefinitions.Length; i++)
        {
            int[] edge = LSCuboidCollider.EdgeDefinitions[i];
            Vector3d start = vertices[edge[0]];
            Vector3d end = vertices[edge[1]];
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

        for (int i = 0; i < LSCuboidCollider.FaceDefinitions.Length; i++)
        {
            int[] face = LSCuboidCollider.FaceDefinitions[i];
            Vector3d first = vertices[face[0]];
            Vector3d second = vertices[face[1]];
            Vector3d third = vertices[face[2]];
            Vector3d fourth = vertices[face[3]];
            if (!(IsOnSupportPlane(first, direction, supportProjection)
                & IsOnSupportPlane(second, direction, supportProjection)
                & IsOnSupportPlane(third, direction, supportProjection)
                & IsOnSupportPlane(fourth, direction, supportProjection)))
            {
                continue;
            }

            KeepClosestFeaturePoint(
                MeshUtils.ClosestPointOnTriangle(first, second, third, direction, target),
                target,
                ref closest,
                ref closestDistanceSquared);
            KeepClosestFeaturePoint(
                MeshUtils.ClosestPointOnTriangle(first, third, fourth, direction, target),
                target,
                ref closest,
                ref closestDistanceSquared);
        }

        return closest;
    }

    private static Vector3d ToCuboidLocal(
        Vector3d point,
        Vector3d center,
        FixedQuaternion inverseRotation) =>
        inverseRotation * (point - center);

    private static Vector3d ToCuboidWorld(
        Vector3d point,
        Vector3d center,
        FixedQuaternion rotation) =>
        center + rotation * point;

    /// <summary>
    /// Checks for collisions between two poly-poly colliders.
    /// </summary>
    /// <returns>true if a collision is detected, false otherwise.</returns>
    private static bool DoCuboidsCheck(CollisionWorkItem pair)
    {
        var cuboidA = (LSCuboidCollider)pair.ColliderA;
        var cuboidB = (LSCuboidCollider)pair.ColliderB;

        if (cuboidA.Shape == ColliderType.AABox && cuboidB.Shape == ColliderType.AABox)
            return TryBuildAxisAlignedCuboidManifold(pair, cuboidA, cuboidB);

        if (!TestCuboidsSeperatingAxes(cuboidA, cuboidB, out CollisionResult output))
            return false;

        pair.Manifold.SetContact(
            output.PointsOfContact.Point1,
            output.PointsOfContact.Point2,
            output.AxisPenetration.Depth,
            OrientNormal(output.AxisPenetration.Vector, cuboidB.Center - cuboidA.Center)
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
        out CollisionResult output)
    {
        output = default;

        (Vector3d PointA, Vector3d PointB) = FindInitialPointsOfContact(cuboidA, cuboidB);
        if (!TryFindCuboidCuboidPenetration(cuboidA, cuboidB, out AxisPenetration penetration))
            return false;

        output = new CollisionResult(
            (PointA, PointB),
            (penetration.Axis, penetration.Depth));

        return true;
    }

    private static bool TryFindCuboidCuboidPenetration(
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB,
        out AxisPenetration penetration)
    {
        penetration = default;
        Vector3d[] verticesA = cuboidA.Vertices;
        Vector3d[] verticesB = cuboidB.Vertices;
        Vector3d displacementAtoB = cuboidB.Center - cuboidA.Center;

        if (!CheckCuboidFaceAxes(verticesA, verticesB, cuboidA, displacementAtoB, ref penetration)
            || !CheckCuboidFaceAxes(verticesA, verticesB, cuboidB, displacementAtoB, ref penetration))
        {
            return false;
        }

        return CheckCuboidEdgeCrossAxes(verticesA, verticesB, cuboidA, cuboidB, displacementAtoB, ref penetration)
            && penetration.HasValue;
    }

    private static bool CheckCuboidFaceAxes(
        Vector3d[] verticesA,
        Vector3d[] verticesB,
        LSCuboidCollider axisSource,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        return CheckVertexProjectionAxis(verticesA, verticesB, axisSource.FaceNormals[0], displacementAtoB, ref penetration)
            && CheckVertexProjectionAxis(verticesA, verticesB, axisSource.FaceNormals[2], displacementAtoB, ref penetration)
            && CheckVertexProjectionAxis(verticesA, verticesB, axisSource.FaceNormals[4], displacementAtoB, ref penetration);
    }

    private static bool CheckCuboidEdgeCrossAxes(
        Vector3d[] verticesA,
        Vector3d[] verticesB,
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        return CheckCuboidEdgeCrossAxes(verticesA, verticesB, cuboidA.EdgeDirections[0], cuboidB, displacementAtoB, ref penetration)
            && CheckCuboidEdgeCrossAxes(verticesA, verticesB, cuboidA.EdgeDirections[2], cuboidB, displacementAtoB, ref penetration)
            && CheckCuboidEdgeCrossAxes(verticesA, verticesB, cuboidA.EdgeDirections[8], cuboidB, displacementAtoB, ref penetration);
    }

    private static bool CheckCuboidEdgeCrossAxes(
        Vector3d[] verticesA,
        Vector3d[] verticesB,
        Vector3d edgeA,
        LSCuboidCollider cuboidB,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        return CheckVertexProjectionAxis(verticesA, verticesB, Vector3d.Cross(edgeA, cuboidB.EdgeDirections[0]), displacementAtoB, ref penetration)
            && CheckVertexProjectionAxis(verticesA, verticesB, Vector3d.Cross(edgeA, cuboidB.EdgeDirections[2]), displacementAtoB, ref penetration)
            && CheckVertexProjectionAxis(verticesA, verticesB, Vector3d.Cross(edgeA, cuboidB.EdgeDirections[8]), displacementAtoB, ref penetration);
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
