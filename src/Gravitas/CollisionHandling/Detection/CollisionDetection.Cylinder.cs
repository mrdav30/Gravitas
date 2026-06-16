using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Cylinder

    private static bool DoCylinderSphereCheck(CollisionWorkItem pair)
    {
        if (!TryGetPairColliders(pair, out LSCylinderCollider cylinder, out LSSphereCollider sphere))
            return false;

        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(sphere.Center);
        Vector3d delta = sphere.Center - cylinderPoint;
        if (delta.MagnitudeSquared > sphere.ScaledRadiusSqr)
            return false;

        Fixed64 distance = delta.Magnitude;
        Vector3d normal = ResolveNormal(delta, sphere.Center - cylinder.Center);
        Vector3d spherePoint = sphere.Center - normal * sphere.ScaledRadius;
        SetContactInPairOrder(
            pair,
            cylinder,
            cylinderPoint,
            sphere,
            spherePoint,
            sphere.ScaledRadius - distance,
            normal);

        return true;
    }

    private static bool DoCylinderCapsuleCheck(CollisionWorkItem pair)
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
        SetContactInPairOrder(
            pair,
            cylinder,
            cylinderPoint,
            capsule,
            capsulePoint,
            penetration.Depth,
            penetration.Axis);

        return true;
    }

    private static bool DoCylindersCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is not LSCylinderCollider cylinderA || pair.ColliderB is not LSCylinderCollider cylinderB)
            return false;

        if (!TestCylinderCylinderSeparatingAxes(cylinderA, cylinderB, out AxisPenetration penetration))
            return false;

        Vector3d cylinderAPoint = cylinderA.ClosestPointOnSurface(cylinderB.Center);
        Vector3d cylinderBPoint = cylinderB.ClosestPointOnSurface(cylinderAPoint);
        pair.Manifold.SetContact(
            cylinderAPoint,
            cylinderBPoint,
            penetration.Depth,
            penetration.Axis);

        return true;
    }

    private static bool DoCuboidCylinderCheck(CollisionWorkItem pair)
    {
        if (!TryGetPairColliders(pair, out LSCuboidCollider cuboid, out LSCylinderCollider cylinder))
            return false;

        if (!TestCuboidCylinderSeparatingAxes(cuboid, cylinder, out AxisPenetration penetration))
            return false;

        Vector3d cuboidPoint = cuboid.ClosestPointOnSurface(cylinder.Center);
        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(cuboidPoint);
        SetContactInPairOrder(
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeMinimumProjectionOverlap(FixedRange projectionA, FixedRange projectionB)
    {
        Fixed64 pushALeft = projectionA.Max - projectionB.Min;
        Fixed64 pushARight = projectionB.Max - projectionA.Min;
        Fixed64 overlap = FixedMath.Min(pushALeft, pushARight);
        return overlap > Fixed64.Zero ? overlap : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryNormalizeAxis(Vector3d axis, out Vector3d normalizedAxis)
    {
        Fixed64 magnitudeSqr = axis.MagnitudeSquared;
        if (magnitudeSqr <= Fixed64.Epsilon)
        {
            normalizedAxis = Vector3d.Zero;
            return false;
        }

        normalizedAxis = axis / FixedMath.Sqrt(magnitudeSqr);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveNormal(Vector3d delta, Vector3d fallback)
    {
        if (delta.MagnitudeSquared > Fixed64.Epsilon)
            return delta.Normalized;

        if (fallback.MagnitudeSquared > Fixed64.Epsilon)
            return fallback.Normalized;

        return Vector3d.Right;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d OrientNormal(Vector3d normal, Vector3d desiredDirection)
    {
        Vector3d resolved = ResolveNormal(normal, desiredDirection);
        return Vector3d.Dot(resolved, desiredDirection) < Fixed64.Zero ? -resolved : resolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryGetPairColliders<TFirst, TSecond>(
        CollisionWorkItem pair,
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetContactInPairOrder(
        CollisionWorkItem pair,
        LSCollider first,
        Vector3d pointOnFirst,
        LSCollider second,
        Vector3d pointOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond)
    {
        if (ReferenceEquals(pair.ColliderA, first))
        {
            pair.Manifold.SetContact(pointOnFirst, pointOnSecond, depth, normalFirstToSecond);
            return;
        }

        if (ReferenceEquals(pair.ColliderA, second))
            pair.Manifold.SetContact(pointOnSecond, pointOnFirst, depth, -normalFirstToSecond);
    }

    #endregion

}
