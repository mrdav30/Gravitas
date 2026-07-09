//=======================================================================
// CollisionDetection.Cylinder.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Cylinder

    private static bool DoCylinderSphereCheck(CollisionWorkItem pair)
    {
        var cylinder = (LSCylinderCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(sphere.Center);
        Vector3d delta = sphere.Center - cylinderPoint;
        if (delta.MagnitudeSquared > sphere.ScaledRadiusSqr)
            return false;

        Fixed64 distance = delta.Magnitude;
        Vector3d normal = ResolveNormal(delta, sphere.Center - cylinder.Center);
        Vector3d spherePoint = sphere.Center - normal * sphere.ScaledRadius;
        pair.Manifold.SetContact(
            cylinderPoint,
            spherePoint,
            sphere.ScaledRadius - distance,
            normal);

        return true;
    }

    private static bool DoCylinderCapsuleCheck(CollisionWorkItem pair)
    {
        LSCylinderCollider cylinder;
        LSCapsuleCollider capsule;
        if (pair.ColliderA is LSCylinderCollider cylinderA)
        {
            cylinder = cylinderA;
            capsule = (LSCapsuleCollider)pair.ColliderB;
        }
        else
        {
            cylinder = (LSCylinderCollider)pair.ColliderB;
            capsule = (LSCapsuleCollider)pair.ColliderA;
        }

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
        var cylinderA = (LSCylinderCollider)pair.ColliderA;
        var cylinderB = (LSCylinderCollider)pair.ColliderB;

        if (!TestCylinderCylinderSeparatingAxes(cylinderA, cylinderB, out AxisPenetration penetration))
            return false;

        if (TryAddCylinderCylinderCapContacts(pair, cylinderA, cylinderB, penetration))
            return true;

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
        var cuboid = (LSCuboidCollider)pair.ColliderA;
        var cylinder = (LSCylinderCollider)pair.ColliderB;

        if (!TestCuboidCylinderSeparatingAxes(cuboid, cylinder, out AxisPenetration penetration))
            return false;

        if (TryAddCuboidCylinderCapContacts(pair, cuboid, cylinder, penetration))
            return true;

        Vector3d cuboidPoint = cuboid.ClosestPointOnSurface(cylinder.Center);
        Vector3d cylinderPoint = cylinder.ClosestPointOnSurface(cuboidPoint);
        pair.Manifold.SetContact(
            cuboidPoint,
            cylinderPoint,
            penetration.Depth,
            penetration.Axis);

        return true;
    }

    private static bool TryAddCylinderCylinderCapContacts(
        CollisionWorkItem pair,
        LSCylinderCollider cylinderA,
        LSCylinderCollider cylinderB,
        AxisPenetration penetration)
    {
        if (!CylinderContactGeometry.IsAxisAligned(penetration.Axis, cylinderA.LineDirection)
            || !CylinderContactGeometry.IsAxisAligned(penetration.Axis, cylinderB.LineDirection))
        {
            return false;
        }

        Vector3d capCenter = CylinderContactGeometry.GetCapCenter(cylinderA, penetration.Axis);
        CylinderContactGeometry.GetCapBasis(cylinderA, out Vector3d tangentA, out Vector3d tangentB);
        AddCylinderCylinderCapContact(pair, capCenter + tangentA * cylinderA.ScaledRadius, penetration);
        AddCylinderCylinderCapContact(pair, capCenter - tangentA * cylinderA.ScaledRadius, penetration);
        AddCylinderCylinderCapContact(pair, capCenter + tangentB * cylinderA.ScaledRadius, penetration);
        AddCylinderCylinderCapContact(pair, capCenter - tangentB * cylinderA.ScaledRadius, penetration);
        return pair.Manifold.HasContact;
    }

    private static bool TryAddCuboidCylinderCapContacts(
        CollisionWorkItem pair,
        LSCuboidCollider cuboid,
        LSCylinderCollider cylinder,
        AxisPenetration penetration)
    {
        if (!CylinderContactGeometry.IsAxisAligned(penetration.Axis, cylinder.LineDirection))
            return false;

        Vector3d capCenter = CylinderContactGeometry.GetCapCenter(cylinder, -penetration.Axis);
        CylinderContactGeometry.GetCapBasis(cylinder, out Vector3d tangentA, out Vector3d tangentB);
        AddCuboidCylinderCapContact(pair, cuboid, cylinder, capCenter + tangentA * cylinder.ScaledRadius, penetration);
        AddCuboidCylinderCapContact(pair, cuboid, cylinder, capCenter - tangentA * cylinder.ScaledRadius, penetration);
        AddCuboidCylinderCapContact(pair, cuboid, cylinder, capCenter + tangentB * cylinder.ScaledRadius, penetration);
        AddCuboidCylinderCapContact(pair, cuboid, cylinder, capCenter - tangentB * cylinder.ScaledRadius, penetration);
        return pair.Manifold.HasContact;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddCylinderCylinderCapContact(
        CollisionWorkItem pair,
        Vector3d pointOnA,
        AxisPenetration penetration)
    {
        pair.Manifold.AddContact(
            pointOnA,
            pointOnA - penetration.Axis * penetration.Depth,
            penetration.Depth,
            penetration.Axis);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddCuboidCylinderCapContact(
        CollisionWorkItem pair,
        LSCuboidCollider cuboid,
        LSCylinderCollider cylinder,
        Vector3d pointOnCylinder,
        AxisPenetration penetration)
    {
        Vector3d pointOnCuboid = pointOnCylinder + penetration.Axis * penetration.Depth;
        pair.Manifold.AddContact(
            pointOnCuboid,
            pointOnCylinder,
            penetration.Depth,
            penetration.Axis);
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

        pair.Manifold.SetContact(pointOnSecond, pointOnFirst, depth, -normalFirstToSecond);
    }

    #endregion

}
