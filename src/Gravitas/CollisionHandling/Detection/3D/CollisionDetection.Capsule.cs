//=======================================================================
// CollisionDetection.Capsule.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Capsule

    private static bool DoCapsuleSphereCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is not LSCapsuleCollider capsule || pair.ColliderB is not LSSphereCollider sphere)
            return false;

        Vector3d closestPointOnCapsule = capsule.ClosestPointOnSurface(sphere.Center);
        Vector3d penetrationVector = sphere.Center - closestPointOnCapsule;
        // Check if the distance from the sphere center to the closest point is less than the sum of the radii
        if (penetrationVector.MagnitudeSquared > sphere.ScaledRadiusSqr)
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        Vector3d penetrationNormal = ResolveNormal(penetrationVector, sphere.Center - capsule.Center);
        pair.Manifold.SetContact(
            closestPointOnCapsule,
            sphere.Center - penetrationNormal * sphere.ScaledRadius,
            penetrationVector.Magnitude - sphere.ScaledRadius,
            penetrationNormal
        );
        return true;
    }

    private static bool DoCapsulesCheck(CollisionWorkItem pair)
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
        if (centerDelta.MagnitudeSquared > radiusSum * radiusSum)
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        Fixed64 distance = centerDelta.Magnitude;
        Vector3d penetrationNormal = distance > Fixed64.Epsilon
            ? centerDelta / distance
            : Vector3d.Right;
        Vector3d collisionPointCapsule1 = closestPointsOnCapsules.Item1 + penetrationNormal * capsule1.ScaledRadius;
        Vector3d collisionPointCapsule2 = closestPointsOnCapsules.Item2 - penetrationNormal * capsule2.ScaledRadius;
        pair.Manifold.SetContact(
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
        bool firstDegenerate = (firstEnd - firstStart).MagnitudeSquared <= Fixed64.Epsilon;
        bool secondDegenerate = (secondEnd - secondStart).MagnitudeSquared <= Fixed64.Epsilon;

        if (firstDegenerate && secondDegenerate)
            return (firstStart, secondStart);

        if (firstDegenerate)
            return (firstStart, Vector3d.ClosestPointOnLineSegment(firstStart, secondStart, secondEnd));

        if (secondDegenerate)
            return (Vector3d.ClosestPointOnLineSegment(secondStart, firstStart, firstEnd), secondStart);

        return Vector3d.ClosestPointsOnTwoLines(firstStart, firstEnd, secondStart, secondEnd);
    }

    #endregion

}
