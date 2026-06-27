//=======================================================================
// GravitasQueryMixedService.SphereAgainst2DReducers.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed swept-sphere reducers against embedded 2D collider targets.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private static bool TrySweepSphereAgainst2D(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        if (collider is LSCompoundCollider2D compound)
            return TrySweepSphereAgainstCompound2D(start, direction, length, radius, compound, out hit);

        if (collider is LSCircleCollider2D circle)
            return TrySweepSphereAgainstCircleSlab(start, direction, length, radius, circle, out hit);

        if (collider is LSCapsuleCollider2D capsule)
            return TrySweepSphereAgainstCapsuleSlab(start, direction, length, radius, capsule, out hit);

        if (collider is LSAABBoxCollider2D || collider is LSPolygonCollider2D)
            return TrySweepSphereAgainstConvexSlab(start, direction, length, radius, collider, out hit);

        return TrySweepSphereAgainstPrismBounds(start, direction, length, radius, collider, out hit);
    }

    private static bool TrySweepSphereAgainstCompound2D(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCompoundCollider2D compound,
        out PhysicsMixedHit hit)
    {
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        PhysicsMixedHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TrySweepSphereAgainst2D(start, direction, length, radius, part, out PhysicsMixedHit candidate))
                continue;

            if (found && candidate.Distance >= bestDistance)
                continue;

            best = candidate;
            bestDistance = candidate.Distance;
            found = true;
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new PhysicsMixedHit(
            null,
            compound,
            best.Point3D,
            best.Point2D,
            best.Normal3DTo2D,
            best.ReducerKind,
            best.Distance,
            best.Direction3D);
        return true;
    }

    private static bool TrySweepSphereAgainstCapsuleSlab(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCapsuleCollider2D capsule,
        out PhysicsMixedHit hit)
    {
        if (DistanceSquaredToCapsuleSlab(start, capsule) <= radius * radius)
        {
            hit = BuildSphereAgainst2DHit(
                capsule,
                start,
                radius,
                PhysicsQueryReducerKind.Exact,
                Fixed64.Zero,
                direction);
            return true;
        }

        Fixed64 slabMinY = capsule.MixedSlabCenterY - capsule.MixedHalfThickness;
        Fixed64 slabMaxY = capsule.MixedSlabCenterY + capsule.MixedHalfThickness;
        Fixed64 combinedRadius = radius + capsule.ScaledRadius;
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;

        TrySweepCapsuleSlabCapFace(
            start,
            direction,
            length,
            capsule,
            slabMaxY + radius,
            requireStartBeyondPlane: true,
            ref found,
            ref bestDistance);
        TrySweepCapsuleSlabCapFace(
            start,
            direction,
            length,
            capsule,
            slabMinY - radius,
            requireStartBeyondPlane: false,
            ref found,
            ref bestDistance);
        TrySweepCapsuleSlabSide(
            start,
            direction,
            length,
            capsule,
            combinedRadius,
            slabMinY,
            slabMaxY,
            ref found,
            ref bestDistance);
        TrySweepCapsuleSlabBoundaryEdges(
            start,
            direction,
            length,
            capsule,
            combinedRadius,
            slabMinY,
            slabMaxY,
            ref found,
            ref bestDistance);

        if (!found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = start + direction * bestDistance;
        hit = BuildSphereAgainst2DHit(
            capsule,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            bestDistance,
            direction);
        return true;
    }

    private static bool TrySweepSphereAgainstConvexSlab(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        if (collider.VertexCount < 3)
        {
            hit = default;
            return false;
        }

        Fixed64 radiusSqr = radius * radius;
        if (DistanceSquaredToConvexSlab(start, collider) <= radiusSqr)
        {
            hit = BuildSphereAgainst2DHit(
                collider,
                start,
                radius,
                PhysicsQueryReducerKind.Exact,
                Fixed64.Zero,
                direction);
            return true;
        }

        Fixed64 slabMinY = collider.MixedSlabCenterY - collider.MixedHalfThickness;
        Fixed64 slabMaxY = collider.MixedSlabCenterY + collider.MixedHalfThickness;
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;

        TrySweepConvexSlabCapFace(
            start,
            direction,
            length,
            collider,
            slabMaxY + radius,
            requireStartBeyondPlane: true,
            ref found,
            ref bestDistance);
        TrySweepConvexSlabCapFace(
            start,
            direction,
            length,
            collider,
            slabMinY - radius,
            requireStartBeyondPlane: false,
            ref found,
            ref bestDistance);
        TrySweepConvexSlabSideFaces(
            start,
            direction,
            length,
            radius,
            collider,
            slabMinY,
            slabMaxY,
            ref found,
            ref bestDistance);
        TrySweepConvexSlabEdges(
            start,
            direction,
            length,
            radius,
            collider,
            slabMinY,
            slabMaxY,
            ref found,
            ref bestDistance);

        if (!found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = start + direction * bestDistance;
        hit = BuildSphereAgainst2DHit(
            collider,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            bestDistance,
            direction);
        return true;
    }

    private static void TrySweepConvexSlabCapFace(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        LSCollider2D collider,
        Fixed64 planeY,
        bool requireStartBeyondPlane,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        bool canReach = requireStartBeyondPlane
            ? start.Y > planeY && direction.Y < -Fixed64.Epsilon
            : start.Y < planeY && direction.Y > Fixed64.Epsilon;
        if (!canReach)
            return;

        Fixed64 distance = (planeY - start.Y) / direction.Y;
        if (distance < Fixed64.Zero || distance > length)
            return;

        Vector3d point = start + direction * distance;
        if (!collider.ContainsPoint(new Vector2d(point.X, point.Z)))
            return;

        TryKeepEarlierSweep(true, distance, ref found, ref bestDistance);
    }

    private static void TrySweepConvexSlabSideFaces(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        Vector2d planarStart = new(start.X, start.Z);
        Vector2d planarDirection = new(direction.X, direction.Z);
        Vector2d polygonCenter = collider.Center;
        int vertexCount = collider.VertexCount;

        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d first = collider.GetVertexUnchecked(i);
            Vector2d second = collider.GetVertexUnchecked((i + 1) % vertexCount);
            Vector2d edge = second - first;
            Fixed64 edgeLengthSqr = edge.MagnitudeSquared;
            if (edgeLengthSqr <= Fixed64.Epsilon)
                continue;

            Fixed64 edgeLength = FixedMath.Sqrt(edgeLengthSqr);
            Vector2d edgeDirection = edge / edgeLength;
            Vector2d outward = new(edge.Y, -edge.X);
            if (Vector2d.Dot(polygonCenter - first, outward) > Fixed64.Zero)
                outward = -outward;
            Fixed64 outwardLengthSqr = outward.MagnitudeSquared;
            if (outwardLengthSqr <= Fixed64.Epsilon)
                continue;

            outward /= FixedMath.Sqrt(outwardLengthSqr);
            Fixed64 signedStart = Vector2d.Dot(planarStart - first, outward);
            Fixed64 signedDirection = Vector2d.Dot(planarDirection, outward);
            if (signedStart <= radius || signedDirection >= -Fixed64.Epsilon)
                continue;

            Fixed64 distance = (radius - signedStart) / signedDirection;
            if (distance < Fixed64.Zero || distance > length)
                continue;

            Vector3d sweepCenter = start + direction * distance;
            if (sweepCenter.Y < slabMinY || sweepCenter.Y > slabMaxY)
                continue;

            Vector2d planarPoint = new(sweepCenter.X, sweepCenter.Z);
            Fixed64 projection = Vector2d.Dot(planarPoint - first, edgeDirection);
            if (projection < Fixed64.Zero || projection > edgeLength)
                continue;

            TryKeepEarlierSweep(true, distance, ref found, ref bestDistance);
        }
    }

    private static void TrySweepConvexSlabEdges(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        int vertexCount = collider.VertexCount;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d first = collider.GetVertexUnchecked(i);
            Vector2d second = collider.GetVertexUnchecked((i + 1) % vertexCount);
            Vector3d bottomFirst = new(first.X, slabMinY, first.Y);
            Vector3d topFirst = new(first.X, slabMaxY, first.Y);
            Vector3d bottomSecond = new(second.X, slabMinY, second.Y);
            Vector3d topSecond = new(second.X, slabMaxY, second.Y);

            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule3D(start, direction, length, bottomFirst, topFirst, radius, out Fixed64 verticalDistance),
                verticalDistance,
                ref found,
                ref bestDistance);
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule3D(start, direction, length, bottomFirst, bottomSecond, radius, out Fixed64 bottomDistance),
                bottomDistance,
                ref found,
                ref bestDistance);
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule3D(start, direction, length, topFirst, topSecond, radius, out Fixed64 topDistance),
                topDistance,
                ref found,
                ref bestDistance);
        }
    }

    private static void TrySweepCapsuleSlabCapFace(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Fixed64 planeY,
        bool requireStartBeyondPlane,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        bool canReach = requireStartBeyondPlane
            ? start.Y > planeY && direction.Y < -Fixed64.Epsilon
            : start.Y < planeY && direction.Y > Fixed64.Epsilon;
        if (!canReach)
            return;

        Fixed64 distance = (planeY - start.Y) / direction.Y;
        if (distance < Fixed64.Zero || distance > length)
            return;

        Vector3d point = start + direction * distance;
        if (!capsule.ContainsPoint(new Vector2d(point.X, point.Z)))
            return;

        TryKeepEarlierSweep(true, distance, ref found, ref bestDistance);
    }

    private static void TrySweepCapsuleSlabSide(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Fixed64 combinedRadius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        Vector2d planarDirection = new(direction.X, direction.Z);
        Fixed64 planarSpeedSquared = planarDirection.MagnitudeSquared;
        if (planarSpeedSquared <= Fixed64.Epsilon)
            return;

        Fixed64 planarSpeed = FixedMath.Sqrt(planarSpeedSquared);
        Vector2d planarUnit = planarDirection / planarSpeed;
        Fixed64 planarLength = length * planarSpeed;
        if (!TrySweepPointAgainstSegmentCapsule(
                new Vector2d(start.X, start.Z),
                planarUnit,
                planarLength,
                capsule.SegmentStart,
                capsule.SegmentEnd,
                combinedRadius,
                out Fixed64 planarDistance))
        {
            return;
        }

        Fixed64 distance = planarDistance / planarSpeed;
        if (distance < Fixed64.Zero || distance > length)
            return;

        Fixed64 y = start.Y + direction.Y * distance;
        if (y < slabMinY || y > slabMaxY)
            return;

        TryKeepEarlierSweep(true, distance, ref found, ref bestDistance);
    }

    private static void TrySweepCapsuleSlabBoundaryEdges(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Fixed64 combinedRadius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        Vector3d bottomStart = new(capsule.SegmentStart.X, slabMinY, capsule.SegmentStart.Y);
        Vector3d bottomEnd = new(capsule.SegmentEnd.X, slabMinY, capsule.SegmentEnd.Y);
        Vector3d topStart = new(capsule.SegmentStart.X, slabMaxY, capsule.SegmentStart.Y);
        Vector3d topEnd = new(capsule.SegmentEnd.X, slabMaxY, capsule.SegmentEnd.Y);

        TryKeepCapsuleSlabBoundaryEdgeSweep(
            start,
            direction,
            length,
            capsule,
            bottomStart,
            bottomEnd,
            combinedRadius,
            ref found,
            ref bestDistance);
        TryKeepCapsuleSlabBoundaryEdgeSweep(
            start,
            direction,
            length,
            capsule,
            topStart,
            topEnd,
            combinedRadius,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepPointAgainstSegmentCapsule3D(start, direction, length, bottomStart, topStart, combinedRadius, out Fixed64 firstVerticalDistance),
            firstVerticalDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepPointAgainstSegmentCapsule3D(start, direction, length, bottomEnd, topEnd, combinedRadius, out Fixed64 secondVerticalDistance),
            secondVerticalDistance,
            ref found,
            ref bestDistance);
    }

    private static void TryKeepCapsuleSlabBoundaryEdgeSweep(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Vector3d edgeStart,
        Vector3d edgeEnd,
        Fixed64 radius,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        if (!TrySweepPointAgainstSegmentCapsule3D(start, direction, length, edgeStart, edgeEnd, radius, out Fixed64 distance))
            return;

        Vector3d sweepCenter = start + direction * distance;
        if (capsule.ContainsPoint(new Vector2d(sweepCenter.X, sweepCenter.Z)))
            return;

        TryKeepEarlierSweep(true, distance, ref found, ref bestDistance);
    }

    private static Fixed64 DistanceSquaredToCapsuleSlab(Vector3d point, LSCapsuleCollider2D capsule)
    {
        Vector2d planarPoint = new(point.X, point.Z);
        Fixed64 planarDistanceSqr = Fixed64.Zero;
        if (!capsule.ContainsPoint(planarPoint))
        {
            Vector2d closestPlanar = capsule.GetClosestPoint(planarPoint);
            planarDistanceSqr = (planarPoint - closestPlanar).MagnitudeSquared;
        }

        Fixed64 slabMinY = capsule.MixedSlabCenterY - capsule.MixedHalfThickness;
        Fixed64 slabMaxY = capsule.MixedSlabCenterY + capsule.MixedHalfThickness;
        Fixed64 verticalDistance = GetIntervalDistance(point.Y, point.Y, slabMinY, slabMaxY);
        return planarDistanceSqr + verticalDistance * verticalDistance;
    }

    private static Fixed64 DistanceSquaredToConvexSlab(Vector3d point, LSCollider2D collider)
    {
        Vector2d planarPoint = new(point.X, point.Z);
        Vector2d closestPlanar = collider.GetClosestPoint(planarPoint);
        Fixed64 planarDistanceSqr = (planarPoint - closestPlanar).MagnitudeSquared;
        Fixed64 slabMinY = collider.MixedSlabCenterY - collider.MixedHalfThickness;
        Fixed64 slabMaxY = collider.MixedSlabCenterY + collider.MixedHalfThickness;
        Fixed64 verticalDistance = GetIntervalDistance(point.Y, point.Y, slabMinY, slabMaxY);
        return planarDistanceSqr + verticalDistance * verticalDistance;
    }

    private static bool TrySweepPointAgainstSegmentCapsule3D(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Vector3d segmentStart,
        Vector3d segmentEnd,
        Fixed64 radius,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        Fixed64 radiusSqr = radius * radius;
        if (DistanceSquaredToSegment3D(start, segmentStart, segmentEnd) <= radiusSqr)
            return true;

        Vector3d segment = segmentEnd - segmentStart;
        Fixed64 segmentLengthSqr = segment.MagnitudeSquared;
        if (segmentLengthSqr <= Fixed64.Epsilon)
            return TrySweepPointInSpace(start, direction, length, segmentStart, radius, out distance);

        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        TryKeepEarlierSweep(
            TrySweepPointInSpace(start, direction, length, segmentStart, radius, out Fixed64 startDistance),
            startDistance,
            ref found,
            ref best);
        TryKeepEarlierSweep(
            TrySweepPointInSpace(start, direction, length, segmentEnd, radius, out Fixed64 endDistance),
            endDistance,
            ref found,
            ref best);

        Fixed64 segmentLength = FixedMath.Sqrt(segmentLengthSqr);
        Vector3d axis = segment / segmentLength;
        Vector3d toStart = start - segmentStart;
        Fixed64 startProjection = Vector3d.Dot(toStart, axis);
        Fixed64 directionProjection = Vector3d.Dot(direction, axis);
        Vector3d startPerpendicular = toStart - axis * startProjection;
        Vector3d directionPerpendicular = direction - axis * directionProjection;
        Fixed64 a = Vector3d.Dot(directionPerpendicular, directionPerpendicular);
        if (a > Fixed64.Epsilon)
        {
            Fixed64 b = 2 * Vector3d.Dot(startPerpendicular, directionPerpendicular);
            Fixed64 c = Vector3d.Dot(startPerpendicular, startPerpendicular) - radiusSqr;
            if (TrySolveQuadraticSweep(a, b, c, out Fixed64 first, out Fixed64 second))
            {
                TryKeepEarlierSweep(
                    IsSegmentCapsuleCylinderHit(startProjection, directionProjection, segmentLength, length, first),
                    first,
                    ref found,
                    ref best);
                TryKeepEarlierSweep(
                    IsSegmentCapsuleCylinderHit(startProjection, directionProjection, segmentLength, length, second),
                    second,
                    ref found,
                    ref best);
            }
        }

        distance = best;
        return found;
    }

    private static bool TrySweepPointInSpace(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Vector3d point,
        Fixed64 radius,
        out Fixed64 distance)
    {
        Fixed64 radiusSqr = radius * radius;
        Vector3d startToPoint = start - point;
        if (startToPoint.MagnitudeSquared <= radiusSqr)
        {
            distance = Fixed64.Zero;
            return true;
        }

        Fixed64 a = direction.MagnitudeSquared;
        if (a <= Fixed64.Epsilon)
        {
            distance = default;
            return false;
        }

        Fixed64 b = 2 * Vector3d.Dot(startToPoint, direction);
        Fixed64 c = startToPoint.MagnitudeSquared - radiusSqr;
        if (c > Fixed64.Zero && b > Fixed64.Zero)
        {
            distance = default;
            return false;
        }

        if (!TrySolveQuadraticSweep(a, b, c, out Fixed64 first, out Fixed64 second))
        {
            distance = default;
            return false;
        }

        if (first >= Fixed64.Zero && first <= length)
        {
            distance = first;
            return true;
        }

        if (second >= Fixed64.Zero && second <= length)
        {
            distance = second;
            return true;
        }

        distance = default;
        return false;
    }

    private static bool TrySolveQuadraticSweep(
        Fixed64 a,
        Fixed64 b,
        Fixed64 c,
        out Fixed64 first,
        out Fixed64 second)
    {
        first = default;
        second = default;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        first = (-b - root) / denominator;
        second = (-b + root) / denominator;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsSegmentCapsuleCylinderHit(
        Fixed64 startProjection,
        Fixed64 directionProjection,
        Fixed64 segmentLength,
        Fixed64 sweepLength,
        Fixed64 distance)
    {
        if (distance < Fixed64.Zero || distance > sweepLength)
            return false;

        Fixed64 projection = startProjection + directionProjection * distance;
        return projection >= Fixed64.Zero && projection <= segmentLength;
    }

}
