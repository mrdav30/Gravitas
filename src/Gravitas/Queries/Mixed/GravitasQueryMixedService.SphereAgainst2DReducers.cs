//=======================================================================
// GravitasQueryMixedService.SphereAgainst2DReducers.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed swept-sphere reducers against embedded 2D collider targets.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private static bool TrySweepSphereAgainst2D(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        if (collider is LSCompoundCollider2D compound)
            return TrySweepSphereAgainstCompound2D(start, end, direction, length, radius, compound, out hit);

        if (collider is LSCircleCollider2D circle)
            return TrySweepSphereAgainstCircleSlab(start, end, direction, length, radius, circle, out hit);

        if (collider is LSCapsuleCollider2D capsule)
            return TrySweepSphereAgainstCapsuleSlab(start, end, direction, length, radius, capsule, out hit);

        if (collider is LSAABBoxCollider2D || collider is LSPolygonCollider2D)
            return TrySweepSphereAgainstConvexSlab(start, end, direction, length, radius, collider, out hit);

        return TrySweepSphereAgainstPrismBounds(start, end, direction, length, radius, collider, out hit);
    }

    private static bool TrySweepSphereAgainstCompound2D(
        Vector3d start,
        Vector3d end,
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
            if (!TrySweepSphereAgainst2D(start, end, direction, length, radius, part, out PhysicsMixedHit candidate))
                continue;

            if (!PhysicsHitSelectionPolicy.ShouldReplaceDistance(candidate.Distance, found, bestDistance))
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

    internal static bool TrySweepSphereAgainstCapsuleSlab(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCapsuleCollider2D capsule,
        out PhysicsMixedHit hit)
    {
        if (IsSphereOverlappingCapsuleSlabCapCore(start, radius, capsule))
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

        Fixed64 slabMinY = capsule.MixedBounds3D.Min.Y;
        Fixed64 slabMaxY = capsule.MixedBounds3D.Max.Y;
        SweepCandidate best = default;

        if (Fixed64.TryAdd(slabMaxY, radius, out Fixed64 expandedMaxY))
        {
            TrySweepCapsuleSlabCapFace(
                start,
                end,
                length,
                capsule,
                expandedMaxY,
                requireStartBeyondPlane: true,
                -Vector3d.Up,
                ref best);
        }

        if (Fixed64.TrySubtract(slabMinY, radius, out Fixed64 expandedMinY))
        {
            TrySweepCapsuleSlabCapFace(
                start,
                end,
                length,
                capsule,
                expandedMinY,
                requireStartBeyondPlane: false,
                Vector3d.Up,
                ref best);
        }

        TrySweepCapsuleSlabSide(
            start,
            end,
            length,
            capsule,
            capsule.ScaledRadius,
            radius,
            slabMinY,
            slabMaxY,
            ref best);
        TrySweepCapsuleSlabBoundaryEdges(
            start,
            end,
            length,
            capsule,
            capsule.ScaledRadius,
            radius,
            slabMinY,
            slabMaxY,
            ref best);

        if (!best.Found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = new FixedSegment(start, end).GetPointAtDistance(best.Distance, length);
        hit = BuildSphereAgainst2DHit(
            capsule,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            best.Distance,
            direction,
            best.FeatureNormal);
        return true;
    }

    private static bool IsSphereOverlappingCapsuleSlabCapCore(
        Vector3d center,
        Fixed64 radius,
        LSCapsuleCollider2D capsule)
    {
        Fixed64 slabMinY = capsule.MixedBounds3D.Min.Y;
        Fixed64 slabMaxY = capsule.MixedBounds3D.Max.Y;
        Fixed64 verticalExcess;
        if (center.Y > slabMaxY)
        {
            if (!Fixed64.TrySubtract(center.Y, slabMaxY, out verticalExcess))
                return false;
        }
        else if (center.Y < slabMinY)
        {
            if (!Fixed64.TrySubtract(slabMinY, center.Y, out verticalExcess))
                return false;
        }
        else
            return false;

        return verticalExcess <= radius
            && capsule.ContainsPoint(new Vector2d(center.X, center.Z));
    }

    internal static bool TrySweepSphereAgainstConvexSlab(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
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

        Fixed64 slabMinY = collider.MixedBounds3D.Min.Y;
        Fixed64 slabMaxY = collider.MixedBounds3D.Max.Y;
        SweepCandidate best = default;

        if (Fixed64.TryAdd(slabMaxY, radius, out Fixed64 expandedMaxY))
        {
            TrySweepConvexSlabCapFace(
                start,
                end,
                length,
                collider,
                expandedMaxY,
                requireStartBeyondPlane: true,
                -Vector3d.Up,
                ref best);
        }

        if (Fixed64.TrySubtract(slabMinY, radius, out Fixed64 expandedMinY))
        {
            TrySweepConvexSlabCapFace(
                start,
                end,
                length,
                collider,
                expandedMinY,
                requireStartBeyondPlane: false,
                Vector3d.Up,
                ref best);
        }

        TrySweepConvexSlabSideFaces(
            start,
            end,
            direction,
            length,
            radius,
            collider,
            slabMinY,
            slabMaxY,
            ref best);
        TrySweepConvexSlabEdges(
            start,
            end,
            length,
            radius,
            collider,
            slabMinY,
            slabMaxY,
            ref best);

        if (!best.Found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = new FixedSegment(start, end).GetPointAtDistance(best.Distance, length);
        hit = BuildSphereAgainst2DHit(
            collider,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            best.Distance,
            direction,
            best.FeatureNormal);
        return true;
    }

    private static void TrySweepConvexSlabCapFace(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        LSCollider2D collider,
        Fixed64 planeY,
        bool requireStartBeyondPlane,
        Vector3d featureNormal,
        ref SweepCandidate best)
    {
        bool canReach = requireStartBeyondPlane
            ? start.Y > planeY && end.Y <= planeY
            : start.Y < planeY && end.Y >= planeY;
        if (!canReach)
            return;

        Fixed64 distance = GetCapDistance(start.Y, end.Y, planeY, length);
        Vector3d point = new FixedSegment(start, end).GetPointAtDistance(distance, length);
        if (!collider.ContainsPoint(new Vector2d(point.X, point.Z)))
            return;

        TryKeepEarlierSweep(true, distance, featureNormal, ref best);
    }

    private static void TrySweepConvexSlabSideFaces(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref SweepCandidate best)
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
            Fixed64 edgeLength = edge.Magnitude;
            Vector2d edgeDirection = edge / edgeLength;
            Vector2d outward = new(edge.Y, -edge.X);
            if (Vector2d.Dot(polygonCenter - first, outward) > Fixed64.Zero)
                outward = -outward;
            outward /= outward.Magnitude;
            Fixed64 signedStart = Vector2d.Dot(planarStart - first, outward);
            Fixed64 signedDirection = Vector2d.Dot(planarDirection, outward);
            if (signedStart <= radius || signedDirection >= -Fixed64.Epsilon)
                continue;

            Fixed64 distance = (radius - signedStart) / signedDirection;
            if (distance > length)
                continue;

            Vector3d sweepCenter = new FixedSegment(start, end).GetPointAtDistance(distance, length);
            if (sweepCenter.Y < slabMinY || sweepCenter.Y > slabMaxY)
                continue;

            Vector2d planarPoint = new(sweepCenter.X, sweepCenter.Z);
            Fixed64 projection = Vector2d.Dot(planarPoint - first, edgeDirection);
            if (projection < Fixed64.Zero || projection > edgeLength)
                continue;

            TryKeepEarlierSweep(true, distance, default, ref best);
        }
    }

    private static void TrySweepConvexSlabEdges(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref SweepCandidate best)
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
                TrySweepPointAgainstSegmentCapsule3D(
                    start,
                    end,
                    length,
                    bottomFirst,
                    topFirst,
                    Fixed64.Zero,
                    radius,
                    out Fixed64 verticalDistance),
                verticalDistance,
                default,
                ref best);
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule3D(
                    start,
                    end,
                    length,
                    bottomFirst,
                    bottomSecond,
                    Fixed64.Zero,
                    radius,
                    out Fixed64 bottomDistance),
                bottomDistance,
                default,
                ref best);
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule3D(
                    start,
                    end,
                    length,
                    topFirst,
                    topSecond,
                    Fixed64.Zero,
                    radius,
                    out Fixed64 topDistance),
                topDistance,
                default,
                ref best);
        }
    }

    private static void TrySweepCapsuleSlabCapFace(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Fixed64 planeY,
        bool requireStartBeyondPlane,
        Vector3d featureNormal,
        ref SweepCandidate best)
    {
        bool canReach = requireStartBeyondPlane
            ? start.Y > planeY && end.Y <= planeY
            : start.Y < planeY && end.Y >= planeY;
        if (!canReach)
            return;

        Fixed64 distance = GetCapDistance(start.Y, end.Y, planeY, length);
        Vector3d point = new FixedSegment(start, end).GetPointAtDistance(distance, length);
        if (!capsule.ContainsPoint(new Vector2d(point.X, point.Z)))
            return;

        TryKeepEarlierSweep(true, distance, featureNormal, ref best);
    }

    private static void TrySweepCapsuleSlabSide(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Fixed64 targetRadius,
        Fixed64 radiusExpansion,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref SweepCandidate best)
    {
        var planarSegment = new FixedSegment2d(
            new Vector2d(start.X, start.Z),
            new Vector2d(end.X, end.Z));
        if (!planarSegment.TryGetCapsuleIntersectionDistanceInterval(
                capsule.Center,
                capsule.WorldAxis,
                capsule.AxisHalfLength,
                targetRadius,
                radiusExpansion,
                length,
                out Fixed64 distance,
                out _,
                out _,
                out _))
        {
            return;
        }

        Fixed64 y = new FixedSegment(start, end).GetPointAtDistance(distance, length).Y;
        if (y < slabMinY || y > slabMaxY)
            return;

        TryKeepEarlierSweep(true, distance, default, ref best);
    }

    private static void TrySweepCapsuleSlabBoundaryEdges(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        LSCapsuleCollider2D capsule,
        Fixed64 targetRadius,
        Fixed64 radiusExpansion,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        ref SweepCandidate best)
    {
        Vector2d firstEndpoint = capsule.SegmentStart;
        Vector2d secondEndpoint = capsule.SegmentEnd;
        Vector2d outward = new Vector2d(capsule.WorldAxis.Y, -capsule.WorldAxis.X) * targetRadius;
        Vector2d firstSideStart = firstEndpoint + outward;
        Vector2d firstSideEnd = secondEndpoint + outward;
        Vector2d secondSideStart = firstEndpoint - outward;
        Vector2d secondSideEnd = secondEndpoint - outward;

        TryKeepCapsuleSlabStraightRimSweep(
            start, end, length, firstSideStart, firstSideEnd, slabMinY, radiusExpansion, ref best);
        TryKeepCapsuleSlabStraightRimSweep(
            start, end, length, firstSideStart, firstSideEnd, slabMaxY, radiusExpansion, ref best);
        TryKeepCapsuleSlabStraightRimSweep(
            start, end, length, secondSideStart, secondSideEnd, slabMinY, radiusExpansion, ref best);
        TryKeepCapsuleSlabStraightRimSweep(
            start, end, length, secondSideStart, secondSideEnd, slabMaxY, radiusExpansion, ref best);

        Fixed64 slabCenterY = capsule.MixedSlabCenterY;
        Fixed64 slabHalfThickness = capsule.MixedHalfThickness;
        TryKeepEarlierSweep(
            new FixedSegment(start, end).TryGetSweptSphereFiniteCylinderIntersectionDistance(
                new Vector3d(firstEndpoint.X, slabCenterY, firstEndpoint.Y),
                Vector3d.Up,
                slabHalfThickness,
                targetRadius,
                radiusExpansion,
                length,
                out Fixed64 firstVerticalDistance),
            firstVerticalDistance,
            default,
            ref best);
        TryKeepEarlierSweep(
            new FixedSegment(start, end).TryGetSweptSphereFiniteCylinderIntersectionDistance(
                new Vector3d(secondEndpoint.X, slabCenterY, secondEndpoint.Y),
                Vector3d.Up,
                slabHalfThickness,
                targetRadius,
                radiusExpansion,
                length,
                out Fixed64 secondVerticalDistance),
            secondVerticalDistance,
            default,
            ref best);
    }

    private static void TryKeepCapsuleSlabStraightRimSweep(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        Vector2d planarStart,
        Vector2d planarEnd,
        Fixed64 y,
        Fixed64 radiusExpansion,
        ref SweepCandidate best)
    {
        TryKeepEarlierSweep(
            TrySweepPointAgainstSegmentCapsule3D(
                start,
                end,
                length,
                new Vector3d(planarStart.X, y, planarStart.Y),
                new Vector3d(planarEnd.X, y, planarEnd.Y),
                Fixed64.Zero,
                radiusExpansion,
                out Fixed64 distance),
            distance,
            default,
            ref best);
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
        Vector3d end,
        Fixed64 length,
        Vector3d segmentStart,
        Vector3d segmentEnd,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 distance)
    {
        return new FixedSegment(start, end).TryGetCapsuleIntersectionDistanceInterval(
                new FixedSegment(segmentStart, segmentEnd),
                radius,
                radiusExpansion,
                length,
                out distance,
                out _,
                out _,
                out _);
    }

    private static Fixed64 GetCapDistance(
        Fixed64 start,
        Fixed64 end,
        Fixed64 plane,
        Fixed64 length)
    {
        // Cap admission proves that plane lies between start and end. The public
        // sweep contract has already proved the segment displacement and length
        // representable, so the fused result is bounded by length and cannot fail.
        Fixed64 verticalSpan = end - start;
        Fixed64 capOffset = plane - start;
        _ = Fixed64.TryMultiplyDivide(length, capOffset, verticalSpan, out Fixed64 distance);
        return distance;
    }

    private static void TryKeepEarlierSweep(
        bool candidateFound,
        Fixed64 candidateDistance,
        Vector3d candidateFeatureNormal,
        ref SweepCandidate best)
    {
        if (!candidateFound)
            return;

        if (best.Found && candidateDistance >= best.Distance)
            return;

        best.Found = true;
        best.Distance = candidateDistance;
        best.FeatureNormal = candidateFeatureNormal;
    }

    private struct SweepCandidate
    {
        public bool Found;
        public Fixed64 Distance;
        public Vector3d FeatureNormal;
    }

}
