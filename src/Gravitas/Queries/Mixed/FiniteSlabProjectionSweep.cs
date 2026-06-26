//=======================================================================
// FiniteSlabProjectionSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Sweeps an X/Z circle against the projection of curved 3D targets clipped to
/// a finite Y slab.
/// </summary>
internal static class FiniteSlabProjectionSweep
{
    private const int MaxGjkIterations = 32;
    private const int MaxConservativeAdvancementIterations = 32;
    private static readonly Fixed64 DistanceTolerance = Fixed64.FromFraction(1, 1_048_576);
    private static readonly Fixed64 DistanceToleranceSqr = DistanceTolerance * DistanceTolerance;
    private static readonly Fixed64 SweepContactTolerance = DistanceTolerance;
    private static readonly Fixed64 ProgressToleranceSqr = DistanceToleranceSqr;

    public static bool TrySweepCircleAgainstCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        LSCapsuleCollider capsule,
        out Fixed64 distance)
    {
        var target = ProjectionTarget.CreateCapsule(capsule, slabMinY, slabMaxY);
        return TrySweepCircle(start, direction, length, radius, target, out distance);
    }

    public static bool TrySweepCircleAgainstCylinder(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        LSCylinderCollider cylinder,
        out Fixed64 distance)
    {
        var target = ProjectionTarget.CreateCylinder(cylinder, slabMinY, slabMaxY);
        return TrySweepCircle(start, direction, length, radius, target, out distance);
    }

    private static bool TrySweepCircle(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        ProjectionTarget target,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (!target.HasProjection)
            return false;

        Fixed64 travelDistance = Fixed64.Zero;
        for (int i = 0; i < MaxConservativeAdvancementIterations; i++)
        {
            Vector2d point = start + direction * travelDistance;
            PlanarGjkResult result = ComputeDistance(point, radius, target);
            if (result.Intersects || result.Distance <= SweepContactTolerance)
            {
                distance = travelDistance;
                return true;
            }

            Vector2d normal = result.Normal;
            Fixed64 closingSpeed = -Vector2d.Dot(direction, normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 stepDistance = result.Distance / closingSpeed;
            if (stepDistance <= Fixed64.Zero)
                return false;

            travelDistance += stepDistance;
            if (travelDistance > length)
                return false;
        }

        return false;
    }

    private static PlanarGjkResult ComputeDistance(Vector2d point, Fixed64 expansionRadius, ProjectionTarget target)
    {
        Span<PlanarSupportPoint> simplex = stackalloc PlanarSupportPoint[3];
        int simplexCount = 0;
        Vector2d direction = target.Center - point;
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            direction = Vector2d.Right;

        Fixed64 previousDistanceSqr = Fixed64.MaxValue;
        ClosestPlanarSimplexResult closest = default;

        for (int i = 0; i < MaxGjkIterations; i++)
        {
            if (!TryCreateSupportPoint(point, expansionRadius, target, direction, out PlanarSupportPoint support))
                return PlanarGjkResult.Separated;

            if (ContainsSupportPoint(simplex, simplexCount, support.Point))
                break;

            simplex[simplexCount++] = support;
            closest = SolveClosestSimplex(simplex, ref simplexCount);
            if (closest.Intersects || closest.DistanceSqr <= DistanceToleranceSqr)
                return PlanarGjkResult.Intersection;

            if (previousDistanceSqr - closest.DistanceSqr <= ProgressToleranceSqr)
                break;

            previousDistanceSqr = closest.DistanceSqr;
            direction = -closest.Point;
            if (direction.MagnitudeSquared <= DistanceToleranceSqr)
                return PlanarGjkResult.Intersection;
        }

        Fixed64 distance = closest.DistanceSqr <= Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Sqrt(closest.DistanceSqr);
        Vector2d normal = closest.Point.MagnitudeSquared > Fixed64.Epsilon
            ? closest.Point.Normalized
            : Vector2d.Zero;
        return new PlanarGjkResult(false, distance, normal);
    }

    private static bool TryCreateSupportPoint(
        Vector2d point,
        Fixed64 expansionRadius,
        ProjectionTarget target,
        Vector2d direction,
        out PlanarSupportPoint support)
    {
        Vector2d targetDirection = -direction;
        if (!target.TrySupport(targetDirection, out Vector2d targetSupport))
        {
            support = default;
            return false;
        }

        Vector2d expansion = NormalizePlanarDirection(targetDirection) * expansionRadius;
        support = new PlanarSupportPoint(point - (targetSupport + expansion));
        return true;
    }

    private static ClosestPlanarSimplexResult SolveClosestSimplex(Span<PlanarSupportPoint> simplex, ref int count)
    {
        if (count == 1)
            return ClosestPlanarSimplexResult.FromPoint(simplex[0].Point);

        if (count == 2)
            return ReduceSegment(simplex, ref count);

        return ReduceTriangle(simplex, ref count);
    }

    private static ClosestPlanarSimplexResult ReduceSegment(Span<PlanarSupportPoint> simplex, ref int count)
    {
        Vector2d a = simplex[0].Point;
        Vector2d b = simplex[1].Point;
        Vector2d ab = b - a;
        Fixed64 denominator = ab.MagnitudeSquared;
        Fixed64 t = denominator <= Fixed64.Epsilon
            ? Fixed64.Zero
            : FixedMath.Clamp(-Vector2d.Dot(a, ab) / denominator, Fixed64.Zero, Fixed64.One);

        if (t <= Fixed64.Epsilon)
        {
            count = 1;
            return ClosestPlanarSimplexResult.FromPoint(a);
        }

        if (t >= Fixed64.One - Fixed64.Epsilon)
        {
            simplex[0] = simplex[1];
            count = 1;
            return ClosestPlanarSimplexResult.FromPoint(b);
        }

        count = 2;
        return ClosestPlanarSimplexResult.FromPoint(a + ab * t);
    }

    private static ClosestPlanarSimplexResult ReduceTriangle(Span<PlanarSupportPoint> simplex, ref int count)
    {
        Vector2d a = simplex[0].Point;
        Vector2d b = simplex[1].Point;
        Vector2d c = simplex[2].Point;
        if (IsOriginInsideTriangle(a, b, c))
        {
            count = 3;
            return ClosestPlanarSimplexResult.Intersection;
        }

        Fixed64 bestDistanceSqr = Fixed64.MaxValue;
        int bestFirst = 0;
        int bestSecond = 1;
        ClosestPlanarSimplexResult best = default;
        EvaluateTriangleEdge(simplex, 0, 1, ref best, ref bestDistanceSqr, ref bestFirst, ref bestSecond);
        EvaluateTriangleEdge(simplex, 1, 2, ref best, ref bestDistanceSqr, ref bestFirst, ref bestSecond);
        EvaluateTriangleEdge(simplex, 2, 0, ref best, ref bestDistanceSqr, ref bestFirst, ref bestSecond);

        PlanarSupportPoint first = simplex[bestFirst];
        PlanarSupportPoint second = simplex[bestSecond];
        simplex[0] = first;
        simplex[1] = second;
        count = 2;
        return best;
    }

    private static void EvaluateTriangleEdge(
        Span<PlanarSupportPoint> simplex,
        int first,
        int second,
        ref ClosestPlanarSimplexResult best,
        ref Fixed64 bestDistanceSqr,
        ref int bestFirst,
        ref int bestSecond)
    {
        Span<PlanarSupportPoint> edge = stackalloc PlanarSupportPoint[2];
        edge[0] = simplex[first];
        edge[1] = simplex[second];
        int edgeCount = 2;
        ClosestPlanarSimplexResult candidate = ReduceSegment(edge, ref edgeCount);
        if (candidate.DistanceSqr >= bestDistanceSqr)
            return;

        best = candidate;
        bestDistanceSqr = candidate.DistanceSqr;
        bestFirst = first;
        bestSecond = second;
    }

    private static bool IsOriginInsideTriangle(Vector2d a, Vector2d b, Vector2d c)
    {
        if (Cross(a, b, c).Abs() <= DistanceTolerance)
            return false;

        Fixed64 ab = Cross(a, b, Vector2d.Zero);
        Fixed64 bc = Cross(b, c, Vector2d.Zero);
        Fixed64 ca = Cross(c, a, Vector2d.Zero);
        bool hasPositive = ab > Fixed64.Epsilon || bc > Fixed64.Epsilon || ca > Fixed64.Epsilon;
        bool hasNegative = ab < -Fixed64.Epsilon || bc < -Fixed64.Epsilon || ca < -Fixed64.Epsilon;
        return !(hasPositive && hasNegative);
    }

    private static bool ContainsSupportPoint(Span<PlanarSupportPoint> simplex, int count, Vector2d point)
    {
        for (int i = 0; i < count; i++)
        {
            if ((simplex[i].Point - point).MagnitudeSquared <= DistanceToleranceSqr)
                return true;
        }

        return false;
    }

    private readonly struct ProjectionTarget
    {
        private readonly LSCapsuleCollider? _capsule;
        private readonly LSCylinderCollider? _cylinder;
        private readonly Fixed64 _slabMinY;
        private readonly Fixed64 _slabMaxY;
        private readonly Vector2d _center;

        private ProjectionTarget(
            LSCapsuleCollider? capsule,
            LSCylinderCollider? cylinder,
            Fixed64 slabMinY,
            Fixed64 slabMaxY,
            Vector2d center)
        {
            _capsule = capsule;
            _cylinder = cylinder;
            _slabMinY = slabMinY;
            _slabMaxY = slabMaxY;
            _center = center;
        }

        public Vector2d Center => _center;

        public bool HasProjection
        {
            get
            {
                if (_capsule != null)
                    return CapsuleIntersectsSlab(_capsule, _slabMinY, _slabMaxY);

                return _cylinder != null && CylinderIntersectsSlab(_cylinder, _slabMinY, _slabMaxY);
            }
        }

        public static ProjectionTarget CreateCapsule(LSCapsuleCollider capsule, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(capsule, null, slabMinY, slabMaxY, new Vector2d(capsule.Center.X, capsule.Center.Z));

        public static ProjectionTarget CreateCylinder(LSCylinderCollider cylinder, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(null, cylinder, slabMinY, slabMaxY, new Vector2d(cylinder.Center.X, cylinder.Center.Z));

        public bool TrySupport(Vector2d direction, out Vector2d support)
        {
            Vector2d normal = NormalizePlanarDirection(direction);
            if (_capsule != null)
                return TrySupportCapsuleProjection(_capsule, _slabMinY, _slabMaxY, normal, out support);

            return TrySupportCylinderProjection(_cylinder!, _slabMinY, _slabMaxY, normal, out support);
        }
    }

    private static bool TrySupportCapsuleProjection(
        LSCapsuleCollider capsule,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        out Vector2d support)
    {
        Vector3d segment = capsule.LineSegmentEnd - capsule.LineSegmentStart;
        Fixed64 dy = segment.Y;
        Fixed64 planarSlope = segment.X * direction.X + segment.Z * direction.Y;
        bool found = false;
        Vector2d best = default;

        // The clipped capsule projection is piecewise over the capsule axis:
        // axis endpoints, slab boundary crossings, and outside-slab stationary
        // points are the only support extrema for a fixed planar direction.
        TryKeepCapsuleSupport(capsule, slabMinY, slabMaxY, direction, Fixed64.Zero, ref found, ref best);
        TryKeepCapsuleSupport(capsule, slabMinY, slabMaxY, direction, Fixed64.One, ref found, ref best);
        if (dy.Abs() > Fixed64.Epsilon)
        {
            AddCapsuleYBoundaryCandidates(capsule, slabMinY, slabMaxY, direction, dy, ref found, ref best);
            AddCapsuleStationaryCandidate(capsule, slabMinY, direction, planarSlope, dy, belowSlab: true, ref found, ref best);
            AddCapsuleStationaryCandidate(capsule, slabMaxY, direction, planarSlope, dy, belowSlab: false, ref found, ref best);
        }

        support = best;
        return found;
    }

    private static void AddCapsuleYBoundaryCandidates(
        LSCapsuleCollider capsule,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        Fixed64 dy,
        ref bool found,
        ref Vector2d best)
    {
        Fixed64 startY = capsule.LineSegmentStart.Y;
        Fixed64 radius = capsule.ScaledRadius;
        TryKeepCapsuleSupport(capsule, slabMinY, slabMaxY, direction, (slabMinY - radius - startY) / dy, ref found, ref best);
        TryKeepCapsuleSupport(capsule, slabMinY, slabMaxY, direction, (slabMinY - startY) / dy, ref found, ref best);
        TryKeepCapsuleSupport(capsule, slabMinY, slabMaxY, direction, (slabMaxY - startY) / dy, ref found, ref best);
        TryKeepCapsuleSupport(capsule, slabMinY, slabMaxY, direction, (slabMaxY + radius - startY) / dy, ref found, ref best);
    }

    private static void AddCapsuleStationaryCandidate(
        LSCapsuleCollider capsule,
        Fixed64 slabPlaneY,
        Vector2d direction,
        Fixed64 planarSlope,
        Fixed64 dy,
        bool belowSlab,
        ref bool found,
        ref Vector2d best)
    {
        Fixed64 denominator = dy * dy + planarSlope * planarSlope;
        if (denominator <= Fixed64.Epsilon)
            return;

        Fixed64 excessMagnitude = planarSlope.Abs() * capsule.ScaledRadius / FixedMath.Sqrt(denominator);
        Fixed64 signProbe = belowSlab ? -planarSlope / dy : planarSlope / dy;
        if (signProbe < Fixed64.Zero)
            return;

        Fixed64 excess = excessMagnitude;
        Fixed64 startY = capsule.LineSegmentStart.Y;
        Fixed64 u = belowSlab
            ? (slabPlaneY - startY - excess) / dy
            : (slabPlaneY + excess - startY) / dy;
        TryKeepCapsuleSupport(capsule, slabPlaneY, slabPlaneY, direction, u, ref found, ref best);
    }

    private static void TryKeepCapsuleSupport(
        LSCapsuleCollider capsule,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        Fixed64 u,
        ref bool found,
        ref Vector2d best)
    {
        if (!TryClampUnitParameter(u, out Fixed64 clampedU))
            return;

        Vector3d point = capsule.LineSegmentStart + (capsule.LineSegmentEnd - capsule.LineSegmentStart) * clampedU;
        Fixed64 verticalExcess = GetPointIntervalDistance(point.Y, slabMinY, slabMaxY);
        Fixed64 radius = capsule.ScaledRadius;
        if (verticalExcess > radius)
            return;

        Fixed64 planarRadiusSqr = radius * radius - verticalExcess * verticalExcess;
        Fixed64 planarRadius = planarRadiusSqr <= Fixed64.Zero ? Fixed64.Zero : FixedMath.Sqrt(planarRadiusSqr);
        Vector2d candidate = new(point.X + direction.X * planarRadius, point.Z + direction.Y * planarRadius);
        KeepBestSupport(candidate, direction, ref found, ref best);
    }

    private static bool TrySupportCylinderProjection(
        LSCylinderCollider cylinder,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        out Vector2d support)
    {
        Vector3d axis = cylinder.LineSegmentEnd - cylinder.LineSegmentStart;
        Fixed64 dy = axis.Y;
        Fixed64 planarSlope = axis.X * direction.X + axis.Z * direction.Y;
        bool found = false;
        Vector2d best = default;

        TryKeepCylinderSupport(cylinder, slabMinY, slabMaxY, direction, Fixed64.Zero, ref found, ref best);
        TryKeepCylinderSupport(cylinder, slabMinY, slabMaxY, direction, Fixed64.One, ref found, ref best);

        Fixed64 verticalRadialCapacity = GetCylinderVerticalRadialCapacity(cylinder);
        if (dy.Abs() > Fixed64.Epsilon)
        {
            // A clipped cylinder disk support can change only at axis endpoints,
            // slab plane intersections, or stationary points along a slab plane.
            AddCylinderBoundaryCandidates(cylinder, slabMinY, slabMaxY, direction, dy, verticalRadialCapacity, ref found, ref best);
            AddCylinderStationaryCandidates(cylinder, slabMinY, direction, planarSlope, dy, ref found, ref best);
            AddCylinderStationaryCandidates(cylinder, slabMaxY, direction, planarSlope, dy, ref found, ref best);
        }

        support = best;
        return found;
    }

    private static void AddCylinderBoundaryCandidates(
        LSCylinderCollider cylinder,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        Fixed64 dy,
        Fixed64 verticalRadialCapacity,
        ref bool found,
        ref Vector2d best)
    {
        Fixed64 startY = cylinder.LineSegmentStart.Y;
        TryGetCylinderUnconstrainedRadialY(cylinder, direction, out Fixed64 unconstrainedRadialY);

        AddCylinderPlaneCandidates(cylinder, slabMinY, slabMinY, slabMaxY, direction, dy, startY, verticalRadialCapacity, unconstrainedRadialY, ref found, ref best);
        AddCylinderPlaneCandidates(cylinder, slabMaxY, slabMinY, slabMaxY, direction, dy, startY, verticalRadialCapacity, unconstrainedRadialY, ref found, ref best);
    }

    private static void AddCylinderPlaneCandidates(
        LSCylinderCollider cylinder,
        Fixed64 planeY,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        Fixed64 dy,
        Fixed64 startY,
        Fixed64 verticalRadialCapacity,
        Fixed64 unconstrainedRadialY,
        ref bool found,
        ref Vector2d best)
    {
        TryKeepCylinderSupport(cylinder, slabMinY, slabMaxY, direction, (planeY - startY - verticalRadialCapacity) / dy, ref found, ref best);
        TryKeepCylinderSupport(cylinder, slabMinY, slabMaxY, direction, (planeY - startY) / dy, ref found, ref best);
        TryKeepCylinderSupport(cylinder, slabMinY, slabMaxY, direction, (planeY - startY + verticalRadialCapacity) / dy, ref found, ref best);
        TryKeepCylinderSupport(cylinder, slabMinY, slabMaxY, direction, (planeY - startY - unconstrainedRadialY) / dy, ref found, ref best);
    }

    private static void AddCylinderStationaryCandidates(
        LSCylinderCollider cylinder,
        Fixed64 planeY,
        Vector2d direction,
        Fixed64 planarSlope,
        Fixed64 dy,
        ref bool found,
        ref Vector2d best)
    {
        Vector3d axisDirection = cylinder.LineDirection;
        Vector3d verticalInRadialPlane = Vector3d.Up - axisDirection * axisDirection.Y;
        Fixed64 verticalCapacitySqr = verticalInRadialPlane.MagnitudeSquared;
        if (verticalCapacitySqr <= Fixed64.Epsilon)
            return;

        Vector3d direction3D = new(direction.X, Fixed64.Zero, direction.Y);
        Fixed64 linearBoundarySlope = Vector3d.Dot(direction3D, verticalInRadialPlane) / verticalCapacitySqr;
        Vector3d tangent = Vector3d.Cross(axisDirection, verticalInRadialPlane);
        Fixed64 tangentMagnitude = tangent.Magnitude;
        if (tangentMagnitude <= Fixed64.Epsilon)
            return;

        Fixed64 tangentProjection = Vector3d.Dot(direction3D, tangent / tangentMagnitude).Abs();
        if (tangentProjection <= Fixed64.Epsilon)
            return;

        Fixed64 k = planarSlope - linearBoundarySlope * dy;
        Fixed64 m = -k * verticalCapacitySqr / (tangentProjection * dy);
        Fixed64 mSqr = m * m;
        Fixed64 denominator = verticalCapacitySqr + mSqr;
        if (denominator <= Fixed64.Epsilon)
            return;

        Fixed64 qSqr = mSqr * cylinder.ScaledRadiusSqr * verticalCapacitySqr / denominator;
        Fixed64 q = FixedMath.Sqrt(qSqr);
        if (m < Fixed64.Zero)
            q = -q;

        Fixed64 u = (planeY - cylinder.LineSegmentStart.Y - q) / dy;
        TryKeepCylinderSupport(cylinder, planeY, planeY, direction, u, ref found, ref best);
    }

    private static void TryKeepCylinderSupport(
        LSCylinderCollider cylinder,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        Fixed64 u,
        ref bool found,
        ref Vector2d best)
    {
        if (!TryClampUnitParameter(u, out Fixed64 clampedU))
            return;

        Vector3d axisPoint = cylinder.LineSegmentStart + (cylinder.LineSegmentEnd - cylinder.LineSegmentStart) * clampedU;
        if (!TrySupportCylinderDiskInBand(axisPoint, cylinder.LineDirection, cylinder.ScaledRadius, slabMinY, slabMaxY, direction, out Vector2d candidate))
            return;

        KeepBestSupport(candidate, direction, ref found, ref best);
    }

    private static bool TrySupportCylinderDiskInBand(
        Vector3d axisPoint,
        Vector3d axisDirection,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        out Vector2d support)
    {
        bool found = false;
        Vector2d best = default;
        Vector3d direction3D = new(direction.X, Fixed64.Zero, direction.Y);
        Vector3d radialDirection = direction3D - axisDirection * Vector3d.Dot(direction3D, axisDirection);
        Fixed64 radialMagnitude = radialDirection.Magnitude;
        if (radialMagnitude > Fixed64.Epsilon)
        {
            Vector3d radial = radialDirection / radialMagnitude * radius;
            Fixed64 y = axisPoint.Y + radial.Y;
            if (y >= slabMinY && y <= slabMaxY)
            {
                KeepBestSupport(new Vector2d(axisPoint.X + radial.X, axisPoint.Z + radial.Z), direction, ref found, ref best);
            }
        }

        TryKeepCylinderDiskBoundary(axisPoint, axisDirection, radius, slabMinY, direction, ref found, ref best);
        if (slabMaxY != slabMinY)
            TryKeepCylinderDiskBoundary(axisPoint, axisDirection, radius, slabMaxY, direction, ref found, ref best);

        support = best;
        return found;
    }

    private static void TryKeepCylinderDiskBoundary(
        Vector3d axisPoint,
        Vector3d axisDirection,
        Fixed64 radius,
        Fixed64 planeY,
        Vector2d direction,
        ref bool found,
        ref Vector2d best)
    {
        if (!TryGetCylinderDiskBoundaryRadial(axisDirection, radius, planeY - axisPoint.Y, direction, out Vector3d radial))
            return;

        KeepBestSupport(new Vector2d(axisPoint.X + radial.X, axisPoint.Z + radial.Z), direction, ref found, ref best);
    }

    private static bool TryGetCylinderDiskBoundaryRadial(
        Vector3d axisDirection,
        Fixed64 radius,
        Fixed64 yOffset,
        Vector2d direction,
        out Vector3d radial)
    {
        Vector3d verticalInRadialPlane = Vector3d.Up - axisDirection * axisDirection.Y;
        Fixed64 verticalCapacitySqr = verticalInRadialPlane.MagnitudeSquared;
        if (verticalCapacitySqr <= Fixed64.Epsilon)
        {
            if (yOffset.Abs() > Fixed64.Epsilon)
            {
                radial = default;
                return false;
            }

            Vector3d planarDirection = new(direction.X, Fixed64.Zero, direction.Y);
            Vector3d radialDirection = planarDirection - axisDirection * Vector3d.Dot(planarDirection, axisDirection);
            Fixed64 radialMagnitude = radialDirection.Magnitude;
            radial = radialMagnitude > Fixed64.Epsilon
                ? radialDirection / radialMagnitude * radius
                : Vector3d.Right * radius;
            return true;
        }

        Fixed64 maxYOffsetSqr = radius * radius * verticalCapacitySqr;
        if (yOffset * yOffset > maxYOffsetSqr + Fixed64.Epsilon)
        {
            radial = default;
            return false;
        }

        Vector3d baseRadial = verticalInRadialPlane * (yOffset / verticalCapacitySqr);
        Fixed64 baseMagnitudeSqr = yOffset * yOffset / verticalCapacitySqr;
        Fixed64 remainingSqr = radius * radius - baseMagnitudeSqr;
        if (remainingSqr < Fixed64.Zero)
            remainingSqr = Fixed64.Zero;

        Vector3d tangent = Vector3d.Cross(axisDirection, verticalInRadialPlane);
        Fixed64 tangentMagnitude = tangent.Magnitude;
        if (tangentMagnitude <= Fixed64.Epsilon || remainingSqr <= Fixed64.Epsilon)
        {
            radial = baseRadial;
            return true;
        }

        Vector3d tangentDirection = tangent / tangentMagnitude;
        Vector3d direction3D = new(direction.X, Fixed64.Zero, direction.Y);
        Fixed64 sign = Vector3d.Dot(direction3D, tangentDirection) >= Fixed64.Zero ? Fixed64.One : -Fixed64.One;
        radial = baseRadial + tangentDirection * sign * FixedMath.Sqrt(remainingSqr);
        return true;
    }

    private static bool CapsuleIntersectsSlab(LSCapsuleCollider capsule, Fixed64 slabMinY, Fixed64 slabMaxY)
    {
        Fixed64 minY = FixedMath.Min(capsule.LineSegmentStart.Y, capsule.LineSegmentEnd.Y) - capsule.ScaledRadius;
        Fixed64 maxY = FixedMath.Max(capsule.LineSegmentStart.Y, capsule.LineSegmentEnd.Y) + capsule.ScaledRadius;
        return maxY >= slabMinY && minY <= slabMaxY;
    }

    private static bool CylinderIntersectsSlab(LSCylinderCollider cylinder, Fixed64 slabMinY, Fixed64 slabMaxY)
    {
        Fixed64 minAxisY = FixedMath.Min(cylinder.LineSegmentStart.Y, cylinder.LineSegmentEnd.Y);
        Fixed64 maxAxisY = FixedMath.Max(cylinder.LineSegmentStart.Y, cylinder.LineSegmentEnd.Y);
        Fixed64 radialY = GetCylinderVerticalRadialCapacity(cylinder);
        return maxAxisY + radialY >= slabMinY && minAxisY - radialY <= slabMaxY;
    }

    private static Fixed64 GetCylinderVerticalRadialCapacity(LSCylinderCollider cylinder)
    {
        Fixed64 capacitySqr = Fixed64.One - cylinder.LineDirection.Y * cylinder.LineDirection.Y;
        return capacitySqr <= Fixed64.Zero ? Fixed64.Zero : cylinder.ScaledRadius * FixedMath.Sqrt(capacitySqr);
    }

    private static bool TryGetCylinderUnconstrainedRadialY(LSCylinderCollider cylinder, Vector2d direction, out Fixed64 radialY)
    {
        Vector3d direction3D = new(direction.X, Fixed64.Zero, direction.Y);
        Vector3d radialDirection = direction3D - cylinder.LineDirection * Vector3d.Dot(direction3D, cylinder.LineDirection);
        Fixed64 radialMagnitude = radialDirection.Magnitude;
        if (radialMagnitude <= Fixed64.Epsilon)
        {
            radialY = Fixed64.Zero;
            return false;
        }

        radialY = radialDirection.Y / radialMagnitude * cylinder.ScaledRadius;
        return true;
    }

    private static Fixed64 GetPointIntervalDistance(Fixed64 point, Fixed64 min, Fixed64 max)
    {
        if (point < min)
            return min - point;

        if (point > max)
            return point - max;

        return Fixed64.Zero;
    }

    private static bool TryClampUnitParameter(Fixed64 value, out Fixed64 clamped)
    {
        if (value < -Fixed64.Epsilon || value > Fixed64.One + Fixed64.Epsilon)
        {
            clamped = default;
            return false;
        }

        clamped = FixedMath.Clamp01(value);
        return true;
    }

    private static void KeepBestSupport(Vector2d candidate, Vector2d direction, ref bool found, ref Vector2d best)
    {
        if (found)
        {
            Fixed64 candidateProjection = Vector2d.Dot(candidate, direction);
            Fixed64 bestProjection = Vector2d.Dot(best, direction);
            if (candidateProjection < bestProjection)
                return;

            if (candidateProjection == bestProjection && ComesAfter(candidate, best))
                return;
        }

        best = candidate;
        found = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d NormalizePlanarDirection(Vector2d direction) =>
        direction.MagnitudeSquared > Fixed64.Epsilon ? direction.Normalized : Vector2d.Right;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ComesAfter(Vector2d first, Vector2d second) =>
        first.X > second.X || (first.X == second.X && first.Y > second.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Cross(Vector2d origin, Vector2d first, Vector2d second) =>
        (first.X - origin.X) * (second.Y - origin.Y) - (first.Y - origin.Y) * (second.X - origin.X);

    private readonly struct PlanarSupportPoint
    {
        public PlanarSupportPoint(Vector2d point)
        {
            Point = point;
        }

        public Vector2d Point { get; }
    }

    private readonly struct PlanarGjkResult
    {
        public PlanarGjkResult(bool intersects, Fixed64 distance, Vector2d normal)
        {
            Intersects = intersects;
            Distance = distance;
            Normal = normal;
        }

        public bool Intersects { get; }

        public Fixed64 Distance { get; }

        public Vector2d Normal { get; }

        public static PlanarGjkResult Intersection => new(true, Fixed64.Zero, Vector2d.Zero);

        public static PlanarGjkResult Separated => new(false, Fixed64.MaxValue, Vector2d.Zero);
    }

    private readonly struct ClosestPlanarSimplexResult
    {
        private ClosestPlanarSimplexResult(bool intersects, Vector2d point)
        {
            Intersects = intersects;
            Point = point;
            DistanceSqr = point.MagnitudeSquared;
        }

        public bool Intersects { get; }

        public Vector2d Point { get; }

        public Fixed64 DistanceSqr { get; }

        public static ClosestPlanarSimplexResult Intersection => new(true, Vector2d.Zero);

        public static ClosestPlanarSimplexResult FromPoint(Vector2d point) => new(false, point);
    }
}
