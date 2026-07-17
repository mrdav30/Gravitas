//=======================================================================
// FiniteSlabProjectionSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Sweeps an X/Z circle against the projection of curved 3D targets clipped to
/// a finite Y slab.
/// </summary>
internal static partial class FiniteSlabProjectionSweep
{
    private const int MaxGjkIterations = 32;
    private const int MaxConservativeAdvancementIterations = 32;
    private static readonly Fixed64 DistanceTolerance = Fixed64.FromFraction(1, 1_048_576);
    private static readonly Fixed64 SweepContactTolerance = DistanceTolerance;

    public static bool TrySweepCircleAgainstCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        LSCapsuleCollider capsule,
        out Fixed64 distance,
        int maxConservativeAdvancementIterations = MaxConservativeAdvancementIterations)
    {
        var target = ProjectionTarget.CreateCapsule(capsule, slabMinY, slabMaxY);
        return TrySweepCircle(
            start,
            direction,
            length,
            radius,
            target,
            out distance,
            maxConservativeAdvancementIterations);
    }

    public static bool TrySweepCircleAgainstCylinder(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        LSCylinderCollider cylinder,
        out Fixed64 distance,
        int maxConservativeAdvancementIterations = MaxConservativeAdvancementIterations)
    {
        var target = ProjectionTarget.CreateCylinder(cylinder, slabMinY, slabMaxY);
        return TrySweepCircle(
            start,
            direction,
            length,
            radius,
            target,
            out distance,
            maxConservativeAdvancementIterations);
    }

    public static bool TrySweepCircleAgainstCone(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        LSConeCollider cone,
        out Fixed64 distance,
        int maxConservativeAdvancementIterations = MaxConservativeAdvancementIterations)
    {
        var target = ProjectionTarget.CreateCone(cone, slabMinY, slabMaxY);
        return TrySweepCircle(
            start,
            direction,
            length,
            radius,
            target,
            out distance,
            maxConservativeAdvancementIterations);
    }

    private static bool TrySweepCircle(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        ProjectionTarget target,
        out Fixed64 distance,
        int maxConservativeAdvancementIterations)
    {
        distance = Fixed64.Zero;
        if (!Vector2d.TryGetMagnitude(direction, out Fixed64 directionMagnitude)
            || (directionMagnitude != Fixed64.Zero
                && FixedMath.Abs(directionMagnitude - Fixed64.One) > Fixed64.Epsilon)
            || !target.HasProjection)
            return false;

        Fixed64 travelDistance = Fixed64.Zero;
        for (int i = 0; i < maxConservativeAdvancementIterations; i++)
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
            travelDistance += stepDistance;
            if (travelDistance > length)
            {
                PlanarGjkResult endpoint = ComputeDistance(start + direction * length, radius, target);
                if (endpoint.Intersects || endpoint.Distance <= SweepContactTolerance)
                {
                    distance = length;
                    return true;
                }

                return false;
            }
        }

        return false;
    }

    private static PlanarGjkResult ComputeDistance(Vector2d point, Fixed64 expansionRadius, ProjectionTarget target)
    {
        Span<PlanarSupportPoint> simplex = stackalloc PlanarSupportPoint[3];
        int workingShift = GjkSimplexScale.SelectThreeTermShift(
            point,
            target.BoundsMin,
            target.BoundsMax,
            expansionRadius);
        Fixed64 workingScale = GjkSimplexScale.GetCoordinateScale(workingShift);
        Fixed64 workingScaleSqr = workingScale * workingScale;
        Fixed64 workingDistanceTolerance = DistanceTolerance * workingScale;
        Fixed64 workingSegmentDegeneracyToleranceSqr = Fixed64.Epsilon * workingScaleSqr;
        Fixed64 workingAreaTolerance = DistanceTolerance * workingScaleSqr;
        Fixed64 workingCrossSignTolerance = Fixed64.Epsilon * workingScaleSqr;
        int simplexCount = 0;
        Vector2d direction = GjkSimplexScale.CreateWorkingDifference(target.Center, point, workingShift);
        if (direction == Vector2d.Zero)
            direction = Vector2d.Right;

        bool hasPreviousDistance = false;
        Fixed64 previousDistance = Fixed64.Zero;
        ClosestPlanarSimplexResult closest = default;
        bool distanceIsRepresentable = false;
        Fixed64 workingDistance = Fixed64.MaxValue;

        for (int i = 0; i < MaxGjkIterations; i++)
        {
            CreateSupportPoint(
                point,
                expansionRadius,
                target,
                direction,
                workingShift,
                out PlanarSupportPoint support);

            if (ContainsSupportPoint(simplex, simplexCount, support.Point))
                break;

            simplex[simplexCount++] = support;
            closest = SolveClosestSimplex(
                simplex,
                ref simplexCount,
                workingSegmentDegeneracyToleranceSqr,
                workingAreaTolerance,
                workingCrossSignTolerance);
            distanceIsRepresentable = Vector2d.TryGetMagnitude(closest.Point, out workingDistance);
            if (closest.Intersects
                || (distanceIsRepresentable && workingDistance <= workingDistanceTolerance))
                return PlanarGjkResult.Intersection;

            if (hasPreviousDistance
                && distanceIsRepresentable
                && previousDistance - workingDistance <= Fixed64.Epsilon)
            {
                break;
            }

            hasPreviousDistance = distanceIsRepresentable;
            previousDistance = workingDistance;
            direction = -closest.Point;
        }

        Fixed64 distance = GjkSimplexScale.RestoreDistance(workingDistance, workingShift);
        Vector2d normal = closest.Point.Normalized;
        return new PlanarGjkResult(false, distance, normal);
    }

    private static void CreateSupportPoint(
        Vector2d point,
        Fixed64 expansionRadius,
        ProjectionTarget target,
        Vector2d direction,
        int workingShift,
        out PlanarSupportPoint support)
    {
        Vector2d supportDirection = direction.Normalized;
        Vector2d targetDirection = -supportDirection;
        target.TrySupport(targetDirection, out Vector2d targetSupport);

        Vector2d expansion = targetDirection * expansionRadius;
        support = new PlanarSupportPoint(
            GjkSimplexScale.CreateWorkingDifference(point, targetSupport, expansion, workingShift));
    }

    private static ClosestPlanarSimplexResult SolveClosestSimplex(
        Span<PlanarSupportPoint> simplex,
        ref int count,
        Fixed64 segmentDegeneracyToleranceSqr,
        Fixed64 areaTolerance,
        Fixed64 crossSignTolerance)
    {
        if (count == 1)
            return ClosestPlanarSimplexResult.FromPoint(simplex[0].Point);

        if (count == 2)
            return ReduceSegment(simplex, ref count, segmentDegeneracyToleranceSqr);

        return ReduceTriangle(
            simplex,
            ref count,
            segmentDegeneracyToleranceSqr,
            areaTolerance,
            crossSignTolerance);
    }

    private static ClosestPlanarSimplexResult ReduceSegment(
        Span<PlanarSupportPoint> simplex,
        ref int count,
        Fixed64 segmentDegeneracyToleranceSqr)
    {
        Vector2d a = simplex[0].Point;
        Vector2d b = simplex[1].Point;
        Span<Vector2d> scaled = stackalloc Vector2d[2];
        scaled[0] = a;
        scaled[1] = b;
        Fixed64 productScale = GjkSimplexScale.ScaleForProducts(scaled);
        Vector2d scaledA = scaled[0];
        Vector2d ab = scaled[1] - scaledA;
        Fixed64 denominator = ab.MagnitudeSquared;
        Fixed64 productScaleSqr = productScale * productScale;
        Fixed64 denominatorTolerance = segmentDegeneracyToleranceSqr * productScaleSqr;
        Fixed64 t = denominator <= denominatorTolerance
            ? Fixed64.Zero
            : FixedMath.Clamp(-Vector2d.Dot(scaledA, ab) / denominator, Fixed64.Zero, Fixed64.One);

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
        return ClosestPlanarSimplexResult.FromPoint(a * (Fixed64.One - t) + b * t);
    }

    private static ClosestPlanarSimplexResult ReduceTriangle(
        Span<PlanarSupportPoint> simplex,
        ref int count,
        Fixed64 segmentDegeneracyToleranceSqr,
        Fixed64 areaTolerance,
        Fixed64 crossSignTolerance)
    {
        Vector2d a = simplex[0].Point;
        Vector2d b = simplex[1].Point;
        Vector2d c = simplex[2].Point;
        if (IsOriginInsideTriangle(a, b, c, areaTolerance, crossSignTolerance))
        {
            count = 3;
            return ClosestPlanarSimplexResult.Intersection;
        }

        bool hasBest = false;
        int bestFirst = 0;
        int bestSecond = 1;
        ClosestPlanarSimplexResult best = default;
        EvaluateTriangleEdge(simplex, 0, 1, segmentDegeneracyToleranceSqr, ref best, ref hasBest, ref bestFirst, ref bestSecond);
        EvaluateTriangleEdge(simplex, 1, 2, segmentDegeneracyToleranceSqr, ref best, ref hasBest, ref bestFirst, ref bestSecond);
        EvaluateTriangleEdge(simplex, 2, 0, segmentDegeneracyToleranceSqr, ref best, ref hasBest, ref bestFirst, ref bestSecond);

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
        Fixed64 segmentDegeneracyToleranceSqr,
        ref ClosestPlanarSimplexResult best,
        ref bool hasBest,
        ref int bestFirst,
        ref int bestSecond)
    {
        Span<PlanarSupportPoint> edge = stackalloc PlanarSupportPoint[2];
        edge[0] = simplex[first];
        edge[1] = simplex[second];
        int edgeCount = 2;
        ClosestPlanarSimplexResult candidate = ReduceSegment(edge, ref edgeCount, segmentDegeneracyToleranceSqr);
        if (hasBest && !IsCloser(
                candidate.DistanceSqr,
                candidate.Point,
                best.DistanceSqr,
                best.Point))
            return;

        best = candidate;
        hasBest = true;
        bestFirst = first;
        bestSecond = second;
    }

    internal static bool IsCloser(
        Fixed64 candidateDistanceSqr,
        Vector2d candidatePoint,
        Fixed64 bestDistanceSqr,
        Vector2d bestPoint)
    {
        if (candidateDistanceSqr != bestDistanceSqr || candidateDistanceSqr != Fixed64.MaxValue)
            return candidateDistanceSqr < bestDistanceSqr;

        return Vector2d.CompareMagnitudeSquared(candidatePoint, bestPoint) < 0;
    }

    private static bool IsOriginInsideTriangle(
        Vector2d a,
        Vector2d b,
        Vector2d c,
        Fixed64 workingAreaTolerance,
        Fixed64 workingCrossSignTolerance)
    {
        Span<Vector2d> scaled = stackalloc Vector2d[3];
        scaled[0] = a;
        scaled[1] = b;
        scaled[2] = c;
        Fixed64 productScale = GjkSimplexScale.ScaleForProducts(scaled);
        a = scaled[0];
        b = scaled[1];
        c = scaled[2];

        Fixed64 productScaleSqr = productScale * productScale;
        Fixed64 areaTolerance = workingAreaTolerance * productScaleSqr;
        if (Cross(a, b, c).Abs() <= areaTolerance)
            return false;

        Fixed64 ab = Cross(a, b, Vector2d.Zero);
        Fixed64 bc = Cross(b, c, Vector2d.Zero);
        Fixed64 ca = Cross(c, a, Vector2d.Zero);
        Fixed64 signTolerance = workingCrossSignTolerance * productScaleSqr;
        bool hasPositive = ab > signTolerance || bc > signTolerance || ca > signTolerance;
        bool hasNegative = ab < -signTolerance || bc < -signTolerance || ca < -signTolerance;
        return !(hasPositive && hasNegative);
    }

    private static bool ContainsSupportPoint(
        Span<PlanarSupportPoint> simplex,
        int count,
        Vector2d point)
    {
        for (int i = 0; i < count; i++)
        {
            Vector2d difference = simplex[i].Point - point;
            if (Vector2d.TryGetMagnitude(difference, out Fixed64 distance)
                && distance <= Fixed64.Epsilon)
                return true;
        }

        return false;
    }

    private readonly struct ProjectionTarget
    {
        private readonly LSCapsuleCollider? _capsule;
        private readonly LSCylinderCollider? _cylinder;
        private readonly LSConeCollider? _cone;
        private readonly Fixed64 _slabMinY;
        private readonly Fixed64 _slabMaxY;
        private readonly Vector2d _center;

        private ProjectionTarget(
            LSCapsuleCollider? capsule,
            LSCylinderCollider? cylinder,
            LSConeCollider? cone,
            Fixed64 slabMinY,
            Fixed64 slabMaxY,
            Vector2d center)
        {
            _capsule = capsule;
            _cylinder = cylinder;
            _cone = cone;
            _slabMinY = slabMinY;
            _slabMaxY = slabMaxY;
            _center = center;
        }

        public Vector2d Center => _center;

        public Vector2d BoundsMin => new(TargetBoundsMin.X, TargetBoundsMin.Z);

        public Vector2d BoundsMax => new(TargetBoundsMax.X, TargetBoundsMax.Z);

        private Vector3d TargetBoundsMin => _capsule?.BoundsMin ?? _cylinder?.BoundsMin ?? _cone!.BoundsMin;

        private Vector3d TargetBoundsMax => _capsule?.BoundsMax ?? _cylinder?.BoundsMax ?? _cone!.BoundsMax;

        public bool HasProjection
        {
            get
            {
                if (_capsule != null)
                    return CapsuleIntersectsSlab(_capsule, _slabMinY, _slabMaxY);

                if (_cylinder != null)
                    return CylinderIntersectsSlab(_cylinder, _slabMinY, _slabMaxY);

                return ConeIntersectsSlab(_cone!, _slabMinY, _slabMaxY);
            }
        }

        public static ProjectionTarget CreateCapsule(LSCapsuleCollider capsule, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(capsule, null, null, slabMinY, slabMaxY, new Vector2d(capsule.Center.X, capsule.Center.Z));

        public static ProjectionTarget CreateCylinder(LSCylinderCollider cylinder, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(null, cylinder, null, slabMinY, slabMaxY, new Vector2d(cylinder.Center.X, cylinder.Center.Z));

        public static ProjectionTarget CreateCone(LSConeCollider cone, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(null, null, cone, slabMinY, slabMaxY, new Vector2d(cone.Center.X, cone.Center.Z));

        public bool TrySupport(Vector2d direction, out Vector2d support)
        {
            Vector2d normal = direction.Normalized;
            if (_capsule != null)
                return TrySupportCapsuleProjection(_capsule, _slabMinY, _slabMaxY, normal, out support);

            if (_cylinder != null)
                return TrySupportCylinderProjection(_cylinder, _slabMinY, _slabMaxY, normal, out support);

            return TrySupportConeProjection(_cone!, _slabMinY, _slabMaxY, normal, out support);
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
        Fixed64 excessMagnitude = planarSlope.Abs() * capsule.ScaledRadius / FixedMath.Sqrt(dy * dy + planarSlope * planarSlope);
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

    private static bool TrySupportConeProjection(
        LSConeCollider cone,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        out Vector2d support)
    {
        bool found = false;
        Vector2d best = default;

        TryKeepConePoint(cone.WorldApex, slabMinY, slabMaxY, direction, ref found, ref best);
        if (TrySupportCylinderDiskInBand(cone.WorldBaseCenter, cone.Axis, cone.ScaledRadius, slabMinY, slabMaxY, direction, out Vector2d baseSupport))
            KeepBestSupport(baseSupport, direction, ref found, ref best);

        Vector3d wholeSupport = ConvexColliderSupport.Support(cone, new Vector3d(direction.X, Fixed64.Zero, direction.Y));
        TryKeepConePoint(wholeSupport, slabMinY, slabMaxY, direction, ref found, ref best);

        TryKeepVerticalConePlaneSupport(cone, slabMinY, direction, ref found, ref best);
        TryKeepVerticalConePlaneSupport(cone, slabMaxY, direction, ref found, ref best);

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
        Fixed64 tangentProjection = Vector3d.Dot(direction3D, tangent.Normalized).Abs();
        if (tangentProjection <= Fixed64.Epsilon)
            return;

        Fixed64 k = planarSlope - linearBoundarySlope * dy;
        Fixed64 m = -k * verticalCapacitySqr / (tangentProjection * dy);
        Fixed64 mSqr = m * m;
        Fixed64 qSqr = mSqr * cylinder.ScaledRadiusSqr * verticalCapacitySqr / (verticalCapacitySqr + mSqr);
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
            radial = radialDirection / radialMagnitude * radius;
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
        if (remainingSqr <= Fixed64.Epsilon)
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

    private static bool ConeIntersectsSlab(LSConeCollider cone, Fixed64 slabMinY, Fixed64 slabMaxY) =>
        cone.BoundsMax.Y >= slabMinY && cone.BoundsMin.Y <= slabMaxY;

    private static void TryKeepConePoint(
        Vector3d point,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        Vector2d direction,
        ref bool found,
        ref Vector2d best)
    {
        if (!IsPointYInsideSlab(point.Y, slabMinY, slabMaxY))
            return;

        KeepBestSupport(new Vector2d(point.X, point.Z), direction, ref found, ref best);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsConeAxisVertical(LSConeCollider cone) =>
        cone.Axis.X.Abs() <= Fixed64.Epsilon && cone.Axis.Z.Abs() <= Fixed64.Epsilon;

    private static void TryKeepVerticalConePlaneSupport(
        LSConeCollider cone,
        Fixed64 planeY,
        Vector2d direction,
        ref bool found,
        ref Vector2d best)
    {
        Vector3d local = cone.Rotation.Inverse() * (new Vector3d(cone.Center.X, planeY, cone.Center.Z) - cone.Center);
        Fixed64 radius = cone.RadiusAtLocalY(local.Y);
        Vector3d localDirection = cone.Rotation.Inverse() * new Vector3d(direction.X, Fixed64.Zero, direction.Y);
        Vector3d localRadial = new(localDirection.X, Fixed64.Zero, localDirection.Z);
        Fixed64 radialMagnitude = localRadial.Magnitude;
        localRadial = radialMagnitude > Fixed64.Epsilon
            ? localRadial / radialMagnitude
            : Vector3d.Right;

        Vector3d localPoint = new(localRadial.X * radius, local.Y, localRadial.Z * radius);
        Vector3d worldPoint = cone.Center + cone.Rotation * localPoint;
        KeepBestSupport(new Vector2d(worldPoint.X, worldPoint.Z), direction, ref found, ref best);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointYInsideSlab(Fixed64 y, Fixed64 slabMinY, Fixed64 slabMaxY) =>
        y >= slabMinY && y <= slabMaxY;

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
            int projectionComparison = Vector2d.CompareProjection(candidate, best, direction);
            if (projectionComparison < 0)
                return;

            if (projectionComparison == 0 && ComesAfter(candidate, best))
                return;
        }

        best = candidate;
        found = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ComesAfter(Vector2d first, Vector2d second) =>
        first.X > second.X || (first.X == second.X && first.Y > second.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Cross(Vector2d origin, Vector2d first, Vector2d second) =>
        (first.X - origin.X) * (second.Y - origin.Y) - (first.Y - origin.Y) * (second.X - origin.X);

}
