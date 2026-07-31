//=======================================================================
// FiniteSlabProjectionSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
        Vector2d end,
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
            end,
            direction,
            length,
            radius,
            target,
            out distance,
            maxConservativeAdvancementIterations);
    }

    public static bool TrySweepCircleAgainstCylinder(
        Vector2d start,
        Vector2d end,
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
            end,
            direction,
            length,
            radius,
            target,
            out distance,
            maxConservativeAdvancementIterations);
    }

    public static bool TrySweepCircleAgainstCone(
        Vector2d start,
        Vector2d end,
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
            end,
            direction,
            length,
            radius,
            target,
            out distance,
            maxConservativeAdvancementIterations);
    }

    private static bool TrySweepCircle(
        Vector2d start,
        Vector2d end,
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
            || maxConservativeAdvancementIterations <= 0
            || !target.TrySupport(Vector2d.Right, out Vector2d rightSupport))
            return false;

        if (target.TryGetPlanarCircle(rightSupport, out Vector2d targetCenter, out Fixed64 targetRadius))
        {
            return new FixedSegment2d(start, end)
                .TryGetCircleIntersectionDistanceInterval(
                    new FixedBoundCircle(targetCenter, targetRadius),
                    radius,
                    length,
                    out distance,
                    out _,
                    out _,
                    out _);
        }

        Fixed64 travelDistance = Fixed64.Zero;
        for (int i = 0; i < maxConservativeAdvancementIterations; i++)
        {
            Vector2d point = GetSweepPoint(start, direction, travelDistance);
            if (!TryComputeDistance(point, radius, target, out PlanarGjkResult result))
            {
                return false;
            }

            if (result.Distance <= SweepContactTolerance)
            {
                distance = travelDistance;
                return true;
            }

            Vector2d normal = result.Normal;
            Fixed64 closingSpeed = -Vector2d.Dot(direction, normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 remainingDistance = length - travelDistance;
            bool reachedEndpoint = !Fixed64.TryMultiplyDivide(
                    result.Distance,
                    Fixed64.One,
                    closingSpeed,
                    out Fixed64 stepDistance)
                || stepDistance > remainingDistance;
            if (reachedEndpoint)
            {
                if (!TryComputeDistance(end, radius, target, out PlanarGjkResult endpoint))
                {
                    return false;
                }

                if (endpoint.Distance <= SweepContactTolerance)
                {
                    distance = length;
                    return true;
                }

                return false;
            }

            travelDistance += stepDistance;
        }

        return false;
    }

    private static bool TryComputeDistance(
        Vector2d point,
        Fixed64 expansionRadius,
        ProjectionTarget target,
        out PlanarGjkResult result)
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
            if (!TryCreateSupportPoint(
                    point,
                    expansionRadius,
                    target,
                    direction,
                    workingShift,
                    out PlanarSupportPoint support))
            {
                result = default;
                return false;
            }

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
            {
                result = PlanarGjkResult.Intersection;
                return true;
            }

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
        result = new PlanarGjkResult(distance, normal);
        return true;
    }

    private static bool TryCreateSupportPoint(
        Vector2d point,
        Fixed64 expansionRadius,
        ProjectionTarget target,
        Vector2d direction,
        int workingShift,
        out PlanarSupportPoint support)
    {
        Vector2d supportDirection = direction.Normalized;
        Vector2d targetDirection = -supportDirection;
        if (!target.TrySupport(targetDirection, out Vector2d targetSupport))
        {
            support = default;
            return false;
        }

        Vector2d expansion = targetDirection * expansionRadius;
        support = new PlanarSupportPoint(
            GjkSimplexScale.CreateWorkingDifference(point, targetSupport, expansion, workingShift));
        return true;
    }

    // The authored endpoint bounds every monotonic intermediate component.
    private static Vector2d GetSweepPoint(
        Vector2d start,
        Vector2d direction,
        Fixed64 distance) =>
        new(
            Fixed64.MultiplyAdd(direction.X, distance, start.X),
            Fixed64.MultiplyAdd(direction.Y, distance, start.Y));

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
            if (Vector2d.TryGetDistance(simplex[i].Point, point, out Fixed64 distance)
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
        private readonly FixedRange _slabY;
        private readonly Vector2d _center;
        private readonly Vector3d _axis;

        private ProjectionTarget(
            LSCapsuleCollider? capsule,
            LSCylinderCollider? cylinder,
            LSConeCollider? cone,
            Fixed64 slabMinY,
            Fixed64 slabMaxY,
            Vector2d center,
            Vector3d axis)
        {
            _capsule = capsule;
            _cylinder = cylinder;
            _cone = cone;
            _slabY = new FixedRange(slabMinY, slabMaxY);
            _center = center;
            _axis = axis;
        }

        public Vector2d Center => _center;

        public Vector2d BoundsMin => new(TargetBoundsMin.X, TargetBoundsMin.Z);

        public Vector2d BoundsMax => new(TargetBoundsMax.X, TargetBoundsMax.Z);

        private Vector3d TargetBoundsMin => _capsule?.BoundsMin ?? _cylinder?.BoundsMin ?? _cone!.BoundsMin;

        private Vector3d TargetBoundsMax => _capsule?.BoundsMax ?? _cylinder?.BoundsMax ?? _cone!.BoundsMax;

        public static ProjectionTarget CreateCapsule(LSCapsuleCollider capsule, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(
                capsule,
                null,
                null,
                slabMinY,
                slabMaxY,
                new Vector2d(capsule.Center.X, capsule.Center.Z),
                GetRigidUpAxis(capsule.Rotation));

        public static ProjectionTarget CreateCylinder(LSCylinderCollider cylinder, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(
                null,
                cylinder,
                null,
                slabMinY,
                slabMaxY,
                new Vector2d(cylinder.Center.X, cylinder.Center.Z),
                GetRigidUpAxis(cylinder.Rotation));

        public static ProjectionTarget CreateCone(LSConeCollider cone, Fixed64 slabMinY, Fixed64 slabMaxY) =>
            new(
                null,
                null,
                cone,
                slabMinY,
                slabMaxY,
                new Vector2d(cone.Center.X, cone.Center.Z),
                GetRigidUpAxis(cone.Rotation));

        public bool TrySupport(Vector2d direction, out Vector2d support)
        {
            Vector2d normal = direction.Normalized;
            if (_capsule != null)
            {
                return FixedSlabProjection.TryGetCapsuleSupport(
                    _capsule.Center,
                    _axis,
                    _capsule.AxisLength,
                    _capsule.ScaledRadius,
                    _slabY,
                    normal,
                    out support);
            }

            if (_cylinder != null)
            {
                return FixedSlabProjection.TryGetCylinderSupport(
                    _cylinder.Center,
                    _axis,
                    _cylinder.Height,
                    _cylinder.ScaledRadius,
                    _slabY,
                    normal,
                    out support);
            }

            return FixedSlabProjection.TryGetConeSupport(
                _cone!.Center,
                _axis,
                _cone.Height,
                _cone.ScaledRadius,
                _slabY,
                normal,
                out support);
        }

        public bool TryGetPlanarCircle(
            Vector2d rightSupport,
            out Vector2d center,
            out Fixed64 radius)
        {
            if ((_axis.X != Fixed64.Zero) | (_axis.Z != Fixed64.Zero))
            {
                center = default;
                radius = default;
                return false;
            }

            center = _center;
            radius = rightSupport.X - _center.X;
            return true;
        }

    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetRigidUpAxis(FixedQuaternion rotation) =>
        (rotation * Vector3d.Up).Normalized;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Cross(Vector2d origin, Vector2d first, Vector2d second) =>
        (first.X - origin.X) * (second.Y - origin.Y) - (first.Y - origin.Y) * (second.X - origin.X);

}
