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
/// <content>Owns conservative advancement and projected support simplex reduction for curved targets.</content>
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
            || !target.TrySupport(Vector2d.Right, out Vector2d rightSupport, out bool translated))
            return false;

        if (target.TryGetPlanarCircle(rightSupport, translated, out Vector2d targetCenter, out Fixed64 targetRadius))
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
        Vector2d searchDirection = Vector2d.Zero;
        var segment = new FixedSegment2d(start, end);
        Vector2d displacement = end - start;
        for (int i = 0; i < maxConservativeAdvancementIterations; i++)
        {
            Vector2d point = segment.GetPointAtDistance(travelDistance, length);
            PlanarGjkResult result = ComputeDistance(point, radius, target, searchDirection);

            if (result.Distance <= SweepContactTolerance)
            {
                distance = travelDistance;
                return true;
            }

            Vector2d normal = result.Normal;
            // Reuse the resolved separating direction at the next nearby pose.
            searchDirection = -normal;
            // Keep the authored chord over its distance denominator. A rounded
            // unit ray can enter a shape that the original segment only grazes.
            Signed320 closing = WideArithmetic.SubtractSigned320(default,
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(Signed192.Raw(displacement.X), Signed192.Raw(normal.X)),
                    WideArithmetic.MultiplySigned192(Signed192.Raw(displacement.Y), Signed192.Raw(normal.Y))));
            if (WideArithmetic.SubtractSigned320(closing,
                    WideArithmetic.MultiplySigned192(Signed192.Raw(Fixed64.Epsilon), Signed192.Raw(length))).Sign <= 0)
                return false;

            Fixed64 remainingDistance = length - travelDistance;
            bool reachedEndpoint = !Fixed64.TryGetSignedRawRatio(
                    Signed832.ExtendValue(Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
                        Signed192.Raw(result.Distance), Signed192.Raw(length)))),
                    Signed832.ExtendValue(Signed576.ExtendValue(closing)),
                    32,
                    out Fixed64 stepDistance)
                || stepDistance > remainingDistance;
            if (reachedEndpoint)
            {
                PlanarGjkResult endpoint = ComputeDistance(end, radius, target, searchDirection);

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

    private static PlanarGjkResult ComputeDistance(
        Vector2d point,
        Fixed64 expansionRadius,
        ProjectionTarget target,
        Vector2d initialDirection)
    {
        Span<PlanarSupportPoint> simplex = stackalloc PlanarSupportPoint[3];
        int workingShift = target.SelectWorkingShift(point, expansionRadius);
        Fixed64 workingScale = GjkSimplexScale.GetCoordinateScale(workingShift);
        Fixed64 workingDistanceTolerance = DistanceTolerance * workingScale;
        int simplexCount = 0;
        Vector2d direction = initialDirection == Vector2d.Zero
            ? GjkSimplexScale.CreateWorkingDifference(target.Center, point, workingShift)
            : initialDirection;
        if (direction == Vector2d.Zero)
            direction = Vector2d.Right;

        bool hasPreviousDistance = false;
        Fixed64 previousDistance = Fixed64.Zero;
        ClosestPlanarSimplexResult closest = default;
        bool distanceIsRepresentable = false;
        Fixed64 workingDistance = Fixed64.MaxValue;

        for (int i = 0; i < MaxGjkIterations; i++)
        {
            PlanarSupportPoint support = CreateSupportPoint(
                point, expansionRadius, target, direction, workingShift);

            if (ContainsSupportPoint(simplex, simplexCount, support.Point))
                break;

            simplex[simplexCount++] = support;
            closest = SolveClosestSimplex(simplex, ref simplexCount);
            distanceIsRepresentable = Vector2d.TryGetMagnitude(closest.Point, out workingDistance);
            if (closest.Intersects
                || (distanceIsRepresentable && workingDistance <= workingDistanceTolerance))
            {
                return PlanarGjkResult.Intersection;
            }

            if (hasPreviousDistance
                && distanceIsRepresentable
                && previousDistance - workingDistance <= Fixed64.Epsilon)
            {
                break;
            }

            hasPreviousDistance = distanceIsRepresentable;
            previousDistance = workingDistance;
            direction = closest.SearchDirection;
        }

        Fixed64 distance = GjkSimplexScale.RestoreDistance(workingDistance, workingShift);
        Vector2d normal = -WideNormalization.GetNormalized(closest.SearchDirection);
        return new PlanarGjkResult(distance, normal);
    }

    private static PlanarSupportPoint CreateSupportPoint(
        Vector2d point,
        Fixed64 expansionRadius,
        ProjectionTarget target,
        Vector2d direction,
        int workingShift)
    {
        Vector2d supportDirection = WideNormalization.GetNormalized(direction);
        Vector2d targetDirection = -supportDirection;
        // The initial right support established a nonempty slab intersection;
        // admission and the chosen chart bound every later directional support.
        _ = target.TrySupport(targetDirection, out Vector2d targetSupport, out bool translated);

        Vector2d expansion = targetDirection * expansionRadius;
        return new PlanarSupportPoint(
            translated
                ? GjkSimplexScale.CreateWorkingDifference(point, target.Center, targetSupport, expansion, workingShift)
                : GjkSimplexScale.CreateWorkingDifference(point, targetSupport, expansion, workingShift));
    }

    private static ClosestPlanarSimplexResult SolveClosestSimplex(
        Span<PlanarSupportPoint> simplex,
        ref int count)
    {
        if (count == 1)
            return ClosestPlanarSimplexResult.FromPoint(simplex[0].Point);

        if (count == 2)
            return ReduceSegment(simplex, ref count);

        return ReduceTriangle(simplex, ref count);
    }

    private static ClosestPlanarSimplexResult ReduceSegment(
        Span<PlanarSupportPoint> simplex,
        ref int count)
    {
        Vector2d a = simplex[0].Point;
        Vector2d b = simplex[1].Point;
        Fixed64 t = Vector2d.GetClosestPointOnLineSegmentParameter(Vector2d.Zero, a, b);

        if (t == Fixed64.Zero)
        {
            count = 1;
            return ClosestPlanarSimplexResult.FromPoint(a);
        }

        if (t == Fixed64.One)
        {
            simplex[0] = simplex[1];
            count = 1;
            return ClosestPlanarSimplexResult.FromPoint(b);
        }

        count = 2;
        return ClosestPlanarSimplexResult.FromSegment(new Vector2d(
            FixedMath.Lerp(a.X, b.X, t), FixedMath.Lerp(a.Y, b.Y, t)), a, b);
    }

    private static ClosestPlanarSimplexResult ReduceTriangle(
        Span<PlanarSupportPoint> simplex,
        ref int count)
    {
        Vector2d a = simplex[0].Point;
        Vector2d b = simplex[1].Point;
        Vector2d c = simplex[2].Point;
        int winding = Vector2d.OrientationSign(a, b, c);
        int ab = Vector2d.OrientationSign(a, b, Vector2d.Zero);
        int bc = Vector2d.OrientationSign(b, c, Vector2d.Zero);
        int ca = Vector2d.OrientationSign(c, a, Vector2d.Zero);
        if (winding != 0 && ab != -winding && bc != -winding && ca != -winding)
        {
            count = 3;
            return ClosestPlanarSimplexResult.Intersection;
        }

        bool hasBest = false;
        int bestFirst = 0;
        int bestSecond = 1;
        ClosestPlanarSimplexResult best = default;
        // Only origin-facing edges can own a closest feature of a proper triangle.
        // Collinear simplexes retain every edge because they have no interior side.
        if (winding == 0 || ab != winding)
            EvaluateTriangleEdge(simplex, 0, 1, ref best, ref hasBest, ref bestFirst, ref bestSecond);
        if (winding == 0 || bc != winding)
            EvaluateTriangleEdge(simplex, 1, 2, ref best, ref hasBest, ref bestFirst, ref bestSecond);
        if (winding == 0 || ca != winding)
            EvaluateTriangleEdge(simplex, 2, 0, ref best, ref hasBest, ref bestFirst, ref bestSecond);

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
        ref bool hasBest,
        ref int bestFirst,
        ref int bestSecond)
    {
        Span<PlanarSupportPoint> edge = stackalloc PlanarSupportPoint[2];
        edge[0] = simplex[first];
        edge[1] = simplex[second];
        int edgeCount = 2;
        ClosestPlanarSimplexResult candidate = ReduceSegment(edge, ref edgeCount);
        if (hasBest && Vector2d.CompareMagnitudeSquared(candidate.Point, best.Point) >= 0)
            return;

        best = candidate;
        hasBest = true;
        bestFirst = first;
        bestSecond = second;
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
        private readonly bool _useTranslatedSupport;
        private readonly Fixed64 _supportExtent;

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
            Fixed64 length = capsule?.AxisLength ?? cylinder?.Height ?? cone!.Height;
            Fixed64 radius = capsule?.ScaledRadius ?? cylinder?.ScaledRadius ?? cone!.ScaledRadius;
            Fixed64 halfLength = Fixed64.FromRaw((length.m_rawValue >> 1) + (length.m_rawValue & 1));
            Fixed64 axialComponent = FixedMath.Max(axis.X.Abs(), axis.Z.Abs());
            // Support uses a rounded axis, so exact rigid bounds alone cannot select the frame.
            // The admitted local dimensions bound this envelope well below the scalar limit.
            _ = Fixed64.TryMultiplyAdd(halfLength, axialComponent, radius, out Fixed64 extent);
            _supportExtent = extent + Fixed64.MinIncrement;
            Vector2d planarExtent = new(_supportExtent, _supportExtent);
            _useTranslatedSupport = !Vector2d.TryAdd(center, planarExtent, out _)
                    || !Vector2d.TrySubtract(center, planarExtent, out _);
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

        public int SelectWorkingShift(Vector2d point, Fixed64 radius)
        {
            if (!_useTranslatedSupport)
                return GjkSimplexScale.SelectThreeTermShift(point, BoundsMin, BoundsMax, radius);
            Vector2d extent = new(_supportExtent, _supportExtent);
            return GjkSimplexScale.SelectFourTermShift(point, _center, -extent, extent, radius);
        }

        public bool TrySupport(Vector2d unitDirection, out Vector2d support, out bool translated)
        {
            // Target support and circle expansion consume the same normalized direction.
            Vector3d center = _capsule?.Center ?? _cylinder?.Center ?? _cone!.Center;
            // Select the frame once; retrying every unavailable support repeats the expensive solve.
            translated = _useTranslatedSupport;
            if (translated)
                center = new Vector3d(Fixed64.Zero, center.Y, Fixed64.Zero);

            if (_capsule != null)
            {
                return FixedSlabProjection.TryGetCapsuleSupport(
                    center,
                    _axis,
                    _capsule.AxisLength,
                    _capsule.ScaledRadius,
                    _slabY,
                    unitDirection,
                    out support);
            }

            if (_cylinder != null)
            {
                return FixedSlabProjection.TryGetCylinderSupport(
                    center,
                    _axis,
                    _cylinder.Height,
                    _cylinder.ScaledRadius,
                    _slabY,
                    unitDirection,
                    out support);
            }

            return FixedSlabProjection.TryGetConeSupport(
                center,
                _axis,
                _cone!.Height,
                _cone.ScaledRadius,
                _slabY,
                unitDirection,
                out support);
        }

        public bool TryGetPlanarCircle(
            Vector2d rightSupport,
            bool translated,
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
            radius = translated ? rightSupport.X : rightSupport.X - _center.X;
            return true;
        }

    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetRigidUpAxis(FixedQuaternion rotation) =>
        (rotation * Vector3d.Up).Normalized;

}
