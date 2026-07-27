//=======================================================================
// ConvexSweepQueryWorker.Gjk.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Queries;

internal sealed partial class ConvexSweepQueryWorker
{
    private GjkResult ComputeDistance(ConvexShape sourceShape, ConvexShape targetShape)
    {
        const int workingShift = 2;
        // Quartering both endpoints bounds every two-term Minkowski component
        // below half the scalar domain. Any convex simplex combination then
        // has a representable 3D magnitude without a lossy retry.
        Fixed64 workingScale = GjkSimplexScale.GetCoordinateScale(workingShift);
        Fixed64 workingDistanceTolerance = DistanceTolerance * workingScale;
        int simplexCount = 0;
        FixedPointAnchor sourceCenter = sourceShape.GetCenterAnchor();
        FixedPointAnchor targetCenter = targetShape.GetCenterAnchor();
        // The shift was selected from bounds containing both centers, so this
        // scaled difference is representable by construction.
        _ = targetCenter.TryGetScaledOffsetFrom(
            sourceCenter,
            workingScale,
            out Vector3d direction);
        if (direction == Vector3d.Zero)
        {
            if (sourceShape.ContainsCenter && targetShape.ContainsCenter)
            {
                return GjkResult.CreateIntersection(
                    sourceCenter,
                    targetCenter);
            }

            direction = Vector3d.Right;
        }

        bool hasPreviousDistance = false;
        Fixed64 previousDistance = Fixed64.Zero;
        ClosestSimplexResult closest = default;
        Fixed64 workingDistance = Fixed64.MaxValue;

        for (int i = 0; i < MaxGjkIterations; i++)
        {
            SupportPoint support = CreateSupportPoint(
                sourceShape,
                targetShape,
                direction,
                workingScale);
            if (ContainsSupportPoint(_simplex, simplexCount, support.Point))
                break;

            ClosestSimplexResult previousClosest = closest;
            _simplex[simplexCount++] = support;
            closest = SolveClosestSimplex(_simplex, ref simplexCount, workingScale);
            _ = Vector3d.TryGetMagnitude(closest.Point, out workingDistance);
            if (closest.Intersects)
            {
                // Tetrahedron entry follows a valid one-to-three point simplex.
                // Preserve that same-pose closest pair as the deterministic
                // surface witness for the intersection result.
                return GjkResult.CreateIntersection(previousClosest.PointA, previousClosest.PointB);
            }

            if (workingDistance <= workingDistanceTolerance)
                return GjkResult.CreateIntersection(closest.PointA, closest.PointB);

            if (hasPreviousDistance
                && previousDistance - workingDistance <= Fixed64.Epsilon)
            {
                break;
            }

            hasPreviousDistance = true;
            previousDistance = workingDistance;
            direction = -closest.Point;
        }

        Fixed64 distance = GjkSimplexScale.RestoreDistance(workingDistance, workingShift);
        Vector3d normal = closest.Point.Normalized;
        return new GjkResult(false, distance, closest.PointA, closest.PointB, normal);
    }

    private static SupportPoint CreateSupportPoint(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        Vector3d direction,
        Fixed64 workingScale)
    {
        Vector3d supportDirection = direction.Normalized;
        FixedPointAnchor pointA =
            sourceShape.GetSupportAnchor(supportDirection);
        FixedPointAnchor pointB =
            targetShape.GetSupportAnchor(-supportDirection);
        // The same bounds-selected shift covers every enclosed support pair.
        _ = pointA.TryGetScaledOffsetFrom(
            pointB,
            workingScale,
            out Vector3d difference);
        return new SupportPoint(pointA, pointB, difference);
    }

    private static ClosestSimplexResult SolveClosestSimplex(
        SupportPoint[] simplex,
        ref int count,
        Fixed64 workingScale)
    {
        if (count == 1)
            return ClosestSimplexResult.FromWeights(simplex, 1, Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        if (count == 2)
            return ReduceSegment(simplex, ref count, workingScale);

        if (count == 3)
            return ReduceTriangle(simplex, ref count);

        return ReduceTetrahedron(simplex, ref count, workingScale);
    }
}
