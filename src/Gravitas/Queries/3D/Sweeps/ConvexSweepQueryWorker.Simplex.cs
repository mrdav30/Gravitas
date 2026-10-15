//=======================================================================
// ConvexSweepQueryWorker.Simplex.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;

namespace Gravitas.Queries;

internal sealed partial class ConvexSweepQueryWorker
{
    private static ClosestSimplexResult ReduceSegment(
        SupportPoint[] simplex,
        ref int count,
        Fixed64 workingScale)
    {
        SupportPoint a = simplex[0];
        SupportPoint b = simplex[1];
        Span<Vector3d> scaled = stackalloc Vector3d[2];
        scaled[0] = a.Point;
        scaled[1] = b.Point;
        Fixed64 productScale = GjkSimplexScale.ScaleForProducts(scaled);
        Vector3d scaledA = scaled[0];
        Vector3d ab = scaled[1] - scaledA;
        Fixed64 denominator = ab.MagnitudeSquared;
        Fixed64 workingScaleSqr = workingScale * workingScale;
        Fixed64 productScaleSqr = productScale * productScale;
        Fixed64 denominatorTolerance = Fixed64.Epsilon * workingScaleSqr * productScaleSqr;
        Fixed64 t = denominator <= denominatorTolerance
            ? Fixed64.Zero
            : FixedMath.Clamp(-Vector3d.Dot(scaledA, ab) / denominator, Fixed64.Zero, Fixed64.One);

        if (t <= Fixed64.Epsilon)
        {
            simplex[0] = a;
            count = 1;
            return ClosestSimplexResult.FromWeights(simplex, count, Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        }

        if (t >= Fixed64.One - Fixed64.Epsilon)
        {
            simplex[0] = b;
            count = 1;
            return ClosestSimplexResult.FromWeights(simplex, count, Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        }

        simplex[0] = a;
        simplex[1] = b;
        count = 2;
        return ClosestSimplexResult.FromWeights(simplex, count, Fixed64.One - t, t, Fixed64.Zero, Fixed64.Zero);
    }

    private static ClosestSimplexResult ReduceTriangle(SupportPoint[] simplex, ref int count)
    {
        TriangleWeights weights = ClosestPointOnTriangleToOrigin(simplex[0].Point, simplex[1].Point, simplex[2].Point);
        return ReduceByWeights(simplex, ref count, weights.A, weights.B, weights.C, Fixed64.Zero);
    }

    private static ClosestSimplexResult ReduceTetrahedron(
        SupportPoint[] simplex,
        ref int count)
    {
        if (IsOriginInsideTetrahedron(
                simplex[0].Point,
                simplex[1].Point,
                simplex[2].Point,
                simplex[3].Point))
        {
            count = 4;
            return ClosestSimplexResult.Intersection;
        }

        Vector3d bestPoint = default;
        bool hasBest = false;
        Span<int> bestIndices = stackalloc int[3];
        Span<Fixed64> bestWeights = stackalloc Fixed64[3];
        Span<Fixed64> weights = stackalloc Fixed64[3];

        EvaluateFace(simplex, 0, 1, 2, ref bestPoint, ref hasBest, bestIndices, bestWeights, weights);
        EvaluateFace(simplex, 0, 3, 1, ref bestPoint, ref hasBest, bestIndices, bestWeights, weights);
        EvaluateFace(simplex, 0, 2, 3, ref bestPoint, ref hasBest, bestIndices, bestWeights, weights);
        EvaluateFace(simplex, 1, 3, 2, ref bestPoint, ref hasBest, bestIndices, bestWeights, weights);

        SupportPoint first = simplex[bestIndices[0]];
        SupportPoint second = simplex[bestIndices[1]];
        SupportPoint third = simplex[bestIndices[2]];
        simplex[0] = first;
        simplex[1] = second;
        simplex[2] = third;
        count = 3;
        return ClosestSimplexResult.FromWeights(simplex, count, bestWeights[0], bestWeights[1], bestWeights[2], Fixed64.Zero);
    }

    private static void EvaluateFace(
        SupportPoint[] simplex,
        int first,
        int second,
        int third,
        ref Vector3d bestPoint,
        ref bool hasBest,
        Span<int> bestIndices,
        Span<Fixed64> bestWeights,
        Span<Fixed64> weights)
    {
        TriangleWeights triangleWeights = ClosestPointOnTriangleToOrigin(
            simplex[first].Point,
            simplex[second].Point,
            simplex[third].Point);
        weights[0] = triangleWeights.A;
        weights[1] = triangleWeights.B;
        weights[2] = triangleWeights.C;
        // Rank only the Minkowski point; construct both witnesses once for the winner.
        Span<Vector3d> points = stackalloc Vector3d[3];
        points[0] = simplex[first].Point;
        points[1] = simplex[second].Point;
        points[2] = simplex[third].Point;
        _ = Vector3d.TryGetWeightedAverage(points, weights, out Vector3d candidatePoint);
        if (hasBest && Vector3d.CompareMagnitudeSquared(candidatePoint, bestPoint) >= 0)
            return;

        bestPoint = candidatePoint;
        hasBest = true;
        bestIndices[0] = first;
        bestIndices[1] = second;
        bestIndices[2] = third;
        bestWeights[0] = weights[0];
        bestWeights[1] = weights[1];
        bestWeights[2] = weights[2];
    }

    private static ClosestSimplexResult ReduceByWeights(
        SupportPoint[] simplex,
        ref int count,
        Fixed64 firstWeight,
        Fixed64 secondWeight,
        Fixed64 thirdWeight,
        Fixed64 fourthWeight)
    {
        Span<SupportPoint> reduced = stackalloc SupportPoint[4];
        Span<Fixed64> weights = stackalloc Fixed64[4];
        int reducedCount = 0;
        AddWeightedPoint(simplex[0], firstWeight, reduced, weights, ref reducedCount);
        AddWeightedPoint(simplex[1], secondWeight, reduced, weights, ref reducedCount);
        AddWeightedPoint(simplex[2], thirdWeight, reduced, weights, ref reducedCount);
        AddWeightedPoint(simplex[3], fourthWeight, reduced, weights, ref reducedCount);

        for (int i = 0; i < reducedCount; i++)
            simplex[i] = reduced[i];

        count = reducedCount;
        return ClosestSimplexResult.FromSpanWeights(reduced, weights, reducedCount);
    }

    private static void AddWeightedPoint(
        SupportPoint point,
        Fixed64 weight,
        Span<SupportPoint> reduced,
        Span<Fixed64> weights,
        ref int count)
    {
        if (weight <= Fixed64.Epsilon)
            return;

        reduced[count] = point;
        weights[count] = weight;
        count++;
    }

    /// <summary>
    /// Returns barycentric weights for the point on a triangle closest to the origin.
    /// </summary>
    internal static TriangleWeights ClosestPointOnTriangleToOrigin(Vector3d a, Vector3d b, Vector3d c)
    {
        Span<Vector3d> scaled = stackalloc Vector3d[3];
        scaled[0] = a;
        scaled[1] = b;
        scaled[2] = c;
        GjkSimplexScale.ScaleForProducts(scaled);
        a = scaled[0];
        b = scaled[1];
        c = scaled[2];

        var triangle = new FixedTriangle(a, b, c);
        if (!triangle.TryGetClosestPointBarycentrics(Vector3d.Zero,
                out Signed320 numeratorB, out Signed320 numeratorC,
                out Signed320 denominator, out _))
        {
            return ClosestPointOnDegenerateTriangleToOrigin(a, b, c);
        }

        // Retain exact feature weights rather than reprojecting a rounded point.
        _ = Fixed64.TryGetUnitIntervalRatio(numeratorB, denominator, out Fixed64 weightB);
        _ = Fixed64.TryGetUnitIntervalRatio(numeratorC, denominator, out Fixed64 weightC);
        weightB = FixedMath.Clamp(
            weightB,
            Fixed64.Zero,
            Fixed64.One);
        weightC = FixedMath.Clamp(
            weightC,
            Fixed64.Zero,
            Fixed64.One - weightB);
        return new TriangleWeights(
            Fixed64.One - weightB - weightC,
            weightB,
            weightC);
    }

    private static TriangleWeights ClosestPointOnDegenerateTriangleToOrigin(
        Vector3d first,
        Vector3d second,
        Vector3d third)
    {
        GetClosestSegmentWeights(
            first,
            second,
            out Fixed64 firstWeight,
            out Fixed64 secondWeight,
            out Vector3d bestPoint);
        TriangleWeights best = new(
            firstWeight,
            secondWeight,
            Fixed64.Zero);

        GetClosestSegmentWeights(
            first,
            third,
            out firstWeight,
            out Fixed64 thirdWeight,
            out Vector3d candidatePoint);
        if (Vector3d.CompareMagnitudeSquared(
                candidatePoint,
                bestPoint) < 0)
        {
            bestPoint = candidatePoint;
            best = new TriangleWeights(
                firstWeight,
                Fixed64.Zero,
                thirdWeight);
        }

        GetClosestSegmentWeights(
            second,
            third,
            out secondWeight,
            out thirdWeight,
            out candidatePoint);
        if (Vector3d.CompareMagnitudeSquared(
                candidatePoint,
                bestPoint) < 0)
        {
            best = new TriangleWeights(
                Fixed64.Zero,
                secondWeight,
                thirdWeight);
        }

        return best;
    }

    private static void GetClosestSegmentWeights(
        Vector3d start,
        Vector3d end,
        out Fixed64 startWeight,
        out Fixed64 endWeight,
        out Vector3d point)
    {
        Vector3d delta = end - start;
        Fixed64 denominator = Vector3d.Dot(delta, delta);
        Fixed64 numerator = -Vector3d.Dot(start, delta);
        if (denominator == Fixed64.Zero || numerator <= Fixed64.Zero)
        {
            startWeight = Fixed64.One;
            endWeight = Fixed64.Zero;
            point = start;
            return;
        }
        if (numerator >= denominator)
        {
            startWeight = Fixed64.Zero;
            endWeight = Fixed64.One;
            point = end;
            return;
        }

        endWeight = numerator / denominator;
        startWeight = Fixed64.One - endWeight;
        point = start * startWeight + end * endWeight;
    }

    /// <summary>Tests inclusive origin containment in a nondegenerate GJK tetrahedron.</summary>
    internal static bool IsOriginInsideTetrahedron(
        Vector3d a,
        Vector3d b,
        Vector3d c,
        Vector3d d)
    {
        // These consistently oriented face determinants are barycentric volume
        // numerators. Equal nonzero signs certify containment and nonzero volume;
        // rounded plane-side products can instead erase a genuine separation.
        int abc = WideGeometry.GetTripleProductSign(
            a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z);
        int adb = WideGeometry.GetTripleProductSign(
            a.X, a.Y, a.Z, d.X, d.Y, d.Z, b.X, b.Y, b.Z);
        int acd = WideGeometry.GetTripleProductSign(
            a.X, a.Y, a.Z, c.X, c.Y, c.Z, d.X, d.Y, d.Z);
        int bdc = WideGeometry.GetTripleProductSign(
            b.X, b.Y, b.Z, d.X, d.Y, d.Z, c.X, c.Y, c.Z);
        bool hasPositive = abc > 0 || adb > 0 || acd > 0 || bdc > 0;
        bool hasNegative = abc < 0 || adb < 0 || acd < 0 || bdc < 0;
        // Coplanar simplexes have conflicting signs or four zeros; retain their
        // closest-face reduction rather than declaring a volume intersection.
        return hasPositive != hasNegative;
    }

    private static bool ContainsSupportPoint(
        SupportPoint[] simplex,
        int count,
        Vector3d point)
    {
        for (int i = 0; i < count; i++)
        {
            Vector3d difference = simplex[i].Point - point;
            if (Vector3d.TryGetMagnitude(difference, out Fixed64 distance)
                && distance <= Fixed64.Epsilon)
                return true;
        }

        return false;
    }
}
