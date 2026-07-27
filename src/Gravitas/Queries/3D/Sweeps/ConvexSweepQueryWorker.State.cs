//=======================================================================
// ConvexSweepQueryWorker.State.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal sealed partial class ConvexSweepQueryWorker
{
    private readonly struct SupportPoint
    {
        public SupportPoint(
            FixedPointAnchor pointA,
            FixedPointAnchor pointB,
            Vector3d point)
        {
            PointA = pointA;
            PointB = pointB;
            Point = point;
        }

        public FixedPointAnchor PointA { get; }

        public FixedPointAnchor PointB { get; }

        public Vector3d Point { get; }
    }

    private readonly struct SweepTriangleCandidate
    {
        public SweepTriangleCandidate(
            int triangleIndex,
            Fixed64 lowerBoundNumerator)
        {
            TriangleIndex = triangleIndex;
            LowerBoundNumerator = lowerBoundNumerator;
        }

        public int TriangleIndex { get; }

        public Fixed64 LowerBoundNumerator { get; }
    }

    private sealed class SweepTriangleCandidateComparer : IComparer<SweepTriangleCandidate>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Compare(SweepTriangleCandidate left, SweepTriangleCandidate right)
        {
            int lowerBoundCompare =
                left.LowerBoundNumerator.CompareTo(
                    right.LowerBoundNumerator);
            return lowerBoundCompare != 0
                ? lowerBoundCompare
                : left.TriangleIndex.CompareTo(right.TriangleIndex);
        }
    }

    private readonly struct GjkResult
    {
        public GjkResult(
            bool intersects,
            Fixed64 distance,
            FixedPointAnchor pointA,
            FixedPointAnchor pointB,
            Vector3d normal)
        {
            Intersects = intersects;
            Distance = distance;
            PointA = pointA;
            PointB = pointB;
            Normal = normal;
        }

        public bool Intersects { get; }

        public Fixed64 Distance { get; }

        public FixedPointAnchor PointA { get; }

        public FixedPointAnchor PointB { get; }

        public Vector3d Normal { get; }

        public static GjkResult CreateIntersection(
            FixedPointAnchor pointA,
            FixedPointAnchor pointB) =>
            new(true, Fixed64.Zero, pointA, pointB, Vector3d.Zero);
    }

    private readonly struct ClosestSimplexResult
    {
        public ClosestSimplexResult(
            bool intersects,
            Vector3d point,
            FixedPointAnchor pointA,
            FixedPointAnchor pointB)
        {
            Intersects = intersects;
            Point = point;
            PointA = pointA;
            PointB = pointB;
            DistanceSqr = point.MagnitudeSquared;
        }

        public bool Intersects { get; }

        public Vector3d Point { get; }

        public FixedPointAnchor PointA { get; }

        public FixedPointAnchor PointB { get; }

        public Fixed64 DistanceSqr { get; }

        public static ClosestSimplexResult Intersection =>
            new(true, Vector3d.Zero, default, default);

        public static ClosestSimplexResult FromWeights(
            SupportPoint[] simplex,
            int count,
            Fixed64 first,
            Fixed64 second,
            Fixed64 third,
            Fixed64 fourth)
        {
            Span<Fixed64> weights = stackalloc Fixed64[4];
            weights[0] = first;
            weights[1] = second;
            weights[2] = third;
            weights[3] = fourth;
            return FromArrayWeights(simplex, weights, count);
        }

        public static ClosestSimplexResult FromIndexedWeights(
            SupportPoint[] simplex,
            Span<int> indices,
            Span<Fixed64> weights)
        {
            Span<SupportPoint> selected = stackalloc SupportPoint[3];
            for (int i = 0; i < 3; i++)
                selected[i] = simplex[indices[i]];

            return FromSpanWeights(selected, weights, 3);
        }

        public static ClosestSimplexResult FromSpanWeights(
            Span<SupportPoint> simplex,
            Span<Fixed64> weights,
            int count)
        {
            Span<Vector3d> points = stackalloc Vector3d[4];
            Span<Vector3d> firstLocalPoints = stackalloc Vector3d[4];
            Span<Vector3d> firstLocalDisplacements = stackalloc Vector3d[4];
            Span<Vector3d> secondLocalPoints = stackalloc Vector3d[4];
            Span<Vector3d> secondLocalDisplacements = stackalloc Vector3d[4];
            int frameIndex = 0;
            for (int i = 0; i < count; i++)
            {
                points[i] = simplex[i].Point;
                firstLocalPoints[i] = simplex[i].PointA.LocalPoint;
                firstLocalDisplacements[i] =
                    simplex[i].PointA.LocalDisplacement;
                secondLocalPoints[i] = simplex[i].PointB.LocalPoint;
                secondLocalDisplacements[i] =
                    simplex[i].PointB.LocalDisplacement;
                if (weights[frameIndex] <= Fixed64.Zero
                    && weights[i] > Fixed64.Zero)
                {
                    frameIndex = i;
                }
            }

            _ = Vector3d.TryGetWeightedAverage(
                points[..count],
                weights[..count],
                out Vector3d point);
            _ = Vector3d.TryGetWeightedAverage(
                firstLocalPoints[..count],
                weights[..count],
                out Vector3d firstLocalPoint);
            _ = Vector3d.TryGetWeightedAverage(
                firstLocalDisplacements[..count],
                weights[..count],
                out Vector3d firstLocalDisplacement);
            _ = Vector3d.TryGetWeightedAverage(
                secondLocalPoints[..count],
                weights[..count],
                out Vector3d secondLocalPoint);
            _ = Vector3d.TryGetWeightedAverage(
                secondLocalDisplacements[..count],
                weights[..count],
                out Vector3d secondLocalDisplacement);
            FixedPointAnchor firstFrame = simplex[frameIndex].PointA;
            FixedPointAnchor secondFrame = simplex[frameIndex].PointB;
            return new ClosestSimplexResult(
                false,
                point,
                new FixedPointAnchor(
                    firstFrame.Origin,
                    firstFrame.Rotation,
                    firstLocalPoint,
                    firstLocalDisplacement),
                new FixedPointAnchor(
                    secondFrame.Origin,
                    secondFrame.Rotation,
                    secondLocalPoint,
                    secondLocalDisplacement));
        }

        private static ClosestSimplexResult FromArrayWeights(
            SupportPoint[] simplex,
            Span<Fixed64> weights,
            int count)
        {
            return FromSpanWeights(simplex.AsSpan(), weights, count);
        }
    }

    internal readonly struct TriangleWeights
    {
        public TriangleWeights(Fixed64 a, Fixed64 b, Fixed64 c)
        {
            A = a;
            B = b;
            C = c;
        }

        public Fixed64 A { get; }

        public Fixed64 B { get; }

        public Fixed64 C { get; }
    }
}
