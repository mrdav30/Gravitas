//=======================================================================
// ConvexSweepQueryWorker.State.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Queries;

/// <content>Defines support samples, triangle candidates, and simplex results for convex sweeps.</content>
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



    private readonly struct ClosestSimplexResult
    {
        private readonly int _count;
        private readonly Fixed64 _first;
        private readonly Fixed64 _second;
        private readonly Fixed64 _third;
        private readonly Fixed64 _fourth;

        private ClosestSimplexResult(bool intersects, Vector3d point, int count,
            Fixed64 first, Fixed64 second, Fixed64 third, Fixed64 fourth)
        {
            Intersects = intersects;
            Point = point;
            _count = count;
            _first = first;
            _second = second;
            _third = third;
            _fourth = fourth;
        }

        public bool Intersects { get; }

        public Vector3d Point { get; }

        public static ClosestSimplexResult Intersection =>
            new(true, Vector3d.Zero, 0, default, default, default, default);

        public static ClosestSimplexResult FromWeights(
            SupportPoint[] simplex,
            int count,
            Fixed64 first,
            Fixed64 second,
            Fixed64 third,
            Fixed64 fourth)
        {
            ReadOnlySpan<Fixed64> weights = stackalloc Fixed64[4] { first, second, third, fourth };
            return FromSpanWeights(simplex, weights, count);
        }

        public static ClosestSimplexResult FromSpanWeights(
            ReadOnlySpan<SupportPoint> simplex,
            ReadOnlySpan<Fixed64> weights,
            int count)
        {
            if (count == 1)
                return new ClosestSimplexResult(false, simplex[0].Point, count,
                    weights[0], weights[1], weights[2], weights[3]);
            Span<Vector3d> points = stackalloc Vector3d[4];
            for (int i = 0; i < count; i++)
                points[i] = simplex[i].Point;
            _ = Vector3d.TryGetWeightedAverage(points[..count], weights[..count], out Vector3d point);
            return new ClosestSimplexResult(false, point, count,
                weights[0], weights[1], weights[2], weights[3]);
        }

        public void GetWitnesses(ReadOnlySpan<SupportPoint> simplex,
            out FixedPointAnchor pointA, out FixedPointAnchor pointB)
        {
            // Iterations need only the Minkowski point; materialize the selected witnesses once.
            if (_count == 1)
            {
                FixedPointAnchor first = simplex[0].PointA;
                FixedPointAnchor second = simplex[0].PointB;
                pointA = new FixedPointAnchor(first.Origin, first.Rotation, first.LocalPoint, first.LocalDisplacement);
                pointB = new FixedPointAnchor(second.Origin, second.Rotation, second.LocalPoint, second.LocalDisplacement);
                return;
            }
            ReadOnlySpan<Fixed64> weights = stackalloc Fixed64[4] { _first, _second, _third, _fourth };
            Span<Vector3d> firstLocalPoints = stackalloc Vector3d[4];
            Span<Vector3d> firstLocalDisplacements = stackalloc Vector3d[4];
            Span<Vector3d> secondLocalPoints = stackalloc Vector3d[4];
            Span<Vector3d> secondLocalDisplacements = stackalloc Vector3d[4];
            int frameIndex = 0;
            for (int i = 0; i < _count; i++)
            {
                firstLocalPoints[i] = simplex[i].PointA.LocalPoint;
                firstLocalDisplacements[i] = simplex[i].PointA.LocalDisplacement;
                secondLocalPoints[i] = simplex[i].PointB.LocalPoint;
                secondLocalDisplacements[i] = simplex[i].PointB.LocalDisplacement;
                if (weights[frameIndex] <= Fixed64.Zero && weights[i] > Fixed64.Zero)
                    frameIndex = i;
            }

            _ = Vector3d.TryGetWeightedAverage(firstLocalPoints[.._count], weights[.._count], out Vector3d firstLocalPoint);
            _ = Vector3d.TryGetWeightedAverage(firstLocalDisplacements[.._count], weights[.._count], out Vector3d firstLocalDisplacement);
            _ = Vector3d.TryGetWeightedAverage(secondLocalPoints[.._count], weights[.._count], out Vector3d secondLocalPoint);
            _ = Vector3d.TryGetWeightedAverage(secondLocalDisplacements[.._count], weights[.._count], out Vector3d secondLocalDisplacement);
            FixedPointAnchor firstFrame = simplex[frameIndex].PointA;
            FixedPointAnchor secondFrame = simplex[frameIndex].PointB;
            pointA = new FixedPointAnchor(firstFrame.Origin, firstFrame.Rotation, firstLocalPoint, firstLocalDisplacement);
            pointB = new FixedPointAnchor(secondFrame.Origin, secondFrame.Rotation, secondLocalPoint, secondLocalDisplacement);
        }
    }


}
