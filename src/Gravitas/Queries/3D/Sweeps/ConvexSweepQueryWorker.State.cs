//=======================================================================
// ConvexSweepQueryWorker.State.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal sealed partial class ConvexSweepQueryWorker
{
    private readonly struct SupportPoint
    {
        public SupportPoint(Vector3d pointA, Vector3d pointB, int workingShift)
        {
            PointA = pointA;
            PointB = pointB;
            Point = GjkSimplexScale.CreateWorkingDifference(pointA, pointB, workingShift);
        }

        public Vector3d PointA { get; }

        public Vector3d PointB { get; }

        public Vector3d Point { get; }
    }

    private readonly struct SweepTriangleCandidate
    {
        public SweepTriangleCandidate(int triangleIndex, Fixed64 lowerBound)
        {
            TriangleIndex = triangleIndex;
            LowerBound = lowerBound;
        }

        public int TriangleIndex { get; }

        public Fixed64 LowerBound { get; }
    }

    private sealed class SweepTriangleCandidateComparer : IComparer<SweepTriangleCandidate>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Compare(SweepTriangleCandidate left, SweepTriangleCandidate right)
        {
            int lowerBoundCompare = left.LowerBound.CompareTo(right.LowerBound);
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
            Vector3d pointA,
            Vector3d pointB,
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

        public Vector3d PointA { get; }

        public Vector3d PointB { get; }

        public Vector3d Normal { get; }

        public static GjkResult CreateIntersection(Vector3d pointA, Vector3d pointB) =>
            new(true, Fixed64.Zero, pointA, pointB, Vector3d.Zero);
    }

    private readonly struct ClosestSimplexResult
    {
        public ClosestSimplexResult(
            bool intersects,
            Vector3d point,
            Vector3d pointA,
            Vector3d pointB)
        {
            Intersects = intersects;
            Point = point;
            PointA = pointA;
            PointB = pointB;
            DistanceSqr = point.MagnitudeSquared;
        }

        public bool Intersects { get; }

        public Vector3d Point { get; }

        public Vector3d PointA { get; }

        public Vector3d PointB { get; }

        public Fixed64 DistanceSqr { get; }

        public static ClosestSimplexResult Intersection =>
            new(true, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);

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
            Vector3d point = Vector3d.Zero;
            Vector3d pointA = Vector3d.Zero;
            Vector3d pointB = Vector3d.Zero;
            for (int i = 0; i < 3; i++)
            {
                SupportPoint support = simplex[indices[i]];
                point += support.Point * weights[i];
                pointA += support.PointA * weights[i];
                pointB += support.PointB * weights[i];
            }

            return new ClosestSimplexResult(false, point, pointA, pointB);
        }

        public static ClosestSimplexResult FromSpanWeights(
            Span<SupportPoint> simplex,
            Span<Fixed64> weights,
            int count)
        {
            Vector3d point = Vector3d.Zero;
            Vector3d pointA = Vector3d.Zero;
            Vector3d pointB = Vector3d.Zero;
            for (int i = 0; i < count; i++)
            {
                point += simplex[i].Point * weights[i];
                pointA += simplex[i].PointA * weights[i];
                pointB += simplex[i].PointB * weights[i];
            }

            return new ClosestSimplexResult(false, point, pointA, pointB);
        }

        private static ClosestSimplexResult FromArrayWeights(
            SupportPoint[] simplex,
            Span<Fixed64> weights,
            int count)
        {
            Vector3d point = Vector3d.Zero;
            Vector3d pointA = Vector3d.Zero;
            Vector3d pointB = Vector3d.Zero;
            for (int i = 0; i < count; i++)
            {
                point += simplex[i].Point * weights[i];
                pointA += simplex[i].PointA * weights[i];
                pointB += simplex[i].PointB * weights[i];
            }

            return new ClosestSimplexResult(false, point, pointA, pointB);
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
