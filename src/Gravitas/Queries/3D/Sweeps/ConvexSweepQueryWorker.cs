//=======================================================================
// ConvexSweepQueryWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Performs deterministic translational convex-source sweeps against 3D query
/// targets using support-mapped conservative advancement.
/// </summary>
internal sealed partial class ConvexSweepQueryWorker
{
    private const int MaxGjkIterations = 32;
    private const int MaxConservativeAdvancementIterations = 32;
    private static readonly Fixed64 DistanceTolerance = Fixed64.FromFraction(1, 1_048_576);
    internal static readonly Fixed64 ContactTolerance = Fixed64.FromFraction(1, 4096);
    private static readonly SweepTriangleCandidateComparer SweepTriangleComparer = new();

    private readonly SupportPoint[] _simplex = new SupportPoint[4];
    private readonly SwiftList<int> _triangleCandidates = new(16);
    private readonly SwiftList<SweepTriangleCandidate> _sweepTriangleCandidates = new(16);
    private readonly int _maxConservativeAdvancementIterations;

    private LSCollider? _source;
    private ConvexShape _sourceShape;
    private bool _hasSource;
    private Vector3d _displacement;
    private Vector3d _direction;
    private Vector3d _sweptSourceBoundsMin;
    private Vector3d _sweptSourceBoundsMax;
    private Fixed64 _length;

    internal int LastMeshTriangleCandidateCount { get; private set; }

    internal ConvexSweepQueryWorker(
        int maxConservativeAdvancementIterations = MaxConservativeAdvancementIterations) =>
        _maxConservativeAdvancementIterations = maxConservativeAdvancementIterations;

    public void PrepareConvexMeshSource(LSMeshCollider source, Vector3d displacement)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        ThrowIfConcaveSource(source);
        Prepare(source, displacement);
    }

    public void PrepareCompoundSource(LSCompoundCollider source, Vector3d displacement)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        Prepare(source, displacement);
    }

    public void PreparePrimitiveSource(LSCollider source, Vector3d displacement)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        if (!ConvexColliderSupport.IsSupported(source))
            throw new NotSupportedException(
                $"Convex swept queries do not support {source.GetType().Name} sources.");

        Prepare(source, displacement);
    }

    public void PrepareCircleSlabSource(Vector3d center, Fixed64 radius, Fixed64 halfHeight, Vector3d displacement)
    {
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Circle-slab sweep radius must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(halfHeight <= Fixed64.Zero, nameof(halfHeight), "Circle-slab sweep half-height must be greater than zero.");
        _source = null;
        Prepare(ConvexShape.CreateCircleSlab(center, radius, halfHeight), displacement);
    }

    public bool TrySweepPreparedSource(LSCollider target, out Physics3DHit hit)
    {
        LastMeshTriangleCandidateCount = 0;
        hit = default;
        if (!_hasSource
            || _length <= Fixed64.Epsilon
            || !SweepBoundsUtility.OverlapsInclusive(_sweptSourceBoundsMin, _sweptSourceBoundsMax, target.BoundsMin, target.BoundsMax))
        {
            return false;
        }

        if (_source is LSCompoundCollider compound)
            return TrySweepCompoundSource(compound, target, out hit);

        return TrySweepSourceShape(_sourceShape, target, out hit);
    }

    private void Prepare(LSCollider source, Vector3d displacement)
    {
        _source = source;
        Prepare(CreateColliderShape(source, Vector3d.Zero), displacement);
    }

    private void Prepare(ConvexShape sourceShape, Vector3d displacement)
    {
        _sourceShape = sourceShape;
        _displacement = displacement;
        _hasSource = Vector3d.TryGetMagnitude(displacement, out _length);
        if (!_hasSource)
        {
            _direction = Vector3d.Zero;
            return;
        }

        _direction = _length <= Fixed64.Epsilon ? Vector3d.Zero : displacement.Normalized;
        sourceShape.GetSourceBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        if (!Vector3d.TryAdd(sourceMin, displacement, out _)
            || !Vector3d.TryAdd(sourceMax, displacement, out _))
        {
            _hasSource = false;
            _direction = Vector3d.Zero;
            return;
        }

        SweepBoundsUtility.CreateSweptBounds(
            sourceMin,
            sourceMax,
            displacement,
            ContactTolerance,
            out _sweptSourceBoundsMin,
            out _sweptSourceBoundsMax);
    }

    private bool TrySweepCompoundSource(LSCompoundCollider source, LSCollider target, out Physics3DHit hit)
    {
        hit = default;
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        int closestPartIndex = int.MaxValue;

        for (int i = 0; i < source.PartCount; i++)
        {
            LSCollider part = source.GetPartCollider(i);
            if (!TrySweepSourceShape(CreateColliderShape(part, Vector3d.Zero), target, out Physics3DHit candidate)
                || !ComesBeforeReducerCandidate(candidate, i, found, closestDistance, closestPartIndex))
            {
                continue;
            }

            hit = candidate;
            closestDistance = candidate.Distance;
            closestPartIndex = i;
            found = true;
        }

        return found;
    }

    private bool TrySweepSourceShape(ConvexShape sourceShape, LSCollider target, out Physics3DHit hit)
    {
        hit = default;

        if (!CanSweptSourceShapeReachTarget(sourceShape, target))
            return false;

        if (target is LSCompoundCollider compound)
            return TrySweepTargetCompound(sourceShape, compound, out hit);

        if (target is LSMeshCollider mesh && mesh.Mode == MeshColliderMode.Concave)
            return TrySweepConcaveMeshTarget(sourceShape, mesh, out hit);

        return TrySweepConvexTarget(sourceShape, CreateColliderShape(target, Vector3d.Zero), target, out hit);
    }

    private bool TrySweepTargetCompound(ConvexShape sourceShape, LSCompoundCollider compound, out Physics3DHit hit)
    {
        hit = default;
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        int closestPartIndex = int.MaxValue;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!TrySweepSourceShape(sourceShape, part, out Physics3DHit partHit)
                || !ComesBeforeReducerCandidate(partHit, i, found, closestDistance, closestPartIndex))
            {
                continue;
            }

            hit = new Physics3DHit(compound, partHit.Point, partHit.Normal, partHit.Distance, partHit.Direction);
            closestDistance = partHit.Distance;
            closestPartIndex = i;
            found = true;
        }

        return found;
    }

    private bool TrySweepConcaveMeshTarget(ConvexShape sourceShape, LSMeshCollider mesh, out Physics3DHit hit)
    {
        hit = default;
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        int closestTriangleIndex = int.MaxValue;

        CreateSweptSourceBounds(sourceShape, out Vector3d min, out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _triangleCandidates);
        LastMeshTriangleCandidateCount += _triangleCandidates.Count;
        BuildOrderedSweepTriangleCandidates(sourceShape, mesh);

        for (int i = 0; i < _sweepTriangleCandidates.Count; i++)
        {
            SweepTriangleCandidate sweepCandidate = _sweepTriangleCandidates[i];
            if (RemainingSweepTrianglesCannotBeat(
                sweepCandidate,
                found,
                closestDistance,
                closestTriangleIndex))
            {
                break;
            }

            int triangleIndex = sweepCandidate.TriangleIndex;
            ConvexShape triangle = CreateTriangleShape(mesh, triangleIndex);
            if (!TrySweepConvexTarget(sourceShape, triangle, mesh, out Physics3DHit candidate)
                || !ComesBeforeReducerCandidate(candidate, triangleIndex, found, closestDistance, closestTriangleIndex))
            {
                continue;
            }

            hit = candidate;
            closestDistance = candidate.Distance;
            closestTriangleIndex = triangleIndex;
            found = true;
        }

        return found;
    }

    private void BuildOrderedSweepTriangleCandidates(ConvexShape sourceShape, LSMeshCollider mesh)
    {
        _sweepTriangleCandidates.FastClear();
        for (int i = 0; i < _triangleCandidates.Count; i++)
        {
            int triangleIndex = _triangleCandidates[i];
            ConvexShape triangle = CreateTriangleShape(mesh, triangleIndex);
            Fixed64 lowerBound = ComputeSweepLowerBound(sourceShape, triangle);
            if (lowerBound <= _length + ContactTolerance)
                _sweepTriangleCandidates.Add(new SweepTriangleCandidate(triangleIndex, lowerBound));
        }

        _sweepTriangleCandidates.SortInPlace(SweepTriangleComparer);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ComputeSweepLowerBound(ConvexShape sourceShape, ConvexShape targetShape)
    {
        Vector3d sourceFront = sourceShape.Support(_direction);
        Vector3d targetBack = targetShape.Support(-_direction);
        Fixed64 projectedSeparation = Vector3d.ProjectNonNegativeDifference(
            targetBack,
            sourceFront,
            _direction);
        return FixedMath.Max(projectedSeparation - ContactTolerance, Fixed64.Zero);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool RemainingSweepTrianglesCannotBeat(
        SweepTriangleCandidate candidate,
        bool found,
        Fixed64 closestDistance,
        int closestTriangleIndex)
    {
        if (!found)
            return false;

        if (candidate.LowerBound > closestDistance + DistanceTolerance)
            return true;

        // Conservative advancement accepts contacts inside ContactTolerance;
        // once the best lower-index triangle is already at that recognized TOI,
        // later same-bound triangles cannot win the deterministic tie-break.
        return closestTriangleIndex < candidate.TriangleIndex
            && closestDistance <= candidate.LowerBound + ContactTolerance;
    }

    private bool TrySweepConvexTarget(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        LSCollider targetCollider,
        out Physics3DHit hit)
    {
        hit = default;
        Fixed64 travelDistance = Fixed64.Zero;
        Vector3d normal = Vector3d.Zero;
        GjkResult result = default;

        for (int i = 0; i < _maxConservativeAdvancementIterations; i++)
        {
            ConvexShape movedSource = sourceShape.WithSourceOffset(_direction * travelDistance);
            result = ComputeDistance(movedSource, targetShape);
            if (result.Intersects || result.Distance <= ContactTolerance)
            {
                Vector3d point = ResolveHitPoint(
                    targetShape,
                    targetCollider,
                    movedSource,
                    result,
                    out bool hasRefinedSurfaceNormal);
                Vector3d hitNormal = ResolveHitNormal(
                    targetShape,
                    targetCollider,
                    point,
                    result.Normal,
                    normal,
                    hasRefinedSurfaceNormal);

                hit = new Physics3DHit(targetCollider, point, hitNormal, travelDistance, _direction);
                return true;
            }

            normal = result.Normal;
            Fixed64 closingSpeed = -Vector3d.Dot(_direction, normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 stepDistance = result.Distance / closingSpeed;
            Fixed64 nextTravelDistance = travelDistance + stepDistance;
            if (nextTravelDistance > _length)
            {
                ConvexShape endpointSource = sourceShape.WithSourceOffset(_displacement);
                GjkResult endpointResult = ComputeDistance(endpointSource, targetShape);
                if (!endpointResult.Intersects && endpointResult.Distance > ContactTolerance)
                    return false;

                Vector3d point = ResolveHitPoint(
                    targetShape,
                    targetCollider,
                    endpointSource,
                    endpointResult,
                    out bool hasRefinedSurfaceNormal);
                Vector3d hitNormal = ResolveHitNormal(
                    targetShape,
                    targetCollider,
                    point,
                    endpointResult.Normal,
                    normal,
                    hasRefinedSurfaceNormal);
                hit = new Physics3DHit(targetCollider, point, hitNormal, _length, _direction);
                return true;
            }

            travelDistance = nextTravelDistance;
        }

        return false;
    }

    private Vector3d ResolveHitPoint(
        ConvexShape targetShape,
        LSCollider targetCollider,
        ConvexShape movedSource,
        GjkResult result,
        out bool hasRefinedSurfaceNormal)
    {
        hasRefinedSurfaceNormal = false;
        if (targetCollider is LSSphereCollider
            && movedSource.TryGetClosestPointOnSurface(targetCollider.Center, out Vector3d sourcePoint))
        {
            // A sphere's closest pair is defined by its center and the closest
            // source feature. Refining that feature removes arbitrary support
            // tie bias without changing the conservative TOI.
            hasRefinedSurfaceNormal = true;
            return targetCollider.ClosestPointOnSurface(sourcePoint);
        }

        if ((movedSource.Center - targetCollider.Center).MagnitudeSquared <= Fixed64.Epsilon)
        {
            Vector3d fallbackDirection = -_direction;
            Vector3d surfaceProbe = targetCollider.Center + fallbackDirection * targetCollider.ScaledRadius;
            return targetCollider.ClosestPointOnSurface(surfaceProbe);
        }

        // GJK's target witness identifies the feature that stopped the sweep.
        // Center-to-center projection can select an unrelated feature for long,
        // offset shapes and therefore produce a non-physical response normal.
        return targetShape.IsTriangle
            ? result.PointB
            : targetCollider.ClosestPointOnSurface(result.PointB);
    }

    private Vector3d ResolveHitNormal(
        ConvexShape targetShape,
        LSCollider targetCollider,
        Vector3d point,
        Vector3d resultNormal,
        Vector3d fallbackNormal,
        bool hasRefinedSurfaceNormal)
    {
        targetShape.TryGetPlanarSurfaceNormal(point, out Vector3d planarNormal);
        return ConvexSweepHitPolicy.ResolveHitNormal(
            targetCollider,
            point,
            resultNormal,
            fallbackNormal,
            _direction,
            planarNormal,
            hasRefinedSurfaceNormal);
    }

    private GjkResult ComputeDistance(ConvexShape sourceShape, ConvexShape targetShape)
    {
        sourceShape.GetBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        targetShape.GetBounds(out Vector3d targetMin, out Vector3d targetMax);
        int workingShift = GjkSimplexScale.SelectTwoTermShift(sourceMin, sourceMax, targetMin, targetMax);
        Fixed64 workingScale = GjkSimplexScale.GetCoordinateScale(workingShift);
        Fixed64 workingDistanceTolerance = DistanceTolerance * workingScale;
        int simplexCount = 0;
        Vector3d direction = GjkSimplexScale.CreateWorkingDifference(
            targetShape.Center,
            sourceShape.Center,
            workingShift);
        if (direction == Vector3d.Zero)
        {
            if (sourceShape.ContainsCenter && targetShape.ContainsCenter)
                return GjkResult.CreateIntersection(sourceShape.Center, targetShape.Center);

            direction = Vector3d.Right;
        }

        bool hasPreviousDistance = false;
        Fixed64 previousDistance = Fixed64.Zero;
        ClosestSimplexResult closest = default;
        bool distanceIsRepresentable = false;
        Fixed64 workingDistance = Fixed64.MaxValue;

        for (int i = 0; i < MaxGjkIterations; i++)
        {
            SupportPoint support = CreateSupportPoint(sourceShape, targetShape, direction, workingShift);
            if (ContainsSupportPoint(_simplex, simplexCount, support.Point))
                break;

            ClosestSimplexResult previousClosest = closest;
            _simplex[simplexCount++] = support;
            closest = SolveClosestSimplex(_simplex, ref simplexCount, workingScale);
            distanceIsRepresentable = Vector3d.TryGetMagnitude(closest.Point, out workingDistance);
            if (closest.Intersects)
            {
                // Tetrahedron entry follows a valid one-to-three point simplex.
                // Preserve that same-pose closest pair as the deterministic
                // surface witness for the intersection result.
                return GjkResult.CreateIntersection(previousClosest.PointA, previousClosest.PointB);
            }

            if (distanceIsRepresentable && workingDistance <= workingDistanceTolerance)
                return GjkResult.CreateIntersection(closest.PointA, closest.PointB);

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
        Vector3d normal = closest.Point.Normalized;
        return new GjkResult(false, distance, closest.PointA, closest.PointB, normal);
    }

    private static SupportPoint CreateSupportPoint(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        Vector3d direction,
        int workingShift)
    {
        Vector3d supportDirection = direction.Normalized;
        Vector3d pointA = sourceShape.Support(supportDirection);
        Vector3d pointB = targetShape.Support(-supportDirection);
        return new SupportPoint(pointA, pointB, workingShift);
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
        ref int count,
        Fixed64 workingScale)
    {
        if (IsOriginInsideTetrahedron(
                simplex[0].Point,
                simplex[1].Point,
                simplex[2].Point,
                simplex[3].Point,
                workingScale))
        {
            count = 4;
            return ClosestSimplexResult.Intersection;
        }

        ClosestSimplexResult best = default;
        bool hasBest = false;
        Span<int> bestIndices = stackalloc int[3];
        Span<Fixed64> bestWeights = stackalloc Fixed64[3];
        Span<int> face = stackalloc int[3];
        Span<Fixed64> weights = stackalloc Fixed64[3];

        EvaluateFace(simplex, 0, 1, 2, ref best, ref hasBest, bestIndices, bestWeights, face, weights);
        EvaluateFace(simplex, 0, 3, 1, ref best, ref hasBest, bestIndices, bestWeights, face, weights);
        EvaluateFace(simplex, 0, 2, 3, ref best, ref hasBest, bestIndices, bestWeights, face, weights);
        EvaluateFace(simplex, 1, 3, 2, ref best, ref hasBest, bestIndices, bestWeights, face, weights);

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
        ref ClosestSimplexResult best,
        ref bool hasBest,
        Span<int> bestIndices,
        Span<Fixed64> bestWeights,
        Span<int> face,
        Span<Fixed64> weights)
    {
        face[0] = first;
        face[1] = second;
        face[2] = third;
        TriangleWeights triangleWeights = ClosestPointOnTriangleToOrigin(
            simplex[first].Point,
            simplex[second].Point,
            simplex[third].Point);
        weights[0] = triangleWeights.A;
        weights[1] = triangleWeights.B;
        weights[2] = triangleWeights.C;
        ClosestSimplexResult candidate = ClosestSimplexResult.FromIndexedWeights(simplex, face, weights);
        if (hasBest && !IsCloser(
                candidate.DistanceSqr,
                candidate.Point,
                best.DistanceSqr,
                best.Point))
            return;

        best = candidate;
        hasBest = true;
        bestIndices[0] = first;
        bestIndices[1] = second;
        bestIndices[2] = third;
        bestWeights[0] = weights[0];
        bestWeights[1] = weights[1];
        bestWeights[2] = weights[2];
    }

    internal static bool IsCloser(
        Fixed64 candidateDistanceSqr,
        Vector3d candidatePoint,
        Fixed64 bestDistanceSqr,
        Vector3d bestPoint)
    {
        if (candidateDistanceSqr != bestDistanceSqr || candidateDistanceSqr != Fixed64.MaxValue)
            return candidateDistanceSqr < bestDistanceSqr;

        return Vector3d.CompareMagnitudeSquared(candidatePoint, bestPoint) < 0;
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

        Vector3d ab = b - a;
        Vector3d ac = c - a;
        Vector3d ap = -a;
        Fixed64 d1 = Vector3d.Dot(ab, ap);
        Fixed64 d2 = Vector3d.Dot(ac, ap);
        if (d1 <= Fixed64.Zero && d2 <= Fixed64.Zero)
            return new TriangleWeights(Fixed64.One, Fixed64.Zero, Fixed64.Zero);

        Vector3d bp = -b;
        Fixed64 d3 = Vector3d.Dot(ab, bp);
        Fixed64 d4 = Vector3d.Dot(ac, bp);
        if (d3 >= Fixed64.Zero && d4 <= d3)
            return new TriangleWeights(Fixed64.Zero, Fixed64.One, Fixed64.Zero);

        Fixed64 vc = d1 * d4 - d3 * d2;
        if (vc <= Fixed64.Zero && d1 >= Fixed64.Zero && d3 <= Fixed64.Zero)
        {
            Fixed64 v = d1 / (d1 - d3);
            return new TriangleWeights(Fixed64.One - v, v, Fixed64.Zero);
        }

        Vector3d cp = -c;
        Fixed64 d5 = Vector3d.Dot(ab, cp);
        Fixed64 d6 = Vector3d.Dot(ac, cp);
        if (d6 >= Fixed64.Zero && d5 <= d6)
            return new TriangleWeights(Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        Fixed64 vb = d5 * d2 - d1 * d6;
        if (vb <= Fixed64.Zero && d2 >= Fixed64.Zero && d6 <= Fixed64.Zero)
        {
            Fixed64 w = d2 / (d2 - d6);
            return new TriangleWeights(Fixed64.One - w, Fixed64.Zero, w);
        }

        Fixed64 va = d3 * d6 - d5 * d4;
        if (va <= Fixed64.Zero && (d4 - d3) >= Fixed64.Zero && (d5 - d6) >= Fixed64.Zero)
        {
            Fixed64 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return new TriangleWeights(Fixed64.Zero, Fixed64.One - w, w);
        }

        Fixed64 denominator = Fixed64.One / (va + vb + vc);
        Fixed64 vInside = vb * denominator;
        Fixed64 wInside = vc * denominator;
        return new TriangleWeights(Fixed64.One - vInside - wInside, vInside, wInside);
    }

    private static bool IsOriginInsideTetrahedron(
        Vector3d a,
        Vector3d b,
        Vector3d c,
        Vector3d d,
        Fixed64 workingScale)
    {
        Span<Vector3d> scaled = stackalloc Vector3d[4];
        scaled[0] = a;
        scaled[1] = b;
        scaled[2] = c;
        scaled[3] = d;
        Fixed64 productScale = GjkSimplexScale.ScaleForProducts(scaled);
        a = scaled[0];
        b = scaled[1];
        c = scaled[2];
        d = scaled[3];

        Fixed64 productScaleSqr = productScale * productScale;
        Fixed64 productScaleCubed = productScaleSqr * productScale;
        Fixed64 workingScaleSqr = workingScale * workingScale;
        Fixed64 workingScaleCubed = workingScaleSqr * workingScale;
        Fixed64 volumeTolerance = DistanceTolerance * workingScaleCubed * productScaleCubed;
        if (SignedTetrahedronVolume6(a, b, c, d).Abs() <= volumeTolerance)
            return false;

        Fixed64 workingScaleSixth = workingScaleCubed * workingScaleCubed;
        Fixed64 sideTolerance =
            DistanceTolerance * workingScaleSixth * productScaleCubed * productScaleCubed;
        return IsSameSideOfFace(Vector3d.Zero, d, a, b, c, sideTolerance)
            && IsSameSideOfFace(Vector3d.Zero, c, a, d, b, sideTolerance)
            && IsSameSideOfFace(Vector3d.Zero, b, a, c, d, sideTolerance)
            && IsSameSideOfFace(Vector3d.Zero, a, b, d, c, sideTolerance);
    }

    private static bool IsSameSideOfFace(
        Vector3d point,
        Vector3d opposite,
        Vector3d a,
        Vector3d b,
        Vector3d c,
        Fixed64 sideTolerance)
    {
        Vector3d normal = Vector3d.Cross(b - a, c - a);
        if (normal.MagnitudeSquared <= Fixed64.Zero)
            return false;

        Fixed64 pointSide = Vector3d.Dot(normal, point - a);
        Fixed64 oppositeSide = Vector3d.Dot(normal, opposite - a);
        return pointSide * oppositeSide >= -sideTolerance;
    }

    private static Fixed64 SignedTetrahedronVolume6(Vector3d a, Vector3d b, Vector3d c, Vector3d d) =>
        Vector3d.Dot(b - a, Vector3d.Cross(c - a, d - a));

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

    private static ConvexShape CreateColliderShape(LSCollider collider, Vector3d offset)
    {
        return new ConvexShape(collider, offset);
    }

    private static ConvexShape CreateTriangleShape(LSMeshCollider mesh, int triangleIndex)
    {
        mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
        return new ConvexShape(mesh, triangleIndex, first, second, third);
    }

    private static void ThrowIfConcaveSource(LSMeshCollider source)
    {
        if (source.Mode == MeshColliderMode.Concave)
            throw CreateConcaveSourceException(source);
    }

    private void CreateSweptSourceBounds(ConvexShape sourceShape, out Vector3d min, out Vector3d max)
    {
        sourceShape.GetSourceBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        SweepBoundsUtility.CreateSweptBounds(
            sourceMin,
            sourceMax,
            _displacement,
            ContactTolerance,
            out min,
            out max);
    }

    private bool CanSweptSourceShapeReachTarget(ConvexShape sourceShape, LSCollider target)
    {
        sourceShape.GetSourceBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        SweepBoundsUtility.CreateSweptBounds(
            sourceMin,
            sourceMax,
            _displacement,
            ContactTolerance,
            out Vector3d min,
            out Vector3d max);
        return SweepBoundsUtility.OverlapsInclusive(min, max, target.BoundsMin, target.BoundsMax);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ComesBeforeReducerCandidate(
        Physics3DHit hit,
        int candidateOrdinal,
        bool found,
        Fixed64 closestDistance,
        int closestOrdinal)
    {
        if (!found)
            return true;

        int distanceCompare = hit.Distance.CompareTo(closestDistance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        return candidateOrdinal < closestOrdinal;
    }

    private static ArgumentException CreateConcaveSourceException(LSMeshCollider source) =>
        new("Concave mesh sources are not supported by swept query APIs. Use an LSCompoundCollider built from authored convex decomposition parts.", nameof(source));
}
