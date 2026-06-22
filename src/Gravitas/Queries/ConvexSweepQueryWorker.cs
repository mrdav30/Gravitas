//=======================================================================
// ConvexSweepQueryWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Performs deterministic translational convex-source sweeps against 3D query
/// targets using support-mapped conservative advancement.
/// </summary>
internal sealed class ConvexSweepQueryWorker
{
    private const int MaxGjkIterations = 32;
    private const int MaxConservativeAdvancementIterations = 32;
    private static readonly Fixed64 DistanceTolerance = Fixed64.FromFraction(1, 1_048_576);
    private static readonly Fixed64 DistanceToleranceSqr = DistanceTolerance * DistanceTolerance;
    private static readonly Fixed64 SweepContactTolerance = Fixed64.FromFraction(1, 4096);
    private static readonly Fixed64 ProgressToleranceSqr = DistanceToleranceSqr;

    private readonly SupportPoint[] _simplex = new SupportPoint[4];
    private readonly SwiftList<int> _triangleCandidates = new(16);

    private LSCollider? _source;
    private Vector3d _displacement;
    private Vector3d _direction;
    private Fixed64 _length;

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
        if (!IsSupportedSourcePart(source))
            throw new NotSupportedException(
                $"Convex swept queries do not support {source.GetType().Name} sources.");

        Prepare(source, displacement);
    }

    public bool TrySweepPreparedSource(LSCollider target, out Physics3DHit hit)
    {
        hit = default;
        if (_source == null || _length <= Fixed64.Epsilon)
            return false;

        if (_source is LSCompoundCollider compound)
            return TrySweepCompoundSource(compound, target, out hit);

        return TrySweepSourceShape(CreateColliderShape(_source, Vector3d.Zero), target, out hit);
    }

    private void Prepare(LSCollider source, Vector3d displacement)
    {
        _source = source;
        _displacement = displacement;
        _length = displacement.Magnitude;
        _direction = _length <= Fixed64.Epsilon ? Vector3d.Zero : displacement / _length;
    }

    private bool TrySweepCompoundSource(LSCompoundCollider source, LSCollider target, out Physics3DHit hit)
    {
        hit = default;
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;

        for (int i = 0; i < source.PartCount; i++)
        {
            LSCollider part = source.GetPartCollider(i);
            if (!IsSupportedSourcePart(part))
                throw new NotSupportedException(
                    $"Compound swept query sources do not support {part.GetType().Name} parts.");

            if (!TrySweepSourceShape(CreateColliderShape(part, Vector3d.Zero), target, out Physics3DHit candidate)
                || !ComesBefore(candidate, found, closestDistance, hit))
            {
                continue;
            }

            hit = candidate;
            closestDistance = candidate.Distance;
            found = true;
        }

        return found;
    }

    private bool TrySweepSourceShape(ConvexShape sourceShape, LSCollider target, out Physics3DHit hit)
    {
        hit = default;

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

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!TrySweepSourceShape(sourceShape, part, out Physics3DHit partHit)
                || !ComesBefore(partHit, found, closestDistance, hit))
            {
                continue;
            }

            hit = new Physics3DHit(compound, partHit.Point, partHit.Normal, partHit.Distance, partHit.Direction);
            closestDistance = partHit.Distance;
            found = true;
        }

        return found;
    }

    private bool TrySweepConcaveMeshTarget(ConvexShape sourceShape, LSMeshCollider mesh, out Physics3DHit hit)
    {
        hit = default;
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;

        CreateSweptSourceBounds(sourceShape, out Vector3d min, out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _triangleCandidates);

        for (int i = 0; i < _triangleCandidates.Count; i++)
        {
            ConvexShape triangle = CreateTriangleShape(mesh, _triangleCandidates[i]);
            if (!TrySweepConvexTarget(sourceShape, triangle, mesh, out Physics3DHit candidate)
                || !ComesBefore(candidate, found, closestDistance, hit))
            {
                continue;
            }

            hit = candidate;
            closestDistance = candidate.Distance;
            found = true;
        }

        return found;
    }

    private bool TrySweepConvexTarget(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        LSCollider targetCollider,
        out Physics3DHit hit)
    {
        hit = default;
        Fixed64 travelDistance = Fixed64.Zero;
        Vector3d normal = _direction.MagnitudeSquared > Fixed64.Epsilon ? -_direction : Vector3d.Zero;
        GjkResult result = default;

        for (int i = 0; i < MaxConservativeAdvancementIterations; i++)
        {
            ConvexShape movedSource = sourceShape.WithOffset(_direction * travelDistance);
            result = ComputeDistance(movedSource, targetShape);
            if (result.Intersects || result.Distance <= SweepContactTolerance)
            {
                Vector3d point = ResolveHitPoint(targetShape, targetCollider, movedSource, result);
                Vector3d hitNormal = ResolveHitNormal(targetCollider, point, result.Normal, normal);

                hit = new Physics3DHit(targetCollider, point, hitNormal, travelDistance, _direction);
                return true;
            }

            normal = result.Normal;
            Fixed64 closingSpeed = -Vector3d.Dot(_direction, normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 stepDistance = result.Distance / closingSpeed;
            if (stepDistance <= Fixed64.Zero)
                return false;

            travelDistance += stepDistance;
            if (travelDistance > _length)
                return false;
        }

        return false;
    }

    private Vector3d ResolveHitPoint(
        ConvexShape targetShape,
        LSCollider targetCollider,
        ConvexShape movedSource,
        GjkResult result)
    {
        if (targetShape.IsTriangle && result.PointB.MagnitudeSquared > Fixed64.Epsilon)
            return result.PointB;

        if ((movedSource.Center - targetCollider.Center).MagnitudeSquared <= Fixed64.Epsilon)
        {
            if (result.PointB.MagnitudeSquared > Fixed64.Epsilon)
                return result.PointB;

            Vector3d fallbackDirection = _direction.MagnitudeSquared > Fixed64.Epsilon
                ? -_direction
                : Vector3d.Right;
            return targetCollider.Center + fallbackDirection * targetCollider.ScaledRadius;
        }

        return targetCollider.ClosestPointOnSurface(movedSource.Center);
    }

    private Vector3d ResolveHitNormal(
        LSCollider targetCollider,
        Vector3d point,
        Vector3d resultNormal,
        Vector3d fallbackNormal)
    {
        Vector3d normal = targetCollider.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            normal = resultNormal.MagnitudeSquared > Fixed64.Epsilon ? resultNormal : fallbackNormal;

        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return _direction.MagnitudeSquared > Fixed64.Epsilon ? -_direction : Vector3d.Zero;

        normal = normal.Normalized;
        return Vector3d.Dot(normal, _direction) > Fixed64.Zero ? -normal : normal;
    }

    private GjkResult ComputeDistance(ConvexShape sourceShape, ConvexShape targetShape)
    {
        int simplexCount = 0;
        Vector3d direction = targetShape.Center - sourceShape.Center;
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            direction = Vector3d.Right;

        Fixed64 previousDistanceSqr = Fixed64.MaxValue;
        ClosestSimplexResult closest = default;

        for (int i = 0; i < MaxGjkIterations; i++)
        {
            SupportPoint support = CreateSupportPoint(sourceShape, targetShape, direction);
            if (ContainsSupportPoint(_simplex, simplexCount, support.Point))
                break;

            _simplex[simplexCount++] = support;
            closest = SolveClosestSimplex(_simplex, ref simplexCount);
            if (closest.Intersects || closest.DistanceSqr <= DistanceToleranceSqr)
            {
                return GjkResult.CreateIntersection(closest.PointA, closest.PointB);
            }

            if (previousDistanceSqr - closest.DistanceSqr <= ProgressToleranceSqr)
                break;

            previousDistanceSqr = closest.DistanceSqr;
            direction = -closest.Point;
            if (direction.MagnitudeSquared <= DistanceToleranceSqr)
                return GjkResult.CreateIntersection(closest.PointA, closest.PointB);
        }

        Fixed64 distance = closest.DistanceSqr <= Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Sqrt(closest.DistanceSqr);
        Vector3d normal = closest.Point.MagnitudeSquared > Fixed64.Epsilon
            ? closest.Point.Normalized
            : Vector3d.Zero;
        return new GjkResult(false, distance, closest.PointA, closest.PointB, normal);
    }

    private static SupportPoint CreateSupportPoint(ConvexShape sourceShape, ConvexShape targetShape, Vector3d direction)
    {
        Vector3d pointA = sourceShape.Support(direction);
        Vector3d pointB = targetShape.Support(-direction);
        return new SupportPoint(pointA, pointB);
    }

    private static ClosestSimplexResult SolveClosestSimplex(SupportPoint[] simplex, ref int count)
    {
        if (count == 1)
            return ClosestSimplexResult.FromWeights(simplex, 1, Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        if (count == 2)
            return ReduceSegment(simplex, ref count);

        if (count == 3)
            return ReduceTriangle(simplex, ref count);

        return ReduceTetrahedron(simplex, ref count);
    }

    private static ClosestSimplexResult ReduceSegment(SupportPoint[] simplex, ref int count)
    {
        SupportPoint a = simplex[0];
        SupportPoint b = simplex[1];
        Vector3d ab = b.Point - a.Point;
        Fixed64 denominator = ab.MagnitudeSquared;
        Fixed64 t = denominator <= Fixed64.Epsilon
            ? Fixed64.Zero
            : FixedMath.Clamp(-Vector3d.Dot(a.Point, ab) / denominator, Fixed64.Zero, Fixed64.One);

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

    private static ClosestSimplexResult ReduceTetrahedron(SupportPoint[] simplex, ref int count)
    {
        if (IsOriginInsideTetrahedron(simplex[0].Point, simplex[1].Point, simplex[2].Point, simplex[3].Point))
        {
            count = 4;
            return ClosestSimplexResult.Intersection;
        }

        ClosestSimplexResult best = default;
        Fixed64 bestDistanceSqr = Fixed64.MaxValue;
        Span<int> bestIndices = stackalloc int[3];
        Span<Fixed64> bestWeights = stackalloc Fixed64[3];
        Span<int> face = stackalloc int[3];
        Span<Fixed64> weights = stackalloc Fixed64[3];

        EvaluateFace(simplex, 0, 1, 2, ref best, ref bestDistanceSqr, bestIndices, bestWeights, face, weights);
        EvaluateFace(simplex, 0, 3, 1, ref best, ref bestDistanceSqr, bestIndices, bestWeights, face, weights);
        EvaluateFace(simplex, 0, 2, 3, ref best, ref bestDistanceSqr, bestIndices, bestWeights, face, weights);
        EvaluateFace(simplex, 1, 3, 2, ref best, ref bestDistanceSqr, bestIndices, bestWeights, face, weights);

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
        ref Fixed64 bestDistanceSqr,
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
        if (candidate.DistanceSqr >= bestDistanceSqr)
            return;

        best = candidate;
        bestDistanceSqr = candidate.DistanceSqr;
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

    private static TriangleWeights ClosestPointOnTriangleToOrigin(Vector3d a, Vector3d b, Vector3d c)
    {
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

    private static bool IsOriginInsideTetrahedron(Vector3d a, Vector3d b, Vector3d c, Vector3d d)
    {
        if (SignedTetrahedronVolume6(a, b, c, d).Abs() <= DistanceTolerance)
            return false;

        return IsSameSideOfFace(Vector3d.Zero, d, a, b, c)
            && IsSameSideOfFace(Vector3d.Zero, c, a, d, b)
            && IsSameSideOfFace(Vector3d.Zero, b, a, c, d)
            && IsSameSideOfFace(Vector3d.Zero, a, b, d, c);
    }

    private static bool IsSameSideOfFace(Vector3d point, Vector3d opposite, Vector3d a, Vector3d b, Vector3d c)
    {
        Vector3d normal = Vector3d.Cross(b - a, c - a);
        if (normal.MagnitudeSquared <= DistanceToleranceSqr)
            return false;

        Fixed64 pointSide = Vector3d.Dot(normal, point - a);
        Fixed64 oppositeSide = Vector3d.Dot(normal, opposite - a);
        return pointSide * oppositeSide >= -DistanceTolerance;
    }

    private static Fixed64 SignedTetrahedronVolume6(Vector3d a, Vector3d b, Vector3d c, Vector3d d) =>
        Vector3d.Dot(b - a, Vector3d.Cross(c - a, d - a));

    private static bool ContainsSupportPoint(SupportPoint[] simplex, int count, Vector3d point)
    {
        for (int i = 0; i < count; i++)
        {
            if ((simplex[i].Point - point).MagnitudeSquared <= DistanceToleranceSqr)
                return true;
        }

        return false;
    }

    private static ConvexShape CreateColliderShape(LSCollider collider, Vector3d offset)
    {
        if (collider is LSMeshCollider mesh && mesh.Mode == MeshColliderMode.Concave)
            throw CreateConcaveSourceException(mesh);

        return new ConvexShape(collider, offset);
    }

    private static ConvexShape CreateTriangleShape(LSMeshCollider mesh, int triangleIndex)
    {
        mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
        return new ConvexShape(first, second, third);
    }

    private static bool IsSupportedSourcePart(LSCollider part)
    {
        return part is LSSphereCollider
            || part is LSCapsuleCollider
            || part is LSCuboidCollider
            || part is LSCylinderCollider
            || part is LSMeshCollider { Mode: MeshColliderMode.Convex };
    }

    private static void ThrowIfConcaveSource(LSMeshCollider source)
    {
        if (source.Mode == MeshColliderMode.Concave)
            throw CreateConcaveSourceException(source);
    }

    private void CreateSweptSourceBounds(ConvexShape sourceShape, out Vector3d min, out Vector3d max)
    {
        sourceShape.GetBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        min = Vector3d.Min(sourceMin, sourceMin + _displacement);
        max = Vector3d.Max(sourceMax, sourceMax + _displacement);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ComesBefore(Physics3DHit hit, bool found, Fixed64 closestDistance, Physics3DHit closestHit)
    {
        if (!found)
            return true;

        int distanceCompare = hit.Distance.CompareTo(closestDistance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        return (hit.Collider?.Id ?? -1) < (closestHit.Collider?.Id ?? -1);
    }

    private static ArgumentException CreateConcaveSourceException(LSMeshCollider source) =>
        new("Concave mesh sources are not supported by swept query APIs. Use an LSCompoundCollider built from authored convex decomposition parts.", nameof(source));

    private readonly struct ConvexShape
    {
        private readonly LSCollider? _collider;
        private readonly Vector3d _offset;
        private readonly Vector3d _triangleA;
        private readonly Vector3d _triangleB;
        private readonly Vector3d _triangleC;
        private readonly bool _isTriangle;

        public ConvexShape(LSCollider collider, Vector3d offset)
        {
            _collider = collider;
            _offset = offset;
            _triangleA = Vector3d.Zero;
            _triangleB = Vector3d.Zero;
            _triangleC = Vector3d.Zero;
            _isTriangle = false;
        }

        public ConvexShape(Vector3d triangleA, Vector3d triangleB, Vector3d triangleC)
        {
            _collider = null;
            _offset = Vector3d.Zero;
            _triangleA = triangleA;
            _triangleB = triangleB;
            _triangleC = triangleC;
            _isTriangle = true;
        }

        public Vector3d Center => _isTriangle
            ? (_triangleA + _triangleB + _triangleC) / (Fixed64)3
            : _collider!.Center + _offset;

        public bool IsTriangle => _isTriangle;

        public void GetBounds(out Vector3d min, out Vector3d max)
        {
            if (!_isTriangle)
            {
                min = _collider!.Bounds.Min + _offset;
                max = _collider.Bounds.Max + _offset;
                return;
            }

            min = Vector3d.Min(Vector3d.Min(_triangleA, _triangleB), _triangleC);
            max = Vector3d.Max(Vector3d.Max(_triangleA, _triangleB), _triangleC);
        }

        public ConvexShape WithOffset(Vector3d additionalOffset) =>
            _isTriangle
                ? this
                : new ConvexShape(_collider!, _offset + additionalOffset);

        public Vector3d Support(Vector3d direction)
        {
            if (_isTriangle)
                return SupportTriangle(direction);

            return SupportCollider(_collider!, direction) + _offset;
        }

        private Vector3d SupportTriangle(Vector3d direction)
        {
            Vector3d best = _triangleA;
            Fixed64 bestProjection = Vector3d.Dot(best, direction);
            Fixed64 projection = Vector3d.Dot(_triangleB, direction);
            if (projection > bestProjection)
            {
                best = _triangleB;
                bestProjection = projection;
            }

            projection = Vector3d.Dot(_triangleC, direction);
            if (projection > bestProjection)
                best = _triangleC;

            return best;
        }

        private static Vector3d SupportCollider(LSCollider collider, Vector3d direction)
        {
            Vector3d normal = direction.MagnitudeSquared > Fixed64.Epsilon
                ? direction.Normalized
                : Vector3d.Right;

            return collider switch
            {
                LSSphereCollider sphere => sphere.Center + normal * sphere.ScaledRadius,
                LSCapsuleCollider capsule => SupportCapsule(capsule, normal),
                LSCuboidCollider cuboid => SupportVertices(cuboid.Vertices, normal),
                LSCylinderCollider cylinder => SupportCylinder(cylinder, normal),
                LSMeshCollider mesh => SupportMesh(mesh, normal),
                _ => throw new NotSupportedException(
                    $"Convex swept queries do not support {collider.GetType().Name}.")
            };
        }

        private static Vector3d SupportCapsule(LSCapsuleCollider capsule, Vector3d direction)
        {
            Fixed64 startProjection = Vector3d.Dot(capsule.LineSegmentStart, direction);
            Fixed64 endProjection = Vector3d.Dot(capsule.LineSegmentEnd, direction);
            Vector3d segmentPoint = endProjection > startProjection
                ? capsule.LineSegmentEnd
                : capsule.LineSegmentStart;
            return segmentPoint + direction * capsule.ScaledRadius;
        }

        private static Vector3d SupportCylinder(LSCylinderCollider cylinder, Vector3d direction)
        {
            FixedQuaternion inverse = cylinder.Rotation.Inverse();
            Vector3d localDirection = inverse * direction;
            Vector3d radial = new(localDirection.X, Fixed64.Zero, localDirection.Z);
            Fixed64 radialMagnitude = radial.Magnitude;
            Vector3d radialSupport = radialMagnitude > Fixed64.Epsilon
                ? radial / radialMagnitude * cylinder.ScaledRadius
                : Vector3d.Right * cylinder.ScaledRadius;
            Fixed64 y = localDirection.Y >= Fixed64.Zero ? cylinder.HalfHeight : -cylinder.HalfHeight;
            return cylinder.Center + cylinder.Rotation * new Vector3d(radialSupport.X, y, radialSupport.Z);
        }

        private static Vector3d SupportMesh(LSMeshCollider mesh, Vector3d direction)
        {
            Vector3d best = mesh.Mesh.GetVertexWorld(0);
            Fixed64 bestProjection = Vector3d.Dot(best, direction);
            for (int i = 1; i < mesh.Mesh.VertexCount; i++)
            {
                Vector3d vertex = mesh.Mesh.GetVertexWorld(i);
                Fixed64 projection = Vector3d.Dot(vertex, direction);
                if (projection <= bestProjection)
                    continue;

                bestProjection = projection;
                best = vertex;
            }

            return best;
        }

        private static Vector3d SupportVertices(Vector3d[] vertices, Vector3d direction)
        {
            Vector3d best = vertices[0];
            Fixed64 bestProjection = Vector3d.Dot(best, direction);
            for (int i = 1; i < vertices.Length; i++)
            {
                Fixed64 projection = Vector3d.Dot(vertices[i], direction);
                if (projection <= bestProjection)
                    continue;

                bestProjection = projection;
                best = vertices[i];
            }

            return best;
        }
    }

    private readonly struct SupportPoint
    {
        public SupportPoint(Vector3d pointA, Vector3d pointB)
        {
            PointA = pointA;
            PointB = pointB;
            Point = pointA - pointB;
        }

        public Vector3d PointA { get; }

        public Vector3d PointB { get; }

        public Vector3d Point { get; }
    }

    private readonly struct GjkResult
    {
        public GjkResult(bool intersects, Fixed64 distance, Vector3d pointA, Vector3d pointB, Vector3d normal)
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

    private readonly struct TriangleWeights
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
