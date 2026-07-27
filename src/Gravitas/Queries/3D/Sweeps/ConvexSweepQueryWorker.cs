//=======================================================================
// ConvexSweepQueryWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;
using System;
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
    private static readonly Fixed64 DistanceTolerance =
        Fixed64.FromFraction(1, 1_048_576);
    internal static readonly Fixed64 ContactTolerance = Fixed64.FromFraction(1, 4096);
    private static readonly SweepTriangleCandidateComparer SweepTriangleComparer = new();
    private static readonly FixedPointAnchor ZeroAnchor =
        new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

    private readonly SupportPoint[] _simplex = new SupportPoint[4];
    private readonly SwiftList<int> _triangleCandidates = new(16);
    private readonly SwiftList<SweepTriangleCandidate> _sweepTriangleCandidates = new(16);
    private readonly int _maxConservativeAdvancementIterations;

    private LSCollider? _source;
    private ConvexShape _sourceShape;
    private bool _hasSource;
    private Vector3d _displacement;
    private Vector3d _outputDirection;
    private Vector3d _sweptSourceBoundsMin;
    private Vector3d _sweptSourceBoundsMax;
    private FixedPointAnchor _displacementAnchor;
    private FixedSegment _chord;
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

    internal void PrepareSphereSource(
        Vector3d center,
        Fixed64 radius,
        Vector3d displacement)
    {
        SwiftThrowHelper.ThrowIfArgument(
            radius < Fixed64.Zero,
            nameof(radius),
            "Sphere sweep radius cannot be negative.");
        _source = null;
        Prepare(ConvexShape.CreateSphere(center, radius), displacement);
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

        return TrySweepSourceShape(
            _sourceShape,
            target,
            out hit,
            out _);
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
        _displacementAnchor = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            displacement);
        _chord = new FixedSegment(Vector3d.Zero, displacement);
        _hasSource =
            Vector3d.TryGetMagnitude(displacement, out _length)
            && sourceShape.CanTranslateCenter(displacement);
        if (!_hasSource)
        {
            _outputDirection = Vector3d.Zero;
            return;
        }

        _outputDirection =
            _length <= Fixed64.Epsilon
                ? Vector3d.Zero
                : displacement.Normalized;
        sourceShape.GetSourceBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        // Bounds are broad-phase clips, not canonical pose coordinates. A
        // valid center may have support beyond a scalar face.
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
        Fixed64 closestNumerator = Fixed64.MaxValue;
        int closestPartIndex = int.MaxValue;

        for (int i = 0; i < source.PartCount; i++)
        {
            LSCollider part = source.GetPartCollider(i);
            if (!TrySweepSourceShape(
                    CreateColliderShape(part, Vector3d.Zero),
                    target,
                    out Physics3DHit candidate,
                    out Fixed64 candidateNumerator)
                || !ComesBeforeReducerCandidate(
                    candidateNumerator,
                    i,
                    found,
                    closestNumerator,
                    closestPartIndex))
            {
                continue;
            }

            hit = candidate;
            closestNumerator = candidateNumerator;
            closestPartIndex = i;
            found = true;
        }

        return found;
    }

    private bool TrySweepSourceShape(
        ConvexShape sourceShape,
        LSCollider target,
        out Physics3DHit hit,
        out Fixed64 hitNumerator)
    {
        hit = default;
        hitNumerator = default;

        if (!CanSweptSourceShapeReachTarget(sourceShape, target))
            return false;

        if (target is LSCompoundCollider compound)
        {
            return TrySweepTargetCompound(
                sourceShape,
                compound,
                out hit,
                out hitNumerator);
        }

        if (target is LSMeshCollider mesh && mesh.Mode == MeshColliderMode.Concave)
        {
            return TrySweepConcaveMeshTarget(
                sourceShape,
                mesh,
                out hit,
                out hitNumerator);
        }

        return TrySweepConvexTarget(
            sourceShape,
            CreateColliderShape(target, Vector3d.Zero),
            target,
            out hit,
            out hitNumerator);
    }

    private bool TrySweepTargetCompound(
        ConvexShape sourceShape,
        LSCompoundCollider compound,
        out Physics3DHit hit,
        out Fixed64 hitNumerator)
    {
        hit = default;
        hitNumerator = default;
        bool found = false;
        Fixed64 closestNumerator = Fixed64.MaxValue;
        int closestPartIndex = int.MaxValue;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!TrySweepSourceShape(
                    sourceShape,
                    part,
                    out Physics3DHit partHit,
                    out Fixed64 partNumerator)
                || !ComesBeforeReducerCandidate(
                    partNumerator,
                    i,
                    found,
                    closestNumerator,
                    closestPartIndex))
            {
                continue;
            }

            hit = new Physics3DHit(compound, partHit.Anchor, partHit.Normal, partHit.Distance, partHit.Direction);
            hitNumerator = partNumerator;
            closestNumerator = partNumerator;
            closestPartIndex = i;
            found = true;
        }

        return found;
    }

    private bool TrySweepConcaveMeshTarget(
        ConvexShape sourceShape,
        LSMeshCollider mesh,
        out Physics3DHit hit,
        out Fixed64 hitNumerator)
    {
        hit = default;
        hitNumerator = default;
        bool found = false;
        Fixed64 closestNumerator = Fixed64.MaxValue;
        int closestTriangleIndex = int.MaxValue;

        if (!TryCreateSweptSourceBoundsInMeshFrame(
                sourceShape,
                mesh,
                out Vector3d min,
                out Vector3d max))
        {
            return false;
        }

        mesh.Mesh.GetTrianglesInLocalBounds(
            new FixedBoundVolume(min, max),
            _triangleCandidates);
        LastMeshTriangleCandidateCount += _triangleCandidates.Count;
        BuildOrderedSweepTriangleCandidates(sourceShape, mesh);

        for (int i = 0; i < _sweepTriangleCandidates.Count; i++)
        {
            SweepTriangleCandidate sweepCandidate = _sweepTriangleCandidates[i];
            if (RemainingSweepTrianglesCannotBeat(
                sweepCandidate.LowerBoundNumerator,
                found,
                closestNumerator))
            {
                break;
            }

            int triangleIndex = sweepCandidate.TriangleIndex;
            ConvexShape triangle = CreateTriangleShape(mesh, triangleIndex);
            if (!TrySweepConvexTarget(
                    sourceShape,
                    triangle,
                    mesh,
                    out Physics3DHit candidate,
                    out Fixed64 candidateNumerator)
                || !ComesBeforeReducerCandidate(
                    candidateNumerator,
                    triangleIndex,
                    found,
                    closestNumerator,
                    closestTriangleIndex))
            {
                continue;
            }

            hit = candidate;
            hitNumerator = candidateNumerator;
            closestNumerator = candidateNumerator;
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
            if (TryComputeSweepLowerBoundNumerator(
                    sourceShape,
                    triangle,
                    out Fixed64 lowerBoundNumerator))
            {
                _sweepTriangleCandidates.Add(
                    new SweepTriangleCandidate(
                        triangleIndex,
                        lowerBoundNumerator));
            }
        }

        _sweepTriangleCandidates.SortInPlace(SweepTriangleComparer);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool TryComputeSweepLowerBoundNumerator(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        out Fixed64 lowerBoundNumerator)
    {
        sourceShape.GetSourceBounds(
            out Vector3d sourceMin,
            out Vector3d sourceMax);
        targetShape.GetBounds(
            out Vector3d targetMin,
            out Vector3d targetMax);

        Vector3d padding = Vector3d.One * ContactTolerance;
        sourceMin -= padding;
        sourceMax += padding;
        lowerBoundNumerator = Fixed64.Zero;
        return IncludeAxisEntryNumerator(
                sourceMin.X,
                sourceMax.X,
                targetMin.X,
                targetMax.X,
                _displacement.X,
                ref lowerBoundNumerator)
            && IncludeAxisEntryNumerator(
                sourceMin.Y,
                sourceMax.Y,
                targetMin.Y,
                targetMax.Y,
                _displacement.Y,
                ref lowerBoundNumerator)
            && IncludeAxisEntryNumerator(
                sourceMin.Z,
                sourceMax.Z,
                targetMin.Z,
                targetMax.Z,
                _displacement.Z,
                ref lowerBoundNumerator);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IncludeAxisEntryNumerator(
        Fixed64 sourceMin,
        Fixed64 sourceMax,
        Fixed64 targetMin,
        Fixed64 targetMax,
        Fixed64 displacement,
        ref Fixed64 entryNumerator)
    {
        if (sourceMax >= targetMin
            && sourceMin <= targetMax)
        {
            return true;
        }

        Fixed64 gap;
        Fixed64 span;
        if (sourceMax < targetMin
            && displacement > Fixed64.Zero)
        {
            // Saturation is safe here: an unrepresentable positive gap is
            // necessarily farther than the representable sweep span.
            gap = targetMin - sourceMax;
            span = displacement;
        }
        else if (sourceMin > targetMax
            && displacement < Fixed64.Zero)
        {
            gap = sourceMin - targetMax;
            // Prepare admits only representable chord magnitudes, so no
            // component can be Fixed64.MinValue here.
            span = -displacement;
        }
        else
        {
            return false;
        }

        if (gap > span)
            return false;

        // With 0 <= gap <= span, the fused result is bounded by the already
        // representable chord length.
        _ = Fixed64.TryMultiplyDivide(
            _length,
            gap,
            span,
            out Fixed64 axisNumerator);

        // Fused conversion keeps u = numerator / chord length exact until the
        // final Q32.32 rounding. One raw unit makes that rounded result a
        // conservative lower bound without moving an underflowed zero.
        axisNumerator = FixedMath.Max(
            Fixed64.Zero,
            axisNumerator - Fixed64.Epsilon);

        if (axisNumerator > entryNumerator)
            entryNumerator = axisNumerator;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool RemainingSweepTrianglesCannotBeat(
        Fixed64 candidateLowerBoundNumerator,
        bool found,
        Fixed64 closestNumerator)
    {
        if (!found)
            return false;

        return candidateLowerBoundNumerator
            > closestNumerator + Fixed64.Epsilon;
    }

    private bool TrySweepConvexTarget(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        LSCollider targetCollider,
        out Physics3DHit hit,
        out Fixed64 hitNumerator)
    {
        hit = default;
        hitNumerator = default;
        // u is retained as travelNumerator / _length. Keeping the common
        // denominator exact preserves small chord components and physical
        // distance resolution without exposing wide arithmetic.
        Fixed64 travelNumerator = Fixed64.Zero;
        Vector3d normal = Vector3d.Zero;
        GjkResult result = default;

        for (int i = 0; i < _maxConservativeAdvancementIterations; i++)
        {
            ConvexShape movedSource =
                sourceShape.WithSourceOffset(
                    GetChordOffset(travelNumerator));
            result = ComputeDistance(movedSource, targetShape);
            if (result.Intersects || result.Distance <= ContactTolerance)
            {
                ContactAnchor anchor = ResolveHitAnchor(
                    targetShape,
                    targetCollider,
                    movedSource,
                    result,
                    out Vector3d point,
                    out bool hasMaterializedPoint,
                    out bool hasRefinedSurfaceNormal);
                Vector3d hitNormal = ResolveHitNormal(
                    targetShape,
                    targetCollider,
                    point,
                    result.Normal,
                    normal,
                    hasRefinedSurfaceNormal,
                    hasMaterializedPoint);

                hit = new Physics3DHit(
                    targetCollider,
                    anchor,
                    hitNormal,
                    travelNumerator,
                    _outputDirection);
                hitNumerator = travelNumerator;
                return true;
            }

            normal = result.Normal;
            // Projecting a representable chord onto a unit normal is bounded
            // by the admitted chord length.
            _ = _displacementAnchor.TryGetProjectedOffsetFrom(
                ZeroAnchor,
                -normal,
                out Fixed64 closingPerFraction);
            if (closingPerFraction <= Fixed64.Epsilon)
            {
                return TryResolveEndpointBracket(
                    sourceShape,
                    targetShape,
                    targetCollider,
                    travelNumerator,
                    out hit,
                    out hitNumerator);
            }
            bool hasStep = Fixed64.TryMultiplyDivide(
                result.Distance,
                _length,
                closingPerFraction,
                out Fixed64 stepNumerator);
            Fixed64 nextTravelNumerator =
                travelNumerator + stepNumerator;
            if (!hasStep || nextTravelNumerator > _length)
            {
                ConvexShape endpointSource = sourceShape.WithSourceOffset(_displacement);
                GjkResult endpointResult = ComputeDistance(endpointSource, targetShape);
                if (!endpointResult.Intersects && endpointResult.Distance > ContactTolerance)
                    return false;

                ContactAnchor anchor = ResolveHitAnchor(
                    targetShape,
                    targetCollider,
                    endpointSource,
                    endpointResult,
                    out Vector3d point,
                    out bool hasMaterializedPoint,
                    out bool hasRefinedSurfaceNormal);
                Vector3d hitNormal = ResolveHitNormal(
                    targetShape,
                    targetCollider,
                    point,
                    endpointResult.Normal,
                    normal,
                    hasRefinedSurfaceNormal,
                    hasMaterializedPoint);
                hit = new Physics3DHit(
                    targetCollider,
                    anchor,
                    hitNormal,
                    _length,
                    _outputDirection);
                hitNumerator = _length;
                return true;
            }

            travelNumerator = nextTravelNumerator;
        }

        return TryResolveEndpointBracket(
            sourceShape,
            targetShape,
            targetCollider,
            travelNumerator,
            out hit,
            out hitNumerator);
    }

    private bool TryResolveEndpointBracket(
        ConvexShape sourceShape,
        ConvexShape targetShape,
        LSCollider targetCollider,
        Fixed64 lowerNumerator,
        out Physics3DHit hit,
        out Fixed64 hitNumerator)
    {
        hit = default;
        hitNumerator = default;
        Fixed64 upperNumerator = _length;
        ConvexShape upperSource =
            sourceShape.WithSourceOffset(_displacement);
        GjkResult upperResult =
            ComputeDistance(upperSource, targetShape);
        if (!upperResult.Intersects
            && upperResult.Distance > ContactTolerance)
        {
            return false;
        }

        for (int iteration = 0;
            iteration < _maxConservativeAdvancementIterations
            && upperNumerator - lowerNumerator > DistanceTolerance;
            iteration++)
        {
            Fixed64 middleNumerator =
                FixedMath.Midpoint(
                    lowerNumerator,
                    upperNumerator);
            ConvexShape middleSource =
                sourceShape.WithSourceOffset(
                    GetChordOffset(middleNumerator));
            GjkResult middleResult =
                ComputeDistance(middleSource, targetShape);
            if (middleResult.Intersects
                || middleResult.Distance <= ContactTolerance)
            {
                upperNumerator = middleNumerator;
                upperSource = middleSource;
                upperResult = middleResult;
            }
            else
            {
                lowerNumerator = middleNumerator;
            }
        }

        ContactAnchor anchor = ResolveHitAnchor(
            targetShape,
            targetCollider,
            upperSource,
            upperResult,
            out Vector3d point,
            out bool hasMaterializedPoint,
            out bool hasRefinedSurfaceNormal);
        Vector3d hitNormal = ResolveHitNormal(
            targetShape,
            targetCollider,
            point,
            upperResult.Normal,
            Vector3d.Zero,
            hasRefinedSurfaceNormal,
            hasMaterializedPoint);
        hit = new Physics3DHit(
            targetCollider,
            anchor,
            hitNormal,
            upperNumerator,
            _outputDirection);
        hitNumerator = upperNumerator;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector3d GetChordOffset(Fixed64 numerator) =>
        _chord.GetPointAtDistance(
            numerator,
            _length);

    private ContactAnchor ResolveHitAnchor(
        ConvexShape targetShape,
        LSCollider targetCollider,
        ConvexShape movedSource,
        GjkResult result,
        out Vector3d point,
        out bool hasMaterializedPoint,
        out bool hasRefinedSurfaceNormal)
    {
        hasMaterializedPoint = true;
        hasRefinedSurfaceNormal = false;
        if (targetCollider is LSSphereCollider
            && movedSource.TryGetClosestPointOnSurface(
                targetCollider.Center,
                out Vector3d sourcePoint)
            && Vector3d.TrySubtract(
                sourcePoint,
                targetCollider.Center,
                out Vector3d centerToSource)
            && centerToSource.MagnitudeSquared > Fixed64.Epsilon)
        {
            // A sphere's closest pair is defined by its center and the closest
            // source feature. Refining that feature removes arbitrary support
            // tie bias without changing the conservative TOI.
            FixedPointAnchor sphereAnchor =
                ConvexColliderSupport.GetSupportAnchor(
                    targetCollider,
                    centerToSource,
                    Vector3d.Zero);
            hasMaterializedPoint = sphereAnchor.TryGetPoint(out point);
            hasRefinedSurfaceNormal = hasMaterializedPoint;
            return new ContactAnchor(sphereAnchor);
        }

        FixedPointAnchor movedSourceCenter = movedSource.GetCenterAnchor();
        FixedPointAnchor targetCenter = targetShape.GetCenterAnchor();
        if (movedSourceCenter.TryGetOffsetFrom(
                targetCenter,
                out Vector3d centerDifference)
            && centerDifference.MagnitudeSquared <= Fixed64.Epsilon)
        {
            FixedPointAnchor fallbackAnchor =
                targetShape.GetFallbackSurfaceAnchor(-_displacement);
            hasMaterializedPoint = fallbackAnchor.TryGetPoint(out point);
            return new ContactAnchor(fallbackAnchor);
        }

        // GJK's target witness identifies the feature that stopped the sweep.
        // Center-to-center projection can select an unrelated feature for long,
        // offset shapes and therefore produce a non-physical response normal.
        if (!result.PointB.TryGetPoint(out point))
        {
            point = default;
            hasMaterializedPoint = false;
            return new ContactAnchor(result.PointB);
        }

        return new ContactAnchor(result.PointB);
    }

    private Vector3d ResolveHitNormal(
        ConvexShape targetShape,
        LSCollider targetCollider,
        Vector3d point,
        Vector3d resultNormal,
        Vector3d fallbackNormal,
        bool hasRefinedSurfaceNormal,
        bool hasMaterializedPoint)
    {
        Vector3d planarNormal = Vector3d.Zero;
        if (hasMaterializedPoint)
            targetShape.TryGetPlanarSurfaceNormal(point, out planarNormal);
        return ConvexSweepHitPolicy.ResolveHitNormal(
            targetCollider,
            point,
            resultNormal,
            fallbackNormal,
            _displacement,
            planarNormal,
            hasRefinedSurfaceNormal,
            hasMaterializedPoint);
    }

    private static ConvexShape CreateColliderShape(LSCollider collider, Vector3d offset)
    {
        return new ConvexShape(collider, offset);
    }

    private static ConvexShape CreateTriangleShape(LSMeshCollider mesh, int triangleIndex)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        return new ConvexShape(mesh, triangleIndex, first, second, third);
    }

    private static void ThrowIfConcaveSource(LSMeshCollider source)
    {
        if (source.Mode == MeshColliderMode.Concave)
            throw CreateConcaveSourceException(source);
    }

    private bool TryCreateSweptSourceBoundsInMeshFrame(
        ConvexShape sourceShape,
        LSMeshCollider mesh,
        out Vector3d min,
        out Vector3d max)
    {
        if (!sourceShape.TryGetBoundsRelativeTo(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                out Vector3d sourceMin,
                out Vector3d sourceMax))
        {
            min = default;
            max = default;
            return false;
        }

        // Prepare admitted the chord magnitude, so a unit rotation preserves
        // representability.
        _ = mesh.Mesh.Rotation.Inverse().TryRotate(
            _displacement,
            out Vector3d localDisplacement);
        SweepBoundsUtility.CreateSweptBounds(
            sourceMin,
            sourceMax,
            localDisplacement,
            ContactTolerance,
            out min,
            out max);
        return true;
    }

    private bool CanSweptSourceShapeReachTarget(ConvexShape sourceShape, LSCollider target)
    {
        if (!sourceShape.CanTranslateCenter(_displacement))
            return false;

        sourceShape.GetSourceBounds(out Vector3d sourceMin, out Vector3d sourceMax);
        SweepBoundsUtility.CreateSweptBounds(
            sourceMin,
            sourceMax,
            _displacement,
            ContactTolerance,
            out Vector3d min,
            out Vector3d max);
        return SweepBoundsUtility.OverlapsInclusive(
            min,
            max,
            target.BoundsMin,
            target.BoundsMax);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ComesBeforeReducerCandidate(
        Fixed64 hitNumerator,
        int candidateOrdinal,
        bool found,
        Fixed64 closestNumerator,
        int closestOrdinal)
    {
        if (!found)
            return true;

        int numeratorCompare =
            hitNumerator.CompareTo(closestNumerator);
        if (numeratorCompare != 0)
            return numeratorCompare < 0;

        return candidateOrdinal < closestOrdinal;
    }

    private static ArgumentException CreateConcaveSourceException(LSMeshCollider source) =>
        new("Concave mesh sources are not supported by swept query APIs. Use an LSCompoundCollider built from authored convex decomposition parts.", nameof(source));
}
