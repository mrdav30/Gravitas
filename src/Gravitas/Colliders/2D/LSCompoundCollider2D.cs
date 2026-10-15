//=======================================================================
// LSCompoundCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Represents one pure 2D collider identity whose collision shape is composed
/// from deterministic internal primitive and convex-polygon parts.
/// </summary>
public sealed class LSCompoundCollider2D : LSCollider2D
{
    private readonly CompoundColliderPart2D[] _parts;
    private readonly LSCollider2D[] _partColliders;
    private readonly ExactMassPoint2D[] _massPointScratch;
    private readonly ExactMassWeight[] _massWeightScratch;
    private readonly ContactManifold2D _partManifoldScratch = new();
    private bool _hasMassPropertyCenter;
    private Vector2d _massPropertyOwnerScale;
    private Vector2d _massPropertyOffset;
    private bool _massPropertyCommitted;
    private Vector2d _massPropertyCenter;
    private bool _hasMomentGeometry;
    private Vector2d _momentReferencePoint;
    private Signed576 _momentGeometry;
    private Signed704 _momentDenominator;

    /// <summary>Creates a runtime compound collider from authored pure 2D parts.</summary>
    public LSCompoundCollider2D(params CompoundColliderPart2D[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "2D compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
            ValidatePart(parts[i]);

        _parts = new CompoundColliderPart2D[parts.Length];
        _partColliders = new LSCollider2D[parts.Length];
        _massPointScratch = new ExactMassPoint2D[parts.Length];
        _massWeightScratch = new ExactMassWeight[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            _parts[i] = parts[i];
            _partColliders[i] = MaterializePartCollider(parts[i]);
            _partColliders[i].ReserveCompoundPart(this, parts[i].LocalRotation, parts[i].LocalScale);
        }
    }

    /// <inheritdoc/>
    public override ColliderType2D Shape => ColliderType2D.Compound;

    /// <inheritdoc/>
    public override int Priority => ColliderSettings2D.GetPriority(Shape);

    /// <summary>
    /// Gets the radius of a circle that conservatively contains the current
    /// aggregate shape.
    /// </summary>
    public Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            return HasCommittedShape
                ? CanonicalCenteredProxyRadius
                : ColliderCanonicalBounds2D.GetCurrentCompoundCenteredProxyRadius(this);
        }
    }

    /// <summary>Gets the authored parts in stable source order.</summary>
    public ReadOnlySpan<CompoundColliderPart2D> Parts => _parts;

    /// <summary>Gets the number of authored compound parts.</summary>
    public int PartCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _parts.Length;
    }

    /// <summary>Gets the stable runtime part identifier for a source-order index.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetPartId(int index)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return index;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal LSCollider2D GetPartCollider(int index)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return _partColliders[index];
    }

    internal ContactManifold2D PartManifoldScratch => _partManifoldScratch;

    /// <inheritdoc/>
    public override bool ContainsPoint(Vector2d point)
    {
        for (int i = 0; i < _partColliders.Length; i++)
        {
            if (_partColliders[i].ContainsPoint(point))
                return true;
        }

        return false;
    }

    /// <inheritdoc/>
    public override Vector2d GetClosestPoint(Vector2d point)
    {
        _ = TryGetClosestBoundaryAnchor(
            point,
            out FixedPointAnchor2d anchor);
        if (anchor.TryGetPoint(out Vector2d closest))
        {
            return closest;
        }

        throw new InvalidOperationException(
            "The closest compound surface point is outside the Fixed64 coordinate domain.");
    }

    /// <inheritdoc/>
    public override Vector2d GetSupportPoint(Vector2d direction)
    {
        Vector2d bestPoint = _partColliders[0].GetSupportPoint(direction);
        Fixed64 bestProjection = Vector2d.Dot(bestPoint, direction);
        for (int i = 1; i < _partColliders.Length; i++)
        {
            Vector2d candidate = _partColliders[i].GetSupportPoint(direction);
            Fixed64 projection = Vector2d.Dot(candidate, direction);
            if (projection <= bestProjection)
                continue;

            bestProjection = projection;
            bestPoint = candidate;
        }

        return bestPoint;
    }

    internal override ExactMassPoint2D CalculateLocalMassPoint()
    {
        GetCurrentScaleFactors(out Vector2d ownerScale, out _);
        return _hasMassPropertyCenter
            && ownerScale == _massPropertyOwnerScale
            && LocalOffset == _massPropertyOffset
            && HasCommittedShape == _massPropertyCommitted
            ? ExactMassPoint2D.FromPoint(_massPropertyCenter)
            : CalculateAggregateMassPoint(usePrepared: false);
    }

    internal override ExactMassPoint2D CalculatePreparedLocalMassPoint() =>
        CalculateAggregateMassPoint(usePrepared: true);

    private ExactMassPoint2D CalculateAggregateMassPoint(bool usePrepared)
    {
        bool hasPositiveWeight = false;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D partCollider = _partColliders[i];
            ExactMassWeight weight = usePrepared
                ? partCollider.CalculatePreparedAreaForMassProperties()
                : partCollider.CalculateAreaForMassProperties();
            _massPointScratch[i] = usePrepared
                ? partCollider.CalculatePreparedLocalMassPoint()
                : partCollider.CalculateLocalMassPoint();
            _massWeightScratch[i] = weight;
            hasPositiveWeight |= !weight.IsZero;
        }

        if (!hasPositiveWeight)
        {
            for (int i = 0; i < _massWeightScratch.Length; i++)
                _massWeightScratch[i] = ExactMassWeight.One;
        }

        if (!ExactMassPoint2D.TryGetWeightedAverage(
            _massPointScratch,
            _massWeightScratch,
            out Vector2d center))
        {
            throw new InvalidOperationException(
                usePrepared
                    ? "Prepared 2D compound mass-property point is outside the Fixed64 coordinate domain."
                    : "The 2D compound collider's center of mass is outside the Fixed64 coordinate domain.");
        }
        return ExactMassPoint2D.FromPoint(center);
    }

    internal override ExactMassWeight CalculateAreaForMassProperties() =>
        CalculateAggregateMassWeight(usePrepared: false);

    internal override ExactMassWeight CalculatePreparedAreaForMassProperties() =>
        CalculateAggregateMassWeight(usePrepared: true);

    private ExactMassWeight CalculateAggregateMassWeight(bool usePrepared)
    {
        ExactMassWeight totalWeight = ExactMassWeight.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            ExactMassWeight weight = usePrepared
                ? _partColliders[i].CalculatePreparedAreaForMassProperties()
                : _partColliders[i].CalculateAreaForMassProperties();
            totalWeight = totalWeight.Add(weight);
        }

        return totalWeight;
    }

    internal override Fixed64 CalculateCenterOfMassMoment(Fixed64 mass)
    {
        EnsureMassPropertyCenter();
        return CalculateMomentAboutReference(mass, _massPropertyCenter);
    }

    /// <inheritdoc/>
    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;
        EnsureMassPropertyCenter();
        return localReferencePoint == _massPropertyCenter
            ? CalculateCenterOfMassMoment(mass)
            : CalculateMomentAboutReference(mass, localReferencePoint);
    }

    private void EnsureMassPropertyCenter()
    {
        GetCurrentScaleFactors(out Vector2d ownerScale, out _);
        if (_hasMassPropertyCenter && ownerScale == _massPropertyOwnerScale
            && LocalOffset == _massPropertyOffset && HasCommittedShape == _massPropertyCommitted)
            return;

        _hasMassPropertyCenter = false;
        _massPropertyCenter = CalculateLocalCenterOfMassOffset();
        _massPropertyOwnerScale = ownerScale;
        _massPropertyOffset = LocalOffset;
        _massPropertyCommitted = HasCommittedShape;
        _hasMomentGeometry = false;
        _hasMassPropertyCenter = true;
    }

    private Fixed64 CalculateMomentAboutReference(Fixed64 mass, Vector2d referencePoint)
    {
        if (!_hasMomentGeometry || referencePoint != _momentReferencePoint)
            CacheMomentGeometry(referencePoint);

        Signed704 numerator = WideArithmetic.MultiplySigned576ToSigned704(
            _momentGeometry, Signed320.ExtendValue(Signed192.Raw(mass)));
        if (!Fixed64.TryGetSignedRawRatio(numerator, _momentDenominator, out Fixed64 moment))
        {
            throw new InvalidOperationException(
                "The 2D compound collider's moment of inertia is outside the Fixed64 scalar domain.");
        }
        return moment;
    }

    private void CacheMomentGeometry(Vector2d referencePoint)
    {
        ExactMassWeight totalWeight = ExactMassWeight.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            ExactMassWeight weight = _partColliders[i].CalculateAreaForMassProperties();
            _massWeightScratch[i] = weight;
            totalWeight = totalWeight.Add(weight);
        }

        if (totalWeight.IsZero)
        {
            for (int i = 0; i < _partColliders.Length; i++)
            {
                _massWeightScratch[i] = ExactMassWeight.One;
                totalWeight = totalWeight.Add(ExactMassWeight.One);
            }
        }

        ExactMassPoint2D reference = ExactMassPoint2D.FromPoint(referencePoint);
        Signed576 geometry = default;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D part = _partColliders[i];
            ExactMassWeight weight = _massWeightScratch[i];
            if (weight.IsZero)
                continue;
            ExactMassPoint2D point = part.CalculateLocalMassPoint();
            Signed320 dx = WideArithmetic.SubtractSigned320(point.XNumerator, reference.XNumerator);
            Signed320 dy = WideArithmetic.SubtractSigned320(point.YNumerator, reference.YNumerator);
            Signed320 squaredDistance = Signed320.NarrowValue(WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(dx, dx), WideArithmetic.MultiplySigned320(dy, dy)));
            Signed576 parallelAxis = WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(squaredDistance, weight.Numerator), 60);
            Signed576 intrinsic = WideArithmetic.MultiplySigned576(
                CompoundMomentMath2D.GetWeightedMoment(part, weight, _parts[i].LocalRotation, dx, dy,
                    out Signed576 firstMomentContribution), Signed192.One);
            geometry = WideArithmetic.AddSigned576(geometry, WideArithmetic.AddSigned576(intrinsic, parallelAxis));
            geometry = WideArithmetic.AddSigned576(geometry, firstMomentContribution);
        }
        // Semantic points retain Q64 coordinates. Their squared offsets and
        // weighted intrinsic moments share Q256; only the final parent moment rounds.
        _momentGeometry = geometry;
        _momentDenominator = Signed704.ExtendValue(WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(totalWeight.Numerator),
                Signed192.One, Signed192.One, Signed192.One, Signed192.One), 60));
        _momentReferencePoint = referencePoint;
        _hasMomentGeometry = true;
    }

    internal Fixed64 GetMutationGeometryReach(LSCollider2D changed, Vector2d ownerScale,
        Fixed64 candidateIntrinsic, Vector2d candidateOffset)
    {
        Fixed64 reach = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
        {
            LSCollider2D part = _partColliders[i];
            bool replaced = ReferenceEquals(part, changed);
            Fixed64 intrinsic = replaced ? candidateIntrinsic
                : part.GetCandidateGeometryReach(ownerScale, _parts[i].LocalScale);
            reach = FixedMath.Max(reach, ColliderGeometryPolicy.Add(
                ColliderGeometryPolicy.Attachment(replaced ? candidateOffset : part.LocalOffset, ownerScale),
                intrinsic));
        }
        return reach;
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        Vector2d min = Vector2d.Zero;
        Vector2d max = Vector2d.Zero;

        Fixed64 reach = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart2D part = _parts[i];
            LSCollider2D partCollider = _partColliders[i];
            partCollider.PrepareCompoundPart(
                snapshot,
                part.LocalRotation,
                part.LocalScale,
                PreparedContext);
            reach = FixedMath.Max(reach, ColliderGeometryPolicy.Add(
                ColliderGeometryPolicy.Attachment(partCollider.LocalOffset, snapshot.OwnerScale),
                partCollider.PreparedGeometryReach));

            Vector2d partMin = partCollider.PreparedShapeBounds.Min;
            Vector2d partMax = partCollider.PreparedShapeBounds.Max;
            if (i == 0)
            {
                min = partMin;
                max = partMax;
                continue;
            }

            min = new Vector2d(FixedMath.Min(min.X, partMin.X), FixedMath.Min(min.Y, partMin.Y));
            max = new Vector2d(FixedMath.Max(max.X, partMax.X), FixedMath.Max(max.Y, partMax.Y));
        }

        _ = CalculatePreparedLocalMassPoint();
        _ = CalculatePreparedAreaForMassProperties();
        SetPreparedGeometryReach(reach);
        SetPreparedBounds(FixedBoundArea.FromMinMax(min, max));
    }

    private protected override void PublishShape()
    {
        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart2D part = _parts[i];
            _partColliders[i].PublishCompoundPart(
                part.LocalRotation,
                part.LocalScale,
                PreparedContext);
        }
    }

    private static LSCollider2D MaterializePartCollider(CompoundColliderPart2D part)
    {
        LSCollider2D collider = part.Shape.CreateRuntimeCollider();
        collider.LocalOffset = part.LocalOffset;
        collider.Material = part.ResolveMaterial(PhysicsMaterial.Default);
        return collider;
    }

    /// <inheritdoc/>
    protected override void OnMaterialChanged()
    {
        for (int i = 0; i < _parts.Length; i++)
            _partColliders[i].Material = _parts[i].ResolveMaterial(Material);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidatePart(CompoundColliderPart2D part) =>
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "2D compound collider part cannot be default.");
}
