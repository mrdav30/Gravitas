//=======================================================================
// LSCompoundCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Materials;
using Gravitas.Queries;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Represents one collider identity whose collision shape is composed from
/// deterministic internal primitive and convex-mesh parts.
/// </summary>
public sealed class LSCompoundCollider : LSCollider
{
    private readonly CompoundColliderPart[] _parts;
    private readonly LSCollider[] _partColliders;
    private readonly ExactMassPoint3D[] _massPointScratch;
    private readonly ExactMassWeight[] _massWeightScratch;
    private bool _hasMassPropertyCenter, _massPropertyCommitted;
    private Vector3d _massPropertyCenter, _massPropertyOwnerScale, _massPropertyOffset;
    private bool _inertiaGeometryValid, _inertiaGeometryCommitted;
    private Vector3d _inertiaGeometryScale, _inertiaGeometryOffset, _inertiaGeometryReference;
    private ExactMassWeight _inertiaGeometryWeight;
    private CompoundInertiaTensor _inertiaGeometry;


    /// <summary>Creates a runtime compound collider from authored 3D parts.</summary>
    public LSCompoundCollider(params CompoundColliderPart[] parts)
    {
        ChronicleGuard.ThrowIfNull(parts, nameof(parts));
        ChronicleGuard.ThrowIfArgument(parts.Length == 0, nameof(parts), "Compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
            ValidatePart(parts[i]);

        _parts = new CompoundColliderPart[parts.Length];
        _partColliders = new LSCollider[parts.Length];
        _massPointScratch = new ExactMassPoint3D[parts.Length];
        _massWeightScratch = new ExactMassWeight[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            _parts[i] = parts[i];
            _partColliders[i] = MaterializePartCollider(parts[i]);
            _partColliders[i].ReserveCompoundPart(
                this,
                parts[i].LocalRotation,
                parts[i].LocalScale);
        }
    }

    /// <inheritdoc/>
    public override ColliderType Shape => ColliderType.Compound;

    /// <inheritdoc/>
    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <inheritdoc/>
    public override Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => HasCommittedShape
            ? CanonicalCenteredProxyRadius
            : ColliderCanonicalBounds
                .GetCurrentCenteredProxyRadius(this);
    }

    /// <summary>Gets the authored parts in stable source order.</summary>
    public ReadOnlySpan<CompoundColliderPart> Parts => _parts;

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
        ChronicleGuard.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return index;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal LSCollider GetPartCollider(int index)
    {
        ChronicleGuard.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return _partColliders[index];
    }

    internal Fixed64 GetMutationGeometryReach(LSCollider changed, in ColliderShapeSnapshot candidate)
    {
        Fixed64 reach = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
        {
            LSCollider part = _partColliders[i];
            ColliderShapeSnapshot snapshot = ReferenceEquals(part, changed) ? candidate
                : new ColliderShapeSnapshot(default, default, candidate.OwnerScale, _parts[i].LocalScale,
                    part.LocalOffset, part.Size, part.Radius);
            reach = FixedMath.Max(reach, ColliderGeometryPolicy.Add(
                ColliderGeometryPolicy.Attachment(snapshot.LocalOffset, snapshot.OwnerScale),
                part.GetCandidateGeometryReach(snapshot)));
        }
        return reach;
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        Vector3d min = Vector3d.Zero;
        Vector3d max = Vector3d.Zero;

        Fixed64 reach = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart part = _parts[i];
            LSCollider partCollider = _partColliders[i];
            partCollider.PrepareCompoundPart(
                snapshot,
                part.LocalRotation,
                part.LocalScale,
                PreparedContext);
            reach = FixedMath.Max(reach, ColliderGeometryPolicy.Add(
                partCollider.PreparedAttachmentReach,
                partCollider.PreparedGeometryReach));

            if (i == 0)
            {
                min = partCollider.PreparedShapeBounds.Min;
                max = partCollider.PreparedShapeBounds.Max;
            }
            else
            {
                min = Vector3d.Min(min, partCollider.PreparedShapeBounds.Min);
                max = Vector3d.Max(max, partCollider.PreparedShapeBounds.Max);
            }
        }

        _ = CalculatePreparedLocalMassPoint();
        _ = CalculatePreparedMassPropertyWeight();
        SetPreparedGeometryReach(reach);
        SetPreparedBounds(FixedBoundBox.FromMinMax(min, max));
    }

    private protected override void PublishShape()
    {
        Fixed64 area = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart part = _parts[i];
            _partColliders[i].PublishCompoundPart(
                part.LocalRotation,
                part.LocalScale,
                PreparedContext);
            area += _partColliders[i].Area;
        }

        Area = area;
    }

    internal override ExactMassPoint3D CalculateLocalMassPoint()
    {
        GetCurrentShapeScales(out Vector3d scale, out _);
        return _hasMassPropertyCenter && _massPropertyOwnerScale == scale
            && _massPropertyOffset == LocalOffset && _massPropertyCommitted == HasCommittedShape
            ? ExactMassPoint3D.FromPoint(_massPropertyCenter)
            : CalculateAggregateMassPoint(usePrepared: false);
    }

    internal override ExactMassPoint3D CalculatePreparedLocalMassPoint() =>
        CalculateAggregateMassPoint(usePrepared: true);

    private ExactMassPoint3D CalculateAggregateMassPoint(bool usePrepared)
    {
        bool hasPositiveWeight = false;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider part = _partColliders[i];
            ExactMassWeight weight = usePrepared
                ? part.CalculatePreparedMassPropertyWeight()
                : part.CalculateMassPropertyWeight();
            _massPointScratch[i] = usePrepared
                ? part.CalculatePreparedLocalMassPoint()
                : part.CalculateLocalMassPoint();
            _massWeightScratch[i] = weight;
            hasPositiveWeight |= !weight.IsZero;
        }

        if (!hasPositiveWeight)
        {
            // Admitted solids and shells have positive exact weights. Only
            // unsupported closed-mesh parts need this equal-point fallback.
            for (int i = 0; i < _massWeightScratch.Length; i++)
                _massWeightScratch[i] = ExactMassWeight.One;
        }

        if (!ExactMassPoint3D.TryGetWeightedAverage(
                _massPointScratch,
                _massWeightScratch,
                out Vector3d center))
        {
            throw new InvalidOperationException(
                usePrepared
                    ? "Prepared compound mass-property point is outside the Fixed64 coordinate domain."
                    : "The compound collider's center of mass is outside the Fixed64 coordinate domain.");
        }
        if (!usePrepared)
        {
            GetCurrentShapeScales(out _massPropertyOwnerScale, out _);
            _massPropertyOffset = LocalOffset;
            _massPropertyCommitted = HasCommittedShape;
            _massPropertyCenter = center;
            _hasMassPropertyCenter = true;
        }
        return ExactMassPoint3D.FromPoint(center);
    }

    internal override ExactMassWeight CalculateMassPropertyWeight() =>
        CalculateAggregateMassWeight(usePrepared: false);

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight() =>
        CalculateAggregateMassWeight(usePrepared: true);

    private ExactMassWeight CalculateAggregateMassWeight(bool usePrepared)
    {
        ExactMassWeight totalWeight = ExactMassWeight.Zero;
        for (int i = 0; i < _parts.Length; i++)
        {
            ExactMassWeight weight = usePrepared
                ? _partColliders[i].CalculatePreparedMassPropertyWeight()
                : _partColliders[i].CalculateMassPropertyWeight();
            totalWeight = totalWeight.Add(weight);
        }
        return totalWeight;
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass) =>
        CalculateInertiaTensor(mass, CalculateLocalCenterOfMassOffset());

    /// <inheritdoc/>
    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        if (mass <= Fixed64.Zero)
            return Fixed3x3.Zero;
        GetCurrentShapeScales(out Vector3d scale, out _);
        if (!_inertiaGeometryValid || _inertiaGeometryCommitted != HasCommittedShape || _inertiaGeometryScale != scale
            || _inertiaGeometryOffset != LocalOffset || _inertiaGeometryReference != localCenterOfMassOffset)
        {
            ExactMassWeight total = ExactMassWeight.Zero;
            CompoundInertiaTensor geometry = default;
            for (int i = 0; i < _partColliders.Length; i++)
            {
                LSCollider part = _partColliders[i];
                if (!part.SupportsMassProperties)
                    continue;
                ExactMassWeight weight = part.CalculateMassPropertyWeight();
                total = total.Add(weight);
                CompoundInertiaMath3D.Add(ref geometry, part, weight, localCenterOfMassOffset);
            }
            if (total.IsZero)
                throw new InvalidOperationException("Compound inertia requires at least one part with valid mass properties.");

            // Cache geometry only after a complete evaluation. Neither a mass
            // change nor world pose changes the local intrinsic contribution.
            _inertiaGeometry = geometry;
            _inertiaGeometryWeight = total;
            _inertiaGeometryScale = scale;
            _inertiaGeometryOffset = LocalOffset;
            _inertiaGeometryReference = localCenterOfMassOffset;
            _inertiaGeometryCommitted = HasCommittedShape;
            _inertiaGeometryValid = true;
        }
        return _inertiaGeometry.Materialize(mass, _inertiaGeometryWeight);
    }

    /// <inheritdoc/>
    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 area = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
            area += _partColliders[i].GetFrontalArea(direction);
        return area;
    }

    /// <inheritdoc/>
    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        FixedPointAnchor anchor =
            GetClosestSurfaceAnchor(other, out _);
        if (anchor.TryGetPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            "The closest compound surface point is outside the representable coordinate domain.");
    }

    /// <inheritdoc/>
    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        _ = GetClosestSurfaceAnchor(
            point,
            out Vector3d normal);
        return normal;
    }

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point,
        out Vector3d normal) =>
        GetClosestSurfaceAnchor(point, out normal, out _);

    internal FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point,
        out Vector3d normal,
        out LSCollider surfaceOwner)
    {
        var reference = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        surfaceOwner = _partColliders[0];
        FixedPointAnchor closest =
            surfaceOwner.GetClosestSurfaceAnchor(
                point,
                out normal);
        for (int i = 1; i < _partColliders.Length; i++)
        {
            FixedPointAnchor candidate =
                _partColliders[i].GetClosestSurfaceAnchor(
                    point,
                    out Vector3d candidateNormal);
            if (reference.CompareSquaredDistance(
                    candidate,
                    closest) >= 0)
            {
                continue;
            }

            closest = candidate;
            normal = candidateNormal;
            surfaceOwner = _partColliders[i];
        }

        return closest;
    }

    /// <inheritdoc/>
    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        bool hit = false;
        for (int i = 0; i < _parts.Length; i++)
            hit |= _partColliders[i].ColliderOverlapsRay(worker, ref outputIntersectionPoints);
        return hit;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool ContainsPartCollider(LSCollider collider) =>
        ReferenceEquals(collider.CompoundOwner, this);

    private static LSCollider MaterializePartCollider(CompoundColliderPart part)
    {
        LSCollider collider = part.Shape.CreateRuntimeCollider();
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
    private static void ValidatePart(CompoundColliderPart part) =>
        ChronicleGuard.ThrowIfArgument(part.IsDefault, nameof(part), "Compound collider part cannot be default.");
}
