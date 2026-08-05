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

    /// <summary>Creates a runtime compound collider from authored 3D parts.</summary>
    public LSCompoundCollider(params CompoundColliderPart[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "Compound collider must contain at least one part.");

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
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return index;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal LSCollider GetPartCollider(int index)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return _partColliders[index];
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        Vector3d min = Vector3d.Zero;
        Vector3d max = Vector3d.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart part = _parts[i];
            LSCollider partCollider = _partColliders[i];
            partCollider.PrepareCompoundPart(
                snapshot,
                part.LocalRotation,
                part.LocalScale,
                PreparedContext);

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

    internal override ExactMassPoint3D CalculateLocalMassPoint() =>
        CalculateAggregateMassPoint(usePrepared: false);

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
            int supportedPartCount = 0;
            for (int i = 0; i < _massWeightScratch.Length; i++)
            {
                if (_partColliders[i].SupportsMassProperties)
                {
                    _massWeightScratch[i] = ExactMassWeight.One;
                    supportedPartCount++;
                }
            }

            if (supportedPartCount == 0)
            {
                for (int i = 0; i < _massWeightScratch.Length; i++)
                    _massWeightScratch[i] = ExactMassWeight.One;
            }
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

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(
        Fixed64 mass)
    {
        Vector3d center = CalculateLocalCenterOfMassOffset();
        ExactMassWeight totalWeight = ExactMassWeight.Zero;
        int residualPartIndex = _parts.Length - 1;
        int eligiblePartCount = 0;
        for (int i = 0; i < _parts.Length; i++)
        {
            LSCollider part = _partColliders[i];
            if (!part.SupportsMassProperties)
            {
                _massWeightScratch[i] = ExactMassWeight.Zero;
                continue;
            }

            eligiblePartCount++;
            ExactMassWeight weight = part.CalculateMassPropertyWeight();
            _massWeightScratch[i] = weight;
            totalWeight = totalWeight.Add(weight);
            residualPartIndex = i;
        }

        if (eligiblePartCount == 0)
        {
            throw new InvalidOperationException(
                "Compound inertia requires at least one part with valid mass properties.");
        }

        ExactMassWeight cumulativeWeight = ExactMassWeight.Zero;
        Fixed64 assignedMass = Fixed64.Zero;
        Fixed3x3 tensor = Fixed3x3.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            LSCollider part = _partColliders[i];
            ExactMassWeight weight = _massWeightScratch[i];
            cumulativeWeight = cumulativeWeight.Add(weight);
            Fixed64 partMass;
            if (i == residualPartIndex)
            {
                partMass = mass - assignedMass;
                assignedMass = mass;
            }
            else
            {
                _ = cumulativeWeight.TryGetProportionalShare(
                    mass,
                    totalWeight,
                    out Fixed64 cumulativeMass);
                partMass = cumulativeMass - assignedMass;
                assignedMass = cumulativeMass;
            }

            if (partMass == Fixed64.Zero)
                continue;

            ExactMassPoint3D partCenterOfMass =
                part.CalculateLocalMassPoint();
            Fixed3x3 partTensor =
                part.CalculateCenterOfMassInertiaTensor(partMass);
            partTensor = InertiaTensorMath.RotateToFrame(partTensor, part.CompoundLocalRotation);
            if (!partCenterOfMass.TryAddParallelAxisTensor(
                    partTensor,
                    partMass,
                    center,
                    out Fixed3x3 contribution)
                || !TryAddTensor(
                    tensor,
                    contribution,
                    out tensor))
            {
                throw new InvalidOperationException(
                    "The compound collider's inertia tensor is outside the Fixed64 scalar domain.");
            }
        }

        return tensor;
    }

    private static bool TryAddTensor(
        Fixed3x3 first,
        Fixed3x3 second,
        out Fixed3x3 result)
    {
        bool representable = Fixed64.TryAdd(first.M11, second.M11, out Fixed64 m11)
            & Fixed64.TryAdd(first.M12, second.M12, out Fixed64 m12)
            & Fixed64.TryAdd(first.M13, second.M13, out Fixed64 m13)
            & Fixed64.TryAdd(first.M21, second.M21, out Fixed64 m21)
            & Fixed64.TryAdd(first.M22, second.M22, out Fixed64 m22)
            & Fixed64.TryAdd(first.M23, second.M23, out Fixed64 m23)
            & Fixed64.TryAdd(first.M31, second.M31, out Fixed64 m31)
            & Fixed64.TryAdd(first.M32, second.M32, out Fixed64 m32)
            & Fixed64.TryAdd(first.M33, second.M33, out Fixed64 m33);
        result = representable
            ? new Fixed3x3(
                m11, m12, m13,
                m21, m22, m23,
                m31, m32, m33)
            : default;
        return representable;
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
        out Vector3d normal)
    {
        var reference = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor closest =
            _partColliders[0].GetClosestSurfaceAnchor(
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
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "Compound collider part cannot be default.");
}
