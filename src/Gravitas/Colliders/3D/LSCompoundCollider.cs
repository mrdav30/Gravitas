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

    public LSCompoundCollider(params CompoundColliderPart[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "Compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
            ValidatePart(parts[i]);

        _parts = new CompoundColliderPart[parts.Length];
        _partColliders = new LSCollider[parts.Length];
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

    public override ColliderType Shape => ColliderType.Compound;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => HasCommittedShape
            ? CanonicalCenteredProxyRadius
            : ColliderCanonicalBounds
                .GetCurrentCenteredProxyRadius(this);
    }

    public ReadOnlySpan<CompoundColliderPart> Parts => _parts;

    public int PartCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _parts.Length;
    }

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

    public override Vector3d CalculateLocalCenterOfMassOffset()
    {
        Fixed64 totalWeight = CalculateMassPropertyWeight();
        Vector3d weightedCenter = Vector3d.Zero;
        if (totalWeight > Fixed64.Zero)
        {
            for (int i = 0; i < _parts.Length; i++)
            {
                LSCollider part = _partColliders[i];
                weightedCenter += part.CalculateLocalCenterOfMassOffset()
                    * part.CalculateMassPropertyWeight();
            }

            return weightedCenter / totalWeight;
        }

        for (int i = 0; i < _parts.Length; i++)
            weightedCenter += _partColliders[i].CalculateLocalCenterOfMassOffset();

        return weightedCenter / (Fixed64)_parts.Length;
    }

    protected internal override Fixed64 CalculateMassPropertyWeight()
    {
        Fixed64 totalWeight = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
            totalWeight += _partColliders[i].CalculateMassPropertyWeight();
        return totalWeight;
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        Fixed64 totalWeight = Fixed64.Zero;
        int residualPartIndex = _parts.Length - 1;
        for (int i = 0; i < _parts.Length; i++)
        {
            Fixed64 weight = _partColliders[i].CalculateMassPropertyWeight();
            totalWeight += weight;
            if (weight > Fixed64.Zero)
                residualPartIndex = i;
        }

        Fixed64 assignedMass = Fixed64.Zero;
        Fixed3x3 tensor = Fixed3x3.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            LSCollider part = _partColliders[i];
            Fixed64 weight = part.CalculateMassPropertyWeight();
            Fixed64 partMass;
            if (i == residualPartIndex)
            {
                partMass = mass - assignedMass;
            }
            else if (totalWeight > Fixed64.Zero)
            {
                partMass = weight > Fixed64.Zero
                    ? (mass * weight) / totalWeight
                    : Fixed64.Zero;
                assignedMass += partMass;
            }
            else
            {
                partMass = mass / (Fixed64)_parts.Length;
                assignedMass += partMass;
            }

            Vector3d partCenterOfMass = part.CalculateLocalCenterOfMassOffset();
            Fixed3x3 partTensor = part.CalculateInertiaTensor(partMass, partCenterOfMass);
            partTensor = InertiaTensorMath.RotateToFrame(partTensor, part.CompoundLocalRotation);
            tensor += AddParallelAxisTensor(partTensor, partMass, localCenterOfMassOffset - partCenterOfMass);
        }

        return tensor;
    }

    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 area = Fixed64.Zero;
        for (int i = 0; i < _parts.Length; i++)
            area += _partColliders[i].GetFrontalArea(direction);
        return area;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        int bestIndex = FindClosestPartIndex(other);
        return _partColliders[bestIndex].ClosestPointOnSurface(other);
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        int bestIndex = FindClosestPartIndex(point);
        return _partColliders[bestIndex].GetNormalAtPoint(point);
    }

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

    private int FindClosestPartIndex(Vector3d point)
    {
        int bestIndex = 0;
        Vector3d closest = _partColliders[0].ClosestPointOnSurface(point);
        Fixed64 bestDistance = Vector3d.DistanceSquared(point, closest);

        for (int i = 1; i < _parts.Length; i++)
        {
            Vector3d candidate = _partColliders[i].ClosestPointOnSurface(point);
            Fixed64 distance = Vector3d.DistanceSquared(point, candidate);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bestIndex = i;
        }

        return bestIndex;
    }

    private static LSCollider MaterializePartCollider(CompoundColliderPart part)
    {
        LSCollider collider = part.Shape.CreateRuntimeCollider();
        collider.LocalOffset = part.LocalOffset;
        collider.Material = part.ResolveMaterial(PhysicsMaterial.Default);
        return collider;
    }

    protected override void OnMaterialChanged()
    {
        for (int i = 0; i < _parts.Length; i++)
            _partColliders[i].Material = _parts[i].ResolveMaterial(Material);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidatePart(CompoundColliderPart part) =>
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "Compound collider part cannot be default.");
}
