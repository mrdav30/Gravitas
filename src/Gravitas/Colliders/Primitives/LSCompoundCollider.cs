using FixedMathSharp;
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
            _partColliders[i].ReserveCompoundPart(this);
        }
    }

    public override ColliderType Shape => ColliderType.Compound;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Bounds.Scope.Magnitude;
    }

    public override Vector3d ScaledSize
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Bounds.Proportions;
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

    protected override void RebuildRuntimeShape() => BuildShape();

    protected override void BuildShape()
    {
        Area = Fixed64.Zero;
        Vector3d min = Vector3d.Zero;
        Vector3d max = Vector3d.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart part = _parts[i];
            LSCollider partCollider = _partColliders[i];
            partCollider.LocalOffset = part.LocalOffset;
            partCollider.BindCompoundPart(this, part.LocalRotation, part.LocalScale, Context);

            if (i == 0)
            {
                min = partCollider.BoundsMin;
                max = partCollider.BoundsMax;
            }
            else
            {
                min = Vector3d.Min(min, partCollider.BoundsMin);
                max = Vector3d.Max(max, partCollider.BoundsMax);
            }

            Area += partCollider.Area;
        }

        SetBoundsMinMax(min, max);
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass)
    {
        if (_parts.Length == 0)
            return Fixed3x3.Zero;

        Fixed64 totalArea = Area;
        Fixed64 equalPartMass = mass / (Fixed64)_parts.Length;
        Fixed3x3 tensor = Fixed3x3.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            LSCollider part = _partColliders[i];
            Fixed64 partMass = totalArea > Fixed64.Zero
                ? mass * (part.Area / totalArea)
                : equalPartMass;

            tensor += part.CalculateInertiaTensor(partMass);
            tensor += CalculateParallelAxisTensor(partMass, part.Center - Center);
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

    private static Fixed3x3 CalculateParallelAxisTensor(Fixed64 mass, Vector3d offset)
    {
        Fixed64 xSqr = offset.X * offset.X;
        Fixed64 ySqr = offset.Y * offset.Y;
        Fixed64 zSqr = offset.Z * offset.Z;

        return new Fixed3x3(
            mass * (ySqr + zSqr), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, mass * (xSqr + zSqr), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, mass * (xSqr + ySqr));
    }

    private static LSCollider MaterializePartCollider(CompoundColliderPart part)
    {
        LSCollider collider = part.Shape.CreateRuntimeCollider();
        collider.LocalOffset = part.LocalOffset;
        return collider;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidatePart(CompoundColliderPart part) =>
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "Compound collider part cannot be default.");
}
