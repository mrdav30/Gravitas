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

    public LSCompoundCollider(params CompoundColliderPart[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "Compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
        {
            ValidatePart(parts[i]);
            for (int j = 0; j < i; j++)
            {
                SwiftThrowHelper.ThrowIfArgument(
                    ReferenceEquals(parts[j].Collider, parts[i].Collider),
                    nameof(parts),
                    "Compound collider parts cannot reuse the same collider instance.");
            }
        }

        _parts = new CompoundColliderPart[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            _parts[i] = parts[i];
            _parts[i].Collider.ReserveCompoundPart(this);
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
    public LSCollider GetPartCollider(int index)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return _parts[index].Collider;
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
            LSCollider partCollider = part.Collider;
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
            LSCollider part = _parts[i].Collider;
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
            area += _parts[i].Collider.GetFrontalArea(direction);
        return area;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        int bestIndex = FindClosestPartIndex(other);
        return _parts[bestIndex].Collider.ClosestPointOnSurface(other);
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        int bestIndex = FindClosestPartIndex(point);
        return _parts[bestIndex].Collider.GetNormalAtPoint(point);
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        bool hit = false;
        for (int i = 0; i < _parts.Length; i++)
            hit |= _parts[i].Collider.ColliderOverlapsRay(worker, ref outputIntersectionPoints);
        return hit;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool ContainsPartCollider(LSCollider collider) =>
        ReferenceEquals(collider.CompoundOwner, this);

    private int FindClosestPartIndex(Vector3d point)
    {
        int bestIndex = 0;
        Vector3d closest = _parts[0].Collider.ClosestPointOnSurface(point);
        Fixed64 bestDistance = Vector3d.SqrDistance(point, closest);

        for (int i = 1; i < _parts.Length; i++)
        {
            Vector3d candidate = _parts[i].Collider.ClosestPointOnSurface(point);
            Fixed64 distance = Vector3d.SqrDistance(point, candidate);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bestIndex = i;
        }

        return bestIndex;
    }

    private static Fixed3x3 CalculateParallelAxisTensor(Fixed64 mass, Vector3d offset)
    {
        Fixed64 xSqr = offset.x * offset.x;
        Fixed64 ySqr = offset.y * offset.y;
        Fixed64 zSqr = offset.z * offset.z;

        return new Fixed3x3(
            mass * (ySqr + zSqr), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, mass * (xSqr + zSqr), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, mass * (xSqr + ySqr));
    }

    private static void ValidatePart(CompoundColliderPart part)
    {
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "Compound collider part cannot be default.");

        LSCollider collider = part.Collider;
        SwiftThrowHelper.ThrowIfArgument(
            collider is LSCompoundCollider,
            nameof(part),
            "Compound collider parts cannot contain another Compound collider.");
        SwiftThrowHelper.ThrowIfArgument(
            collider is LSMeshCollider { Mode: MeshColliderMode.Concave },
            nameof(part),
            "Concave mesh colliders cannot be used as compound collider parts.");
        SwiftThrowHelper.ThrowIfArgument(
            collider.HasHostBinding || collider.TryGetBoundContext(out _),
            nameof(part),
            "Compound collider parts cannot already be initialized or bound to a context.");
    }
}
