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
    private readonly Vector2d[] _massCenterScratch;
    private readonly Fixed64[] _massWeightScratch;
    private readonly ContactManifold2D _partManifoldScratch = new();

    public LSCompoundCollider2D(params CompoundColliderPart2D[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "2D compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
            ValidatePart(parts[i]);

        _parts = new CompoundColliderPart2D[parts.Length];
        _partColliders = new LSCollider2D[parts.Length];
        _massCenterScratch = new Vector2d[parts.Length];
        _massWeightScratch = new Fixed64[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            _parts[i] = parts[i];
            _partColliders[i] = MaterializePartCollider(parts[i]);
            _partColliders[i].ReserveCompoundPart(this, parts[i].LocalRotation, parts[i].LocalScale);
        }
    }

    public override ColliderType2D Shape => ColliderType2D.Compound;

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
                : ColliderCanonicalBounds2D.GetCurrentCenteredProxyRadius(this);
        }
    }

    public ReadOnlySpan<CompoundColliderPart2D> Parts => _parts;

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
    internal LSCollider2D GetPartCollider(int index)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _parts.Length, nameof(index));
        return _partColliders[index];
    }

    internal ContactManifold2D PartManifoldScratch => _partManifoldScratch;

    public override bool ContainsPoint(Vector2d point)
    {
        for (int i = 0; i < _partColliders.Length; i++)
        {
            if (_partColliders[i].ContainsPoint(point))
                return true;
        }

        return false;
    }

    public override Vector2d GetClosestPoint(Vector2d point)
    {
        int bestIndex = FindClosestPartIndex(point);
        return _partColliders[bestIndex].GetClosestPoint(point);
    }

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

    public override Vector2d CalculateLocalCenterOfMassOffset()
    {
        bool hasPositiveArea = false;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D partCollider = _partColliders[i];
            Fixed64 partArea = partCollider.CalculateAreaForMassProperties();
            _massCenterScratch[i] =
                partCollider.CalculateLocalCenterOfMassOffset();
            _massWeightScratch[i] = partArea;
            hasPositiveArea |= partArea > Fixed64.Zero;
        }

        if (hasPositiveArea)
        {
            Vector2d.TryGetWeightedAverage(
                _massCenterScratch,
                _massWeightScratch,
                out Vector2d weightedCenter);
            return weightedCenter;
        }

        for (int i = 0; i < _partColliders.Length; i++)
            _massWeightScratch[i] = Fixed64.One;

        Vector2d.TryGetWeightedAverage(
            _massCenterScratch,
            _massWeightScratch,
            out Vector2d equalWeightedCenter);
        return equalWeightedCenter;
    }

    internal override Fixed64 CalculateAreaForMassProperties()
    {
        Fixed64 totalArea = Fixed64.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
            totalArea += _partColliders[i].CalculateAreaForMassProperties();

        return totalArea;
    }

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 totalArea = CalculateAreaForMassProperties();
        Fixed64 equalPartMass = mass / (Fixed64)_partColliders.Length;

        Fixed64 moment = Fixed64.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D partCollider = _partColliders[i];
            Fixed64 partArea = partCollider.CalculateAreaForMassProperties();
            Fixed64 partMass = totalArea > Fixed64.Zero
                ? mass * (partArea / totalArea)
                : equalPartMass;
            moment += partCollider.CalculateMomentOfInertia(partMass, localReferencePoint);
        }

        return moment;
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        Vector2d min = Vector2d.Zero;
        Vector2d max = Vector2d.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart2D part = _parts[i];
            LSCollider2D partCollider = _partColliders[i];
            partCollider.PrepareCompoundPart(
                snapshot,
                part.LocalRotation,
                part.LocalScale,
                PreparedContext);

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

    private int FindClosestPartIndex(Vector2d point)
    {
        int bestIndex = 0;
        Vector2d closest = _partColliders[0].GetClosestPoint(point);
        Fixed64 bestDistance = Vector2d.DistanceSquared(point, closest);

        for (int i = 1; i < _partColliders.Length; i++)
        {
            Vector2d candidate = _partColliders[i].GetClosestPoint(point);
            Fixed64 distance = Vector2d.DistanceSquared(point, candidate);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bestIndex = i;
        }

        return bestIndex;
    }

    private static LSCollider2D MaterializePartCollider(CompoundColliderPart2D part)
    {
        LSCollider2D collider = part.Shape.CreateRuntimeCollider();
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
    private static void ValidatePart(CompoundColliderPart2D part) =>
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "2D compound collider part cannot be default.");
}
