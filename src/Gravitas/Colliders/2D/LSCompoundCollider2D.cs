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
    private readonly FixedMassPoint2d[] _massPointScratch;
    private readonly FixedMassWeight[] _massWeightScratch;
    private readonly ContactManifold2D _partManifoldScratch = new();

    public LSCompoundCollider2D(params CompoundColliderPart2D[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "2D compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
            ValidatePart(parts[i]);

        _parts = new CompoundColliderPart2D[parts.Length];
        _partColliders = new LSCollider2D[parts.Length];
        _massPointScratch = new FixedMassPoint2d[parts.Length];
        _massWeightScratch = new FixedMassWeight[parts.Length];
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

    internal override FixedMassPoint2d CalculateLocalMassPoint() =>
        CalculateAggregateMassPoint(usePrepared: false);

    internal override FixedMassPoint2d CalculatePreparedLocalMassPoint() =>
        CalculateAggregateMassPoint(usePrepared: true);

    private FixedMassPoint2d CalculateAggregateMassPoint(bool usePrepared)
    {
        bool hasPositiveWeight = false;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D partCollider = _partColliders[i];
            FixedMassWeight weight = usePrepared
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
                _massWeightScratch[i] = FixedMassWeight.One;
        }

        if (!FixedMassPoint2d.TryGetWeightedAverage(
            _massPointScratch,
            _massWeightScratch,
            out Vector2d center))
        {
            throw new InvalidOperationException(
                usePrepared
                    ? "Prepared 2D compound mass-property point is outside the Fixed64 coordinate domain."
                    : "The 2D compound collider's center of mass is outside the Fixed64 coordinate domain.");
        }
        return FixedMassPoint2d.FromPoint(center);
    }

    internal override FixedMassWeight CalculateAreaForMassProperties() =>
        CalculateAggregateMassWeight(usePrepared: false);

    internal override FixedMassWeight CalculatePreparedAreaForMassProperties() =>
        CalculateAggregateMassWeight(usePrepared: true);

    private FixedMassWeight CalculateAggregateMassWeight(bool usePrepared)
    {
        FixedMassWeight totalWeight = FixedMassWeight.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            FixedMassWeight weight = usePrepared
                ? _partColliders[i].CalculatePreparedAreaForMassProperties()
                : _partColliders[i].CalculateAreaForMassProperties();
            totalWeight = totalWeight.Add(weight);
        }

        return totalWeight;
    }

    internal override Fixed64 CalculateCenterOfMassMoment(Fixed64 mass)
    {
        Vector2d center = CalculateLocalCenterOfMassOffset();
        FixedMassWeight totalWeight = FixedMassWeight.Zero;
        int residualPartIndex = _partColliders.Length - 1;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            FixedMassWeight weight =
                _partColliders[i].CalculateAreaForMassProperties();
            totalWeight = totalWeight.Add(weight);
            residualPartIndex = i;
        }

        bool useEqualWeights = totalWeight.IsZero;
        if (useEqualWeights)
        {
            totalWeight = FixedMassWeight.Zero;
            for (int i = 0; i < _partColliders.Length; i++)
                totalWeight = totalWeight.Add(FixedMassWeight.One);
        }

        FixedMassWeight cumulativeWeight = FixedMassWeight.Zero;
        Fixed64 assignedMass = Fixed64.Zero;
        Fixed64 moment = Fixed64.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D part = _partColliders[i];
            FixedMassWeight weight = useEqualWeights
                ? FixedMassWeight.One
                : part.CalculateAreaForMassProperties();
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

            if (!part.CalculateLocalMassPoint()
                    .TryAddParallelAxisMoment(
                        part.CalculateCenterOfMassMoment(partMass),
                        partMass,
                        center,
                        out Fixed64 contribution)
                || !Fixed64.TryAdd(
                    moment,
                    contribution,
                    out moment))
            {
                throw new InvalidOperationException(
                    "The 2D compound collider's moment of inertia is outside the Fixed64 scalar domain.");
            }
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

        _ = CalculatePreparedLocalMassPoint();
        _ = CalculatePreparedAreaForMassProperties();
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
