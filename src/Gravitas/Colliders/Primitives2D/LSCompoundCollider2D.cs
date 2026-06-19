using FixedMathSharp;
using SwiftCollections;
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

    public LSCompoundCollider2D(params CompoundColliderPart2D[] parts)
    {
        SwiftThrowHelper.ThrowIfNull(parts, nameof(parts));
        SwiftThrowHelper.ThrowIfArgument(parts.Length == 0, nameof(parts), "2D compound collider must contain at least one part.");

        for (int i = 0; i < parts.Length; i++)
            ValidatePart(parts[i]);

        _parts = new CompoundColliderPart2D[parts.Length];
        _partColliders = new LSCollider2D[parts.Length];
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
        get
        {
            Vector2d center = Center;
            Fixed64 bestDistance = Fixed64.Zero;
            Fixed64 bestDistanceSquared = Fixed64.Zero;
            for (int i = 0; i < _partColliders.Length; i++)
            {
                LSCollider2D part = _partColliders[i];
                if (part is LSCircleCollider2D circle)
                {
                    Fixed64 distance = Vector2d.Distance(center, circle.Center) + circle.ScaledRadius;
                    if (distance > bestDistance)
                    {
                        bestDistance = distance;
                        bestDistanceSquared = distance * distance;
                    }
                    continue;
                }

                for (int j = 0; j < part.VertexCount; j++)
                {
                    Fixed64 distanceSquared = Vector2d.DistanceSquared(center, part.GetVertexUnchecked(j));
                    if (distanceSquared <= bestDistanceSquared)
                        continue;

                    bestDistanceSquared = distanceSquared;
                    bestDistance = FixedMath.Sqrt(distanceSquared);
                }
            }

            return bestDistance;
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

    internal override int VertexCount => 0;

    internal override Vector2d GetVertexUnchecked(int index) => Center;

    public override Vector2d CalculateLocalCenterOfMassOffset()
    {
        Fixed64 totalArea = Fixed64.Zero;
        Vector2d weightedCenter = Vector2d.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D partCollider = _partColliders[i];
            Fixed64 partArea = partCollider.CalculateAreaForMassProperties();
            if (partArea <= Fixed64.Zero)
                continue;

            totalArea += partArea;
            weightedCenter += partCollider.CalculateLocalCenterOfMassOffset() * partArea;
        }

        return totalArea > Fixed64.Zero
            ? weightedCenter / totalArea
            : base.CalculateLocalCenterOfMassOffset();
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
        if (totalArea <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 moment = Fixed64.Zero;
        for (int i = 0; i < _partColliders.Length; i++)
        {
            LSCollider2D partCollider = _partColliders[i];
            Fixed64 partArea = partCollider.CalculateAreaForMassProperties();
            if (partArea <= Fixed64.Zero)
                continue;

            Fixed64 partMass = mass * (partArea / totalArea);
            moment += partCollider.CalculateMomentOfInertia(partMass, localReferencePoint);
        }

        return moment;
    }

    protected override void RebuildShape()
    {
        Vector2d min = Vector2d.Zero;
        Vector2d max = Vector2d.Zero;

        for (int i = 0; i < _parts.Length; i++)
        {
            CompoundColliderPart2D part = _parts[i];
            LSCollider2D partCollider = _partColliders[i];
            partCollider.LocalOffset = part.LocalOffset;
            partCollider.BindCompoundPart(this, part.LocalRotation, part.LocalScale, Context);

            Vector2d partMin = new(partCollider.MinX, partCollider.MinY);
            Vector2d partMax = new(partCollider.MaxX, partCollider.MaxY);
            if (i == 0)
            {
                min = partMin;
                max = partMax;
                continue;
            }

            min = new Vector2d(FixedMath.Min(min.X, partMin.X), FixedMath.Min(min.Y, partMin.Y));
            max = new Vector2d(FixedMath.Max(max.X, partMax.X), FixedMath.Max(max.Y, partMax.Y));
        }

        SetBoundsFromMinMax(min, max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool ContainsPartCollider(LSCollider2D collider) =>
        ReferenceEquals(collider.CompoundOwner2D, this);

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
        return collider;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidatePart(CompoundColliderPart2D part) =>
        SwiftThrowHelper.ThrowIfArgument(part.IsDefault, nameof(part), "2D compound collider part cannot be default.");
}
