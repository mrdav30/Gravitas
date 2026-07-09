//=======================================================================
// GravitasCollision2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns GridForge-backed pure 2D collision partitioning state for one context.
/// </summary>
public sealed class GravitasCollision2DService
{
    private const int DefaultPartitionPoolCapacity = 1024;
    private static readonly PhysicsPartition2DOrderComparer PartitionOrderComparer = new();
    private static readonly Collider2DIdComparer ColliderIdComparer = new();

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsPartition2D> _activePartitions = new(DefaultPartitionPoolCapacity);
    private readonly SwiftStack<PhysicsPartition2D> _inactivePartitionPool = new(DefaultPartitionPoolCapacity);
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly GridTraceScratch _traceScratch = new();
    private readonly SwiftList<PhysicsPartition2D> _retainedPartitions = new();
    private readonly SwiftList<PhysicsPartition2D> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamicIds = new();
    private readonly SwiftList<int> _distributionStaticIds = new();
    private readonly SwiftList<PhysicsPartition2D> _queryPartitions = new();
    private readonly SwiftList<int> _queryColliderIds = new();
    private readonly Action<PhysicsPartition2D> _releaseRetainedPartition;
    private readonly SwiftSparseSet _deferredPartitionRefreshIds = new();

    private int _retainedPartitionRetirementCursor;
    private bool _isDistributing;

    public GravitasCollision2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
        _releaseRetainedPartition = ReleasePartition;
    }

    public GravitasWorldContext Context => _context;

    public uint Version { get; private set; } = 1;

    public int ActivePartitionCount => _activePartitions.Count;

    public int InactivePartitionCount => _inactivePartitionPool.Count;

    internal int RetainedPartitionCount => _retainedPartitions.Count;

    public void Reset()
    {
        DetachRetainedPartitions();
        _activePartitions.Clear();
        _redundancyChecker.Clear();
        _coveredVoxels.FastClear();
        _traceScratch.Clear();
        _distributionPartitions.FastClear();
        _distributionDynamicIds.FastClear();
        _distributionStaticIds.FastClear();
        _queryPartitions.FastClear();
        _queryColliderIds.FastClear();
        _deferredPartitionRefreshIds.Clear();
        _inactivePartitionPool.Clear();
        Version = 1;
        _retainedPartitionRetirementCursor = 0;
        _isDistributing = false;
    }

    public void Deactivate() => Reset();

    internal bool RefreshColliderPartition(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsActive)
        {
            if (collider.IsPartitioned)
                ClearPartitionedCollider(collider, force: true);

            return false;
        }

        GetPlanarCoverageBounds(collider, out Vector2d coverageMin, out Vector2d coverageMax);
        PhysicsPartitionMobilityKind kind = GetMobilityKind(collider);
        if (collider.MatchesPartitionGridBounds(coverageMin, coverageMax, (int)kind))
            return false;

        if (collider.IsPartitioned)
            ClearPartitionedCollider(collider, force: true);

        return PartitionCollider(collider, coverageMin, coverageMax, kind);
    }

    internal bool RefreshColliderPartitionAfterShapeChange(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!_isDistributing)
        {
            if (collider.Id >= 0)
                _deferredPartitionRefreshIds.Remove(collider.Id);

            return RefreshColliderPartition(collider);
        }

        if (collider.Id >= 0)
            _deferredPartitionRefreshIds.Add(collider.Id);

        return false;
    }

    internal bool PartitionCollider(LSCollider2D collider)
    {
        GetPlanarCoverageBounds(collider, out Vector2d coverageMin, out Vector2d coverageMax);
        return PartitionCollider(collider, coverageMin, coverageMax);
    }

    private bool PartitionCollider(LSCollider2D collider, Vector2d coverageMin, Vector2d coverageMax)
    {
        return PartitionCollider(collider, coverageMin, coverageMax, GetMobilityKind(collider));
    }

    private bool PartitionCollider(
        LSCollider2D collider,
        Vector2d coverageMin,
        Vector2d coverageMax,
        PhysicsPartitionMobilityKind kind)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (collider.IsPartitioned || !collider.IsActive)
            return false;

        SwiftList<WorldVoxelIndex> partitionedCoordinates = collider.GetOrCreatePartitionCoordinates();
        partitionedCoordinates.FastClear();

        try
        {
            PartitionCoveredVoxels(collider, coverageMin, coverageMax, partitionedCoordinates, kind);
            if (partitionedCoordinates.Count == 0)
                return false;

            collider.MarkPartitioned(coverageMin, coverageMax, (int)kind);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal bool ClearPartitionedCollider(LSCollider2D collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsPartitioned)
            return false;

        GetPlanarCoverageBounds(collider, out Vector2d coverageMin, out Vector2d coverageMax);
        PhysicsPartitionMobilityKind currentKind = GetMobilityKind(collider);
        if (!force && collider.MatchesPartitionGridBounds(coverageMin, coverageMax, (int)currentKind))
            return false;

        SwiftList<WorldVoxelIndex> coordinates = collider.PartitionCoordinates!;
        GridWorld world = _context.World;
        PhysicsPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.PartitionKind);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsPartition2D? partition))
                {
                    continue;
                }

                RemoveObject(partition!, collider.Id, partitionKind);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }

        collider.MarkUnpartitioned();
        collider.ClearPartitionCoordinates();
        return true;
    }

    internal void RefreshPartitionAwakeState(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsPartitioned)
            return;

        SwiftList<WorldVoxelIndex> coordinates = collider.PartitionCoordinates!;
        SolidBody2D? body = collider.Body;
        if (collider.IsStatic || body!.IsKinematic)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;

        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsPartition2D? partition))
                {
                    continue;
                }

                partition!.SetDynamicObjectAwake(collider.Id, awake);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal void CheckAndDistributeCollisions()
    {
        RefreshDeferredColliderPartitions();
        Version++;

        _distributionPartitions.FastClear();
        foreach (PhysicsPartition2D partition in _activePartitions)
            _distributionPartitions.Add(partition);

        _distributionPartitions.SortInPlace(PartitionOrderComparer);

        _isDistributing = true;
        try
        {
            for (int i = 0; i < _distributionPartitions.Count; i++)
            {
                _distributionPartitions[i].Distribute(
                    _distributionDynamicIds,
                    _distributionStaticIds);
            }
        }
        finally
        {
            _isDistributing = false;
        }
    }

    internal void CollectOverlapCircleCandidates(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        uint queryVersion,
        SwiftList<LSCollider2D> candidates)
    {
        candidates.FastClear();

        Vector2d min = new(center.X - radius, center.Y - radius);
        Vector2d max = new(center.X + radius, center.Y + radius);

        CollectBoundsCandidates(min, max, layerMask, queryVersion, raycastQuery: false, candidates);
    }

    internal void CollectBoundsCandidates(
        Vector2d min,
        Vector2d max,
        PhysicsLayerMask layerMask,
        uint queryVersion,
        bool raycastQuery,
        SwiftList<LSCollider2D> candidates,
        bool staticStyleOnly = false)
    {
        RefreshDeferredColliderPartitions();
        candidates.FastClear();

        CollectCoveredPartitions(min, max, _queryPartitions);
        _queryPartitions.SortInPlace(PartitionOrderComparer);

        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsPartition2D partition = _queryPartitions[i];
            if (staticStyleOnly)
                partition.CopyStaticStyleColliderIds(_queryColliderIds);
            else
                partition.CopyAllColliderIds(_queryColliderIds);

            for (int j = 0; j < _queryColliderIds.Count; j++)
            {
                int colliderId = _queryColliderIds[j];
                if (!_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider)
                    || !collider!.IsActive
                    || (staticStyleOnly && !IsStaticStyleCollider(collider))
                    || IsDuplicateQueryCandidate(collider, queryVersion, raycastQuery)
                    || !layerMask.Includes(collider.Layer)
                    || collider.MaxX < min.X
                    || collider.MinX > max.X
                    || collider.MaxY < min.Y
                    || collider.MinY > max.Y)
                {
                    continue;
                }

                candidates.Add(collider);
            }
        }

        candidates.SortInPlace(ColliderIdComparer);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsDuplicateQueryCandidate(LSCollider2D collider, uint queryVersion, bool raycastQuery)
    {
        if (raycastQuery)
        {
            if (collider.RaycastVersion == queryVersion)
                return true;

            collider.RaycastVersion = queryVersion;
            return false;
        }

        if (collider.CircleQueryVersion == queryVersion)
            return true;

        collider.CircleQueryVersion = queryVersion;
        return false;
    }

    private void RefreshDeferredColliderPartitions()
    {
        if (_deferredPartitionRefreshIds.Count == 0)
            return;

        for (int i = 0; i < _deferredPartitionRefreshIds.Count; i++)
        {
            int colliderId = _deferredPartitionRefreshIds.DenseKeys[i];
            if (_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider) && collider!.IsActive)
                RefreshColliderPartition(collider);
        }

        _deferredPartitionRefreshIds.Clear();
    }

    private void CollectCoveredPartitions(
        Vector2d min,
        Vector2d max,
        SwiftList<PhysicsPartition2D> partitions)
    {
        partitions.FastClear();
        try
        {
            ScanCoveredQueryPartitions(min, max, min, max, partitions);
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private void PartitionCoveredVoxels(
        LSCollider2D collider,
        Vector2d coverageMin,
        Vector2d coverageMax,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        PhysicsPartitionMobilityKind kind)
    {
        ScanCoveredColliderVoxels(collider, coverageMin, coverageMax, partitionedCoordinates, kind);
    }

    private void ScanCoveredQueryPartitions(
        Vector2d coverageMin,
        Vector2d coverageMax,
        Vector2d queryMin,
        Vector2d queryMax,
        SwiftList<PhysicsPartition2D> partitions)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            coverageMin,
            coverageMax,
            _coveredVoxels,
            _traceScratch,
            layerY: Fixed64.Zero);

        VisitPlanarVoxelsForQuery(world, queryMin, queryMax, partitions);
    }

    private void ScanCoveredColliderVoxels(
        LSCollider2D collider,
        Vector2d coverageMin,
        Vector2d coverageMax,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        PhysicsPartitionMobilityKind kind)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            coverageMin,
            coverageMax,
            _coveredVoxels,
            _traceScratch,
            layerY: Fixed64.Zero);

        VisitPlanarVoxelsForCollider(world, collider, partitionedCoordinates, kind);
    }

    private void VisitPlanarVoxelsForQuery(
        GridWorld world,
        Vector2d queryMin,
        Vector2d queryMax,
        SwiftList<PhysicsPartition2D> partitions)
    {
        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.PlanarMaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];

            if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
                || !GridTraversal.IsPlanarPositionInPaddedBounds(queryMin, queryMax, cellEdge, voxel.WorldPosition)
                || !voxel.TryGetPartition(out PhysicsPartition2D? partition)
                || partition!.IsEmpty)
            {
                continue;
            }

            partitions.Add(partition);
        }
    }

    private void VisitPlanarVoxelsForCollider(
        GridWorld world,
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        PhysicsPartitionMobilityKind kind)
    {
        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.PlanarMaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartitionVoxel(collider, partitionedCoordinates, _coveredVoxels[i], ref traversal, kind);
    }

    private void TryPartitionVoxel(
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Voxel voxel,
        ref GridTraversalState traversal,
        PhysicsPartitionMobilityKind kind)
    {
        if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
            || !collider.IsPositionInPlanarBounds(cellEdge, voxel.WorldPosition))
        {
            return;
        }

        if (!voxel.TryGetPartition(out PhysicsPartition2D? partition))
        {
            partition = RentPartition();
            if (!voxel.TryAddPartition(partition))
            {
                ReleasePartition(partition);
                return;
            }

            TrackRetainedPartition(partition);
        }

        partitionedCoordinates.Add(voxel.WorldIndex);
        AddObject(partition!, collider.Id, kind);
    }

    private void GetPlanarCoverageBounds(LSCollider2D collider, out Vector2d coverageMin, out Vector2d coverageMax)
    {
        coverageMin = new Vector2d(collider.MinX, collider.MinY);
        coverageMax = new Vector2d(collider.MaxX, collider.MaxY);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal int ResolvePartitionKind(LSCollider2D collider) => (int)GetMobilityKind(collider);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PhysicsPartitionMobilityKind GetMobilityKind(LSCollider2D collider)
    {
        if (collider.IsStatic)
            return PhysicsPartitionMobilityKind.Static;

        SolidBody2D? body = collider.Body;
        return body!.IsKinematic ? PhysicsPartitionMobilityKind.Kinematic : PhysicsPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PhysicsPartitionMobilityKind GetStoredMobilityKind(int partitionKind)
    {
        return partitionKind == (int)PhysicsPartitionMobilityKind.Kinematic
            ? PhysicsPartitionMobilityKind.Kinematic
            : partitionKind == (int)PhysicsPartitionMobilityKind.Static
                ? PhysicsPartitionMobilityKind.Static
                : PhysicsPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddObject(PhysicsPartition2D partition, int id, PhysicsPartitionMobilityKind kind)
    {
        if (kind == PhysicsPartitionMobilityKind.Static)
        {
            partition.AddStaticObject(id);
            return;
        }

        if (kind == PhysicsPartitionMobilityKind.Kinematic)
        {
            partition.AddKinematicObject(id);
            return;
        }

        partition.AddDynamicObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RemoveObject(PhysicsPartition2D partition, int id, PhysicsPartitionMobilityKind kind)
    {
        if (kind == PhysicsPartitionMobilityKind.Static)
        {
            partition.RemoveStaticObject(id);
            return;
        }

        if (kind == PhysicsPartitionMobilityKind.Kinematic)
        {
            partition.RemoveKinematicObject(id);
            return;
        }

        partition.RemoveDynamicObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsStaticStyleCollider(LSCollider2D collider)
    {
        SolidBody2D? body = collider.Body;
        return collider.IsStatic || body!.IsKinematic;
    }

    private void DetachRetainedPartitions() => RetainedPartitionLifecycle.DetachAll(
        _retainedPartitions,
        _context.World,
        this,
        _releaseRetainedPartition,
        nameof(PhysicsPartition2D),
        "Unable to detach retained 2D physics partition from its voxel during reset.");

    private void TrackRetainedPartition(PhysicsPartition2D partition) => RetainedPartitionLifecycle.Track(
        _retainedPartitions,
        this,
        partition,
        nameof(PhysicsPartition2D));

    private void UntrackRetainedPartition(PhysicsPartition2D partition) => RetainedPartitionLifecycle.Untrack(
        _retainedPartitions,
        this,
        partition,
        ref _retainedPartitionRetirementCursor);

    internal void RetireExpiredRetainedPartitions() => RetainedPartitionLifecycle.RetireExpired(
            _retainedPartitions,
            _context.World,
            this,
            _context.Settings.RetainedPartitionRetirementSweepBudget,
            _context.FrameCount,
            _context.Settings.RetainedPartitionTimeToKillFrames,
            _releaseRetainedPartition,
            ref _retainedPartitionRetirementCursor);

    internal int ActivatePartition(PhysicsPartition2D partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "2D partition must belong to this collision service.");

        return _activePartitions.Add(partition);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void DeactivatePartition(int activationId) => _activePartitions.TryRemoveAt(activationId);

    internal PhysicsPartition2D RentPartition()
    {
        if (_inactivePartitionPool.Count == 0)
            TryRetireEmptyRetainedPartitionForReuse();

        PhysicsPartition2D partition = _inactivePartitionPool.Count > 0
            ? _inactivePartitionPool.Pop()
            : new PhysicsPartition2D();
        partition.SetOwner(this);
        return partition;
    }

    private bool TryRetireEmptyRetainedPartitionForReuse()
    {
        return RetainedPartitionLifecycle.TryRetireEmptyForReuse(
            _retainedPartitions,
            _inactivePartitionPool,
            _context.World,
            this,
            _releaseRetainedPartition,
            ref _retainedPartitionRetirementCursor);
    }

    internal void ReleasePartition(PhysicsPartition2D partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "2D partition must be released through its owning collision service.");

        UntrackRetainedPartition(partition);
        partition.ResetForPool();
        _inactivePartitionPool.Push(partition);
    }

    private sealed class PhysicsPartition2DOrderComparer : IComparer<PhysicsPartition2D>
    {
        public int Compare(PhysicsPartition2D? left, PhysicsPartition2D? right) =>
            WorldVoxelIndexOrdering.ComparePlanar(left!.WorldIndex, right!.WorldIndex);
    }

    private sealed class Collider2DIdComparer : IComparer<LSCollider2D>
    {
        public int Compare(LSCollider2D? left, LSCollider2D? right) =>
            left!.Id.CompareTo(right!.Id);
    }
}
