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
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly SwiftHashSet<WorldVoxelIndex> _nextCoveredCoordinates = new();
    private readonly GridTraceScratch _traceScratch = new();
    private readonly SwiftList<PhysicsPartition2D> _retainedPartitions = new();
    private readonly SwiftList<PhysicsPartition2D> _emptyRetainedPartitions = new();
    private readonly SwiftList<PhysicsPartition2D> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamicIds = new();
    private readonly SwiftList<int> _distributionStaticIds = new();
    private readonly SwiftList<PhysicsPartition2D> _queryPartitions = new();
    private readonly Action<PhysicsPartition2D> _releaseRetainedPartition;
    private readonly SwiftSparseSet _deferredPartitionRefreshIds = new();

    private int _retainedPartitionRetirementCursor;
    private bool _isDistributing;

    /// <summary>Creates the pure 2D collision partition service for a world context.</summary>
    public GravitasCollision2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
        _releaseRetainedPartition = ReleasePartition;
    }

    /// <summary>Gets the world context that owns this service.</summary>
    public GravitasWorldContext Context => _context;

    /// <summary>Gets the current partition-distribution pass version.</summary>
    public uint Version { get; private set; } = 1;

    /// <summary>Gets the number of partitions participating in collision work.</summary>
    public int ActivePartitionCount => _activePartitions.Count;

    /// <summary>Gets the number of partitions available for reuse.</summary>
    public int InactivePartitionCount => _inactivePartitionPool.Count;

    internal int RetainedPartitionCount => _retainedPartitions.Count;

    /// <summary>Clears all context-local 2D partitioning state.</summary>
    public void Reset()
    {
        DetachRetainedPartitions();
        _activePartitions.Clear();
        _coveredVoxels.FastClear();
        _nextCoveredCoordinates.Clear();
        _traceScratch.Clear();
        _distributionPartitions.FastClear();
        _distributionDynamicIds.FastClear();
        _distributionStaticIds.FastClear();
        _queryPartitions.FastClear();
        _deferredPartitionRefreshIds.Clear();
        _inactivePartitionPool.Clear();
        Version = 1;
        _retainedPartitionRetirementCursor = 0;
        _isDistributing = false;
    }

    internal bool RefreshColliderPartition(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsActive)
        {
            ClearPartitionedCollider(collider, force: true);
            collider.MarkUnpartitioned();
            collider.ClearPartitionCoordinates();

            return false;
        }

        GetPlanarCoverageBounds(collider, out Vector2d coverageMin, out Vector2d coverageMax);
        PhysicsPartitionMobilityKind kind = GetMobilityKind(collider);
        if (collider.MatchesPartitionGridBounds(coverageMin, coverageMax, (int)kind))
            return false;

        return PartitionCollider(collider, coverageMin, coverageMax, kind);
    }

    internal bool RefreshColliderPartitionAfterShapeChange(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!_isDistributing)
        {
            _deferredPartitionRefreshIds.Remove(collider.Id);

            return RefreshColliderPartition(collider);
        }

        _deferredPartitionRefreshIds.Add(collider.Id);

        return false;
    }

    internal bool PartitionCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (collider.IsPartitioned)
            return false;

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
        if (!collider.IsActive)
            return false;

        SwiftList<WorldVoxelIndex> partitionedCoordinates = collider.GetOrCreatePartitionCoordinates();
        PartitionCoveredVoxels(collider, coverageMin, coverageMax, partitionedCoordinates, kind);
        if (partitionedCoordinates.Count == 0)
        {
            collider.MarkUnpartitioned();
            return false;
        }

        collider.MarkPartitioned(coverageMin, coverageMax, (int)kind);
        return true;
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
        PhysicsPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.PartitionKind);
        RemovePartitionMemberships(collider, coordinates, partitionKind, preserveCovered: false);

        collider.MarkUnpartitioned();
        collider.ClearPartitionCoordinates();
        return true;
    }

    private void RemovePartitionMemberships(
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> coordinates,
        PhysicsPartitionMobilityKind partitionKind,
        bool preserveCovered)
    {
        GridWorld world = _context.World;
        for (int i = 0; i < coordinates.Count; i++)
        {
            WorldVoxelIndex coordinate = coordinates[i];
            if (preserveCovered && _nextCoveredCoordinates.Contains(coordinate))
                continue;

            if (!world.ActiveGrids.IsAllocated(coordinate.GridIndex)
                || !world.TryGetVoxel(coordinate, out Voxel? voxel)
                || !voxel!.TryGetPartition(out PhysicsPartition2D? partition))
            {
                continue;
            }

            RemoveObject(partition!, collider.Id, partitionKind);
        }
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

        for (int i = 0; i < coordinates.Count; i++)
        {
            WorldVoxelIndex coordinate = coordinates[i];
            if (!world.ActiveGrids.IsAllocated(coordinate.GridIndex)
                || !world.TryGetVoxel(coordinate, out Voxel? voxel)
                || !voxel!.TryGetPartition(out PhysicsPartition2D? partition))
            {
                continue;
            }

            partition!.SetDynamicObjectAwake(collider.Id, awake);
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

    internal void CollectBoundsCandidates(
        Vector2d min,
        Vector2d max,
        PhysicsLayerMask layerMask,
        uint queryVersion,
        bool raycastQuery,
        SwiftList<LSCollider2D> candidates,
        bool staticStyleOnly = false)
    {
        _context.RefreshQueryPartitions();
        RefreshDeferredColliderPartitions();
        candidates.FastClear();

        CollectCoveredPartitions(min, max, _queryPartitions);

        // Gathering has no callbacks or geometry reduction. Canonicalize once
        // after deduplication, before query workers observe any candidate order.
        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsPartition2D partition = _queryPartitions[i];
            if (!staticStyleOnly)
                CollectMembershipCandidates(partition.ContainedDynamicObjects,
                    min, max, layerMask, queryVersion, raycastQuery, candidates);
            CollectMembershipCandidates(partition.ContainedKinematicObjects,
                min, max, layerMask, queryVersion, raycastQuery, candidates);
            CollectMembershipCandidates(partition.ContainedStaticObjects,
                min, max, layerMask, queryVersion, raycastQuery, candidates);
        }

        candidates.SortInPlace(ColliderIdComparer);
    }

    private void CollectMembershipCandidates(
        SwiftHashSet<int>? membership,
        Vector2d min,
        Vector2d max,
        PhysicsLayerMask layerMask,
        uint queryVersion,
        bool raycastQuery,
        SwiftList<LSCollider2D> candidates)
    {
        if (membership == null)
            return;

        foreach (int colliderId in membership)
        {
            if (!_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider)
                || !collider!.IsActive
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

    internal void RefreshDeferredColliderPartitions()
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

    internal void RebuildPartitionsAfterWorldChange()
    {
        DetachRetainedPartitions();
        for (int i = 0; i < _context.Physics2D.ColliderCount; i++)
        {
            LSCollider2D collider = _context.Physics2D.GetColliderByServiceIndex(i);
            collider.MarkUnpartitioned();
            collider.ClearPartitionCoordinates();
            RefreshColliderPartition(collider);
        }
    }

    private void CollectCoveredPartitions(
        Vector2d min,
        Vector2d max,
        SwiftList<PhysicsPartition2D> partitions)
    {
        partitions.FastClear();
        ScanCoveredQueryPartitions(min, max, min, max, partitions);
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

            Fixed64 cellEdge = traversal.GetCellEdge(voxel);
            if (!GridTraversal.IsPlanarPositionInPaddedBounds(queryMin, queryMax, cellEdge, voxel.WorldPosition)
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
        _nextCoveredCoordinates.Clear();
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];
            if (collider.IsPositionInPlanarBounds(traversal.GetCellEdge(voxel), voxel.WorldPosition))
                _nextCoveredCoordinates.Add(voxel.WorldIndex);
        }

        // Bounds remain exact for versioning. Only memberships shared by both
        // coverage sets survive; a mobility change must replace their old role.
        PhysicsPartitionMobilityKind previousKind = GetStoredMobilityKind(collider.PartitionKind);
        RemovePartitionMemberships(collider, partitionedCoordinates, previousKind, previousKind == kind);
        partitionedCoordinates.FastClear();
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartitionVoxel(collider, partitionedCoordinates, _coveredVoxels[i], kind);
    }

    private void TryPartitionVoxel(
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Voxel voxel,
        PhysicsPartitionMobilityKind kind)
    {
        if (!_nextCoveredCoordinates.Contains(voxel.WorldIndex))
            return;

        if (!voxel.TryGetPartition(out PhysicsPartition2D? partition))
        {
            partition = RentPartition();
            SwiftThrowHelper.ThrowIfTrue(
                !voxel.TryAddPartition(partition),
                nameof(GravitasCollision2DService),
                "Unable to attach 2D physics partition to voxel.");

            TrackRetainedPartition(partition);
        }

        partitionedCoordinates.Add(voxel.WorldIndex);
        AddObject(partition!, collider.Id, kind);
        // A surviving ID bypasses AddDynamicObject's initial awake refresh.
        // Solver mobility may have changed without changing its partition role.
        if (kind == PhysicsPartitionMobilityKind.Dynamic)
            partition!.SetDynamicObjectAwake(collider.Id, collider.Body!.IsAwakeForCollision);
    }

    private void GetPlanarCoverageBounds(LSCollider2D collider, out Vector2d coverageMin, out Vector2d coverageMax)
    {
        coverageMin = new Vector2d(collider.MinX, collider.MinY);
        coverageMax = new Vector2d(collider.MaxX, collider.MaxY);
    }

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

    private void DetachRetainedPartitions() => RetainedPartitionLifecycle.DetachAll(
        _retainedPartitions,
        _emptyRetainedPartitions,
        _context.World,
        this,
        _releaseRetainedPartition,
        nameof(PhysicsPartition2D),
        "Unable to detach retained 2D physics partition from its voxel during reset.");

    private void TrackRetainedPartition(PhysicsPartition2D partition) => RetainedPartitionLifecycle.Track(
        _retainedPartitions,
        _emptyRetainedPartitions,
        this,
        partition,
        nameof(PhysicsPartition2D));

    private void UntrackRetainedPartition(PhysicsPartition2D partition) => RetainedPartitionLifecycle.Untrack(
        _retainedPartitions,
        _emptyRetainedPartitions,
        this,
        partition,
        ref _retainedPartitionRetirementCursor);

    internal void RefreshRetainedPartitionEligibility(PhysicsPartition2D partition) => RetainedPartitionLifecycle.RefreshEmptyEligibility(
        _emptyRetainedPartitions, this, partition);

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
            _emptyRetainedPartitions,
            _inactivePartitionPool,
            _context.World,
            this,
            _releaseRetainedPartition);
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
