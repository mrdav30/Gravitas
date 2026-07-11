//=======================================================================
// GravitasCollisionService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns collision partitioning state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCollisionService
{
    private const int DefaultPartitionPoolCapacity = 1024;
    private static readonly PhysicsPartitionOrderComparer PartitionOrderComparer = new();

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsPartition> _activePartitions = new();
    private readonly SwiftStack<PhysicsPartition> _inactivePartitionPool = new(DefaultPartitionPoolCapacity);
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly GridTraceScratch _traceScratch = new();
    private readonly SwiftList<PhysicsPartition> _retainedPartitions = new();
    private readonly SwiftList<PhysicsPartition> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamicIds = new();
    private readonly SwiftList<int> _distributionStaticIds = new();
    private readonly Action<PhysicsPartition> _releaseRetainedPartition;
    private readonly object _cullDistributorLock = new();

    private int _cullDistributor;
    private int _retainedPartitionRetirementCursor;

    /// <summary>
    /// Initializes a new collision service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasCollisionService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
        _releaseRetainedPartition = ReleasePartition;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the collision distribution version for this context.
    /// </summary>
    public uint Version { get; private set; } = 1;

    /// <summary>
    /// Gets the number of active partitions in this context.
    /// </summary>
    public int ActivePartitionCount => _activePartitions.Count;

    /// <summary>
    /// Gets the number of inactive partitions currently available for reuse.
    /// </summary>
    public int InactivePartitionCount => _inactivePartitionPool.Count;

    internal int RetainedPartitionCount => _retainedPartitions.Count;

    internal int CullDistributor
    {
        get
        {
            lock (_cullDistributorLock)
            {
                if (_cullDistributor > 1)
                    _cullDistributor = -1;

                return _cullDistributor++;
            }
        }
    }

    /// <summary>
    /// Resets transient collision state owned by this context.
    /// </summary>
    public void Reset()
    {
        DetachRetainedPartitions();
        _activePartitions.Clear();
        _coveredVoxels.FastClear();
        _traceScratch.Clear();
        _distributionPartitions.FastClear();
        _distributionDynamicIds.FastClear();
        _distributionStaticIds.FastClear();
        _inactivePartitionPool.Clear();
        Version = 1;
        _cullDistributor = 0;
        _retainedPartitionRetirementCursor = 0;
    }

    private void DetachRetainedPartitions() => RetainedPartitionLifecycle.DetachAll(
        _retainedPartitions,
        _context.World,
        this,
        _releaseRetainedPartition,
        nameof(PhysicsPartition),
        "Unable to detach retained physics partition from its voxel during reset.");

    private void TrackRetainedPartition(PhysicsPartition partition) => RetainedPartitionLifecycle.Track(
        _retainedPartitions,
        this,
        partition,
        nameof(PhysicsPartition));

    private void UntrackRetainedPartition(PhysicsPartition partition) => RetainedPartitionLifecycle.Untrack(
            _retainedPartitions,
            this,
            partition,
            ref _retainedPartitionRetirementCursor);

    internal bool IsPartitionRefreshRequired(LSCollider collider) =>
        !collider.MatchesPartitionGridBounds(collider.BoundsMin, collider.BoundsMax, ResolvePartitionKind(collider));

    internal int ResolvePartitionKind(LSCollider collider) => (int)GetMobilityKind(collider);

    internal bool PartitionObject(
        LSCollider collider,
        ref SwiftList<WorldVoxelIndex> partitionedCoordinates)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "Collider must belong to this collision service context.");

        partitionedCoordinates.FastClear();

        PartitionCoveredVoxels(collider, partitionedCoordinates, GetMobilityKind(collider));
        return partitionedCoordinates.Count > 0;
    }

    private void PartitionCoveredVoxels(
        LSCollider collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        PhysicsPartitionMobilityKind kind)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            collider.BoundsMin,
            collider.BoundsMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartitionVoxel(collider, partitionedCoordinates, _coveredVoxels[i], ref traversal, kind);
    }

    private void TryPartitionVoxel(
        LSCollider collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Voxel voxel,
        ref GridTraversalState traversal,
        PhysicsPartitionMobilityKind kind)
    {
        Fixed64 cellEdge = traversal.GetCellEdge(voxel);
        if (!collider.IsPositionInBounds(cellEdge, voxel.WorldPosition))
            return;

        if (!voxel.TryGetPartition(out PhysicsPartition? partition))
        {
            partition = RentPartition();
            SwiftThrowHelper.ThrowIfTrue(
                !voxel.TryAddPartition(partition),
                nameof(GravitasCollisionService),
                "Unable to attach 3D physics partition to voxel.");

            TrackRetainedPartition(partition);
        }

        partitionedCoordinates.Add(voxel.WorldIndex);
        AddObject(partition!, collider.Id, kind);
    }

    internal bool ClearPartitionedObject(LSCollider collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "Collider must belong to this collision service context.");

        GridWorld world = _context.World;
        if (!collider.IsPartitioned)
            return false;

        PhysicsPartitionMobilityKind currentKind = GetMobilityKind(collider);
        if (!force && collider.MatchesPartitionGridBounds(collider.BoundsMin, collider.BoundsMax, (int)currentKind))
            return false;

        PhysicsPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.PartitionKind);

        for (int i = 0; i < collider.PartitionCoordinates!.Count; i++)
        {
            WorldVoxelIndex coordinate = collider.PartitionCoordinates[i];
            if (!world.ActiveGrids.IsAllocated(coordinate.GridIndex)
                || !world.TryGetVoxel(coordinate, out Voxel? voxel)
                || !voxel!.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            RemoveObject(partition!, collider.Id, partitionKind);

            // Keep the voxel partition attached after it becomes empty. Re-adding
            // the same partition type through GridForge carries metadata overhead,
            // while an empty PhysicsPartition is inactive and query-invisible.
        }

        collider.MarkUnpartitioned();
        collider.ClearPartitionCoordinates();

        return true;
    }

    internal void RefreshPartitionAwakeState(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "Collider must belong to this collision service context.");

        SwiftList<WorldVoxelIndex> coordinates = collider.PartitionCoordinates!;
        SolidBody? body = collider.Body;
        if (collider.IsStatic || body!.IsKinematic)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;

        for (int i = 0; i < coordinates.Count; i++)
        {
            WorldVoxelIndex coordinate = coordinates[i];
            if (!world.ActiveGrids.IsAllocated(coordinate.GridIndex)
                || !world.TryGetVoxel(coordinate, out Voxel? voxel)
                || !voxel!.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            partition!.SetDynamicObjectAwake(collider.Id, awake);
        }
    }

    internal void CheckAndDistributeCollisions()
    {
        Version++;

        _distributionPartitions.FastClear();
        foreach (PhysicsPartition partition in _activePartitions)
            _distributionPartitions.Add(partition);

        _distributionPartitions.SortInPlace(PartitionOrderComparer);

        for (int i = 0; i < _distributionPartitions.Count; i++)
        {
            _distributionPartitions[i].Distribute(
                _distributionDynamicIds,
                _distributionStaticIds);
        }
    }

    internal void RetireExpiredRetainedPartitions() => RetainedPartitionLifecycle.RetireExpired(
        _retainedPartitions,
        _context.World,
        this,
        _context.Settings.RetainedPartitionRetirementSweepBudget,
        _context.FrameCount,
        _context.Settings.RetainedPartitionTimeToKillFrames,
        _releaseRetainedPartition,
        ref _retainedPartitionRetirementCursor);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PhysicsPartitionMobilityKind GetMobilityKind(LSCollider collider)
    {
        if (collider.IsStatic)
            return PhysicsPartitionMobilityKind.Static;

        SolidBody? body = collider.Body;
        return body!.IsKinematic ? PhysicsPartitionMobilityKind.Kinematic : PhysicsPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PhysicsPartitionMobilityKind GetStoredMobilityKind(int partitionKind) => partitionKind == (int)PhysicsPartitionMobilityKind.Kinematic
        ? PhysicsPartitionMobilityKind.Kinematic
        : partitionKind == (int)PhysicsPartitionMobilityKind.Static
            ? PhysicsPartitionMobilityKind.Static
            : PhysicsPartitionMobilityKind.Dynamic;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddObject(PhysicsPartition partition, int id, PhysicsPartitionMobilityKind kind)
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
    private static void RemoveObject(PhysicsPartition partition, int id, PhysicsPartitionMobilityKind kind)
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

    private sealed class PhysicsPartitionOrderComparer : IComparer<PhysicsPartition>
    {
        public int Compare(PhysicsPartition? left, PhysicsPartition? right) =>
            WorldVoxelIndexOrdering.Compare3D(left!.WorldIndex, right!.WorldIndex);
    }

    internal int ActivatePartition(PhysicsPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "Partition must belong to this collision service.");

        return _activePartitions.Add(partition);
    }

    internal void DeactivatePartition(int activationId) => _activePartitions.TryRemoveAt(activationId);

    internal PhysicsPartition RentPartition()
    {
        if (_inactivePartitionPool.Count == 0)
            TryRetireEmptyRetainedPartitionForReuse();

        PhysicsPartition partition = _inactivePartitionPool.Count > 0
            ? _inactivePartitionPool.Pop()
            : new PhysicsPartition();
        partition.SetOwner(this);
        return partition;
    }

    private bool TryRetireEmptyRetainedPartitionForReuse() => RetainedPartitionLifecycle.TryRetireEmptyForReuse(
            _retainedPartitions,
            _inactivePartitionPool,
            _context.World,
            this,
            _releaseRetainedPartition,
            ref _retainedPartitionRetirementCursor);

    internal void ReleasePartition(PhysicsPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "Partition must be released through its owning collision service.");

        UntrackRetainedPartition(partition);
        partition.ResetForPool();
        _inactivePartitionPool.Push(partition);
    }
}
