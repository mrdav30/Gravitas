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
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly GridTraceScratch _traceScratch = new();
    private readonly SwiftList<PhysicsPartition> _retainedPartitions = new();
    private readonly SwiftList<PhysicsPartition> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamicIds = new();
    private readonly SwiftList<int> _distributionStaticIds = new();
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
        _redundancyChecker.Clear();
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

    private void DetachRetainedPartitions()
    {
        // Reset is a context boundary; retained GridForge payloads are a runtime cache, not replay state.
        while (_retainedPartitions.Count > 0)
        {
            PhysicsPartition partition = _retainedPartitions[_retainedPartitions.Count - 1];
            if (!partition.IsOwnedBy(this))
            {
                UntrackRetainedPartition(partition);
                continue;
            }

            if (_context.World.TryGetVoxel(partition.WorldIndex, out Voxel? voxel)
                && voxel!.TryGetPartition(out PhysicsPartition? attachedPartition)
                && ReferenceEquals(attachedPartition, partition))
            {
                bool removed = voxel.TryRemovePartition<PhysicsPartition>();
                SwiftThrowHelper.ThrowIfTrue(
                    !removed,
                    nameof(PhysicsPartition),
                    "Unable to detach retained physics partition from its voxel during reset.");

                if (partition.IsOwnedBy(this))
                    ReleasePartition(partition);

                continue;
            }

            ReleasePartition(partition);
        }
    }

    private void TrackRetainedPartition(PhysicsPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            partition.RetainedIndex >= 0,
            nameof(partition),
            "PhysicsPartition is already tracked as retained.");

        partition.SetRetainedIndex(_retainedPartitions.Count);
        _retainedPartitions.Add(partition);
    }

    private void UntrackRetainedPartition(PhysicsPartition partition)
    {
        int index = FindRetainedPartitionIndex(partition);
        if (index < 0)
        {
            partition.ClearRetainedIndex();
            return;
        }

        int lastIndex = _retainedPartitions.Count - 1;
        if (index != lastIndex)
        {
            PhysicsPartition movedPartition = _retainedPartitions[lastIndex];
            _retainedPartitions[index] = movedPartition;
            movedPartition.SetRetainedIndex(index);
        }

        _retainedPartitions.RemoveAt(lastIndex);
        partition.ClearRetainedIndex();

        if (_retainedPartitionRetirementCursor > index)
            _retainedPartitionRetirementCursor--;
        if (_retainedPartitionRetirementCursor >= _retainedPartitions.Count)
            _retainedPartitionRetirementCursor = 0;
    }

    private int FindRetainedPartitionIndex(PhysicsPartition partition)
    {
        int index = partition.RetainedIndex;
        if ((uint)index < (uint)_retainedPartitions.Count && ReferenceEquals(_retainedPartitions[index], partition))
            return index;

        for (int i = 0; i < _retainedPartitions.Count; i++)
            if (ReferenceEquals(_retainedPartitions[i], partition))
                return i;

        return -1;
    }

    /// <summary>
    /// Deactivates this collision service and clears pooled state.
    /// </summary>
    public void Deactivate() => Reset();

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

        if (collider.IsPartitioned || collider.World == null)
            return false;

        partitionedCoordinates.FastClear();

        try
        {
            PartitionCoveredVoxels(collider, partitionedCoordinates, GetMobilityKind(collider));
            return partitionedCoordinates.Count > 0;
        }
        finally
        {
            _redundancyChecker.Clear();
        }
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
        if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
            || !collider.IsPositionInBounds(cellEdge, voxel.WorldPosition))
        {
            return;
        }

        if (!voxel.TryGetPartition(out PhysicsPartition? partition))
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

    internal bool ClearPartitionedObject(LSCollider collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "Collider must belong to this collision service context.");

        GridWorld world = _context.World;
        if (!collider.IsPartitioned)
        {
            GravitasLogger.Channel.Error($"Attempted to clear partitions for a non-partitioned collider! - {collider}");
            return false;
        }

        PhysicsPartitionMobilityKind currentKind = GetMobilityKind(collider);
        if (!force && collider.MatchesPartitionGridBounds(collider.BoundsMin, collider.BoundsMax, (int)currentKind))
            return false;

        PhysicsPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.PartitionKind);

        for (int i = 0; i < collider.PartitionCoordinates!.Count; i++)
        {
            WorldVoxelIndex coordinate = collider.PartitionCoordinates[i];
            if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsPartition? partition))
            {
                continue;
            }

            RemoveObject(partition!, collider.Id, partitionKind);

            // Keep the voxel partition attached after it becomes empty. Re-adding
            // the same partition type through GridForge carries metadata overhead,
            // while an empty PhysicsPartition is inactive and query-invisible.
        }

        _redundancyChecker.Clear();

        return true;
    }

    internal void RefreshPartitionAwakeState(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "Collider must belong to this collision service context.");

        if (!collider.IsPartitioned || collider.PartitionCoordinates == null)
            return;

        SolidBody? body = collider.Body;
        if (body == null || body.Immovable || body.IsKinematic)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;

        try
        {
            for (int i = 0; i < collider.PartitionCoordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = collider.PartitionCoordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsPartition? partition))
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

    internal void RetireExpiredRetainedPartitions()
    {
        int budget = _context.Settings.RetainedPartitionRetirementSweepBudget;
        if (budget <= 0 || _retainedPartitions.Count == 0)
            return;

        int inspected = 0;
        while (inspected < budget && _retainedPartitions.Count > 0)
        {
            if (_retainedPartitionRetirementCursor >= _retainedPartitions.Count)
                _retainedPartitionRetirementCursor = 0;

            PhysicsPartition partition = _retainedPartitions[_retainedPartitionRetirementCursor];
            inspected++;

            if (!ShouldRetireRetainedPartition(partition))
            {
                _retainedPartitionRetirementCursor++;
                continue;
            }

            if (!RetireRetainedPartition(partition))
                _retainedPartitionRetirementCursor++;
        }
    }

    private bool ShouldRetireRetainedPartition(PhysicsPartition partition)
    {
        if (!partition.IsOwnedBy(this) || !partition.IsEmpty || partition.IsAllocated || partition.EmptySinceFrame < 0)
            return false;

        int idleFrames = _context.FrameCount - partition.EmptySinceFrame;
        return idleFrames >= _context.Settings.RetainedPartitionTimeToKillFrames;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PhysicsPartitionMobilityKind GetMobilityKind(LSCollider collider)
    {
        SolidBody? body = collider.Body;
        if (body == null || body.Immovable)
            return PhysicsPartitionMobilityKind.Static;

        return body.IsKinematic ? PhysicsPartitionMobilityKind.Kinematic : PhysicsPartitionMobilityKind.Dynamic;
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

    private bool RetireRetainedPartition(PhysicsPartition partition)
    {
        if (!_context.World.TryGetVoxel(partition.WorldIndex, out Voxel? voxel))
        {
            ReleasePartition(partition);
            return true;
        }

        if (!voxel!.TryGetPartition(out PhysicsPartition? attachedPartition)
            || !ReferenceEquals(attachedPartition, partition))
        {
            ReleasePartition(partition);
            return true;
        }

        return voxel.TryRemovePartition<PhysicsPartition>();
    }

    private sealed class PhysicsPartitionOrderComparer : IComparer<PhysicsPartition>
    {
        public int Compare(PhysicsPartition? left, PhysicsPartition? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            WorldVoxelIndex leftIndex = left.WorldIndex;
            WorldVoxelIndex rightIndex = right.WorldIndex;

            int compare = leftIndex.GridIndex.CompareTo(rightIndex.GridIndex);
            if (compare != 0)
                return compare;

            compare = leftIndex.GridSpawnToken.CompareTo(rightIndex.GridSpawnToken);
            if (compare != 0)
                return compare;

            compare = leftIndex.VoxelIndex.x.CompareTo(rightIndex.VoxelIndex.x);
            if (compare != 0)
                return compare;

            compare = leftIndex.VoxelIndex.y.CompareTo(rightIndex.VoxelIndex.y);
            if (compare != 0)
                return compare;

            return leftIndex.VoxelIndex.z.CompareTo(rightIndex.VoxelIndex.z);
        }
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

    internal void DeactivatePartition(int activationId)
    {
        if (activationId < 0)
            return;

        _activePartitions.TryRemoveAt(activationId);
    }

    internal PhysicsPartition RentPartition()
    {
        PhysicsPartition partition = _inactivePartitionPool.Count > 0
            ? _inactivePartitionPool.Pop()
            : new PhysicsPartition();
        partition.SetOwner(this);
        return partition;
    }

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
