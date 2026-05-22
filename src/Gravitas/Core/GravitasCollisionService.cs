using FixedMathSharp;
using Gravitas.Colliders;
using GridForge;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using SwiftCollections.Pool;
using System;

namespace Gravitas;

/// <summary>
/// Owns collision partitioning state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCollisionService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsPartition> _activePartitions = new();
    private readonly SwiftObjectPool<PhysicsPartition> _inactivePartitionPool;
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly object _cullDistributorLock = new();

    private int _cullDistributor;

    /// <summary>
    /// Initializes a new collision service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasCollisionService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
        _inactivePartitionPool = new SwiftObjectPool<PhysicsPartition>(
            createFunc: () => new PhysicsPartition(),
            actionOnRelease: partition => partition.ResetForPool());
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
    public int InactivePartitionCount => _inactivePartitionPool.CountInactive;

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
        _activePartitions.Clear();
        _redundancyChecker.Clear();
        _inactivePartitionPool.Clear();
        Version = 1;
        _cullDistributor = 0;
    }

    /// <summary>
    /// Deactivates this collision service and clears pooled state.
    /// </summary>
    public void Deactivate() => Reset();

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

        GridWorld world = _context.World;
        Fixed64 voxelSize = world.VoxelSize;
        foreach (GridVoxelSet gridVoxelSet in GridTracer.GetCoveredVoxels(world, collider.BoundsMin, collider.BoundsMax, Fixed64.Half))
        {
            foreach (Voxel voxel in gridVoxelSet.Voxels)
            {
                if (!_redundancyChecker.Add(voxel.SpawnToken) || !collider.IsPositionInBounds(voxelSize, voxel.WorldPosition))
                    continue;

                if (!voxel.TryGetPartition(out PhysicsPartition? partition))
                {
                    partition = RentPartition();
                    if (!voxel.TryAddPartition(partition))
                    {
                        ReleasePartition(partition);
                        continue;
                    }
                }

                partitionedCoordinates.Add(voxel.WorldIndex);
                if (collider.Body != null && collider.Body.Immovable)
                    partition!.AddStaticObject(collider.Id);
                else
                    partition!.AddDynamicObject(collider.Id);
            }
        }

        _redundancyChecker.Clear();
        return partitionedCoordinates.Count > 0;
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

        (Vector3d snappedMin, Vector3d snappedMax) =
            world.SnapBoundsToVoxelSize(collider.BoundsMin, collider.BoundsMax, Fixed64.Half);

        if (!force && collider.LastGridBoundsMin == snappedMin && collider.LastGridBoundsMax == snappedMax)
            return false;

        bool isStatic = collider.Body != null && collider.Body.Immovable;

        for (int i = 0; i < collider.PartitionCoordinates!.Count; i++)
        {
            WorldVoxelIndex coordinate = collider.PartitionCoordinates[i];
            if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                || !_redundancyChecker.Add(voxel!.SpawnToken)
                || !voxel.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            if (isStatic)
                partition!.RemoveStaticObject(collider.Id);
            else
                partition!.RemoveDynamicObject(collider.Id);

            if ((partition!.ContainedDynamicObjects?.Count ?? 0) == 0
                && (partition.ContainedStaticObjects?.Count ?? 0) == 0)
            {
                voxel.TryRemovePartition<PhysicsPartition>();
            }
        }

        _redundancyChecker.Clear();

        return true;
    }

    internal void CheckAndDistributeCollisions()
    {
        Version++;

        for (int i = 0; i < _activePartitions.PeakCount; i++)
        {
            if (_activePartitions.IsAllocated(i))
                _activePartitions[i].Distribute();
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
        PhysicsPartition partition = _inactivePartitionPool.Rent();
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

        _inactivePartitionPool.Release(partition);
    }
}
