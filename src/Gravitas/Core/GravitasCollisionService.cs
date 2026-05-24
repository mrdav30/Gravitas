using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;

namespace Gravitas;

/// <summary>
/// Owns collision partitioning state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCollisionService
{
    private const int DefaultPartitionPoolCapacity = 1024;

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsPartition> _activePartitions = new();
    private readonly SwiftStack<PhysicsPartition> _inactivePartitionPool = new(DefaultPartitionPoolCapacity);
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly SwiftHashSet<ushort> _processedGrids = new();
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
        _processedGrids.Clear();
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

        try
        {
            PartitionCoveredVoxels(collider, partitionedCoordinates);
            return partitionedCoordinates.Count > 0;
        }
        finally
        {
            _redundancyChecker.Clear();
            _processedGrids.Clear();
        }
    }

    private void PartitionCoveredVoxels(
        LSCollider collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates)
    {
        GridWorld world = _context.World;
        (Vector3d snappedMin, Vector3d snappedMax) =
            world.SnapBoundsToVoxelSize(collider.BoundsMin, collider.BoundsMax, Fixed64.Half);

        GetSpatialCellBounds(
            world,
            snappedMin,
            snappedMax,
            out int xMin,
            out int yMin,
            out int zMin,
            out int xMax,
            out int yMax,
            out int zMax);

        for (int z = zMin; z <= zMax; z++)
        {
            for (int y = yMin; y <= yMax; y++)
            {
                for (int x = xMin; x <= xMax; x++)
                {
                    int cellIndex = SwiftHashTools.CombineHashCodes(x, y, z);
                    if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
                        continue;

                    PartitionCoveredCellGrids(world, collider, partitionedCoordinates, snappedMin, snappedMax, gridList);
                }
            }
        }
    }

    private void PartitionCoveredCellGrids(
        GridWorld world,
        LSCollider collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Vector3d snappedMin,
        Vector3d snappedMax,
        SwiftHashSet<ushort> gridList)
    {
        foreach (ushort gridIndex in gridList)
        {
            if (!world.ActiveGrids.IsAllocated(gridIndex) || !_processedGrids.Add(gridIndex))
                continue;

            VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
            PartitionCoveredGridVoxels(world, currentGrid, collider, partitionedCoordinates, snappedMin, snappedMax);
        }
    }

    private void PartitionCoveredGridVoxels(
        GridWorld world,
        VoxelGrid currentGrid,
        LSCollider collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Vector3d snappedMin,
        Vector3d snappedMax)
    {
        Fixed64 voxelSize = world.VoxelSize;
        for (Fixed64 x = snappedMin.x; x <= snappedMax.x; x += voxelSize)
        {
            for (Fixed64 y = snappedMin.y; y <= snappedMax.y; y += voxelSize)
            {
                for (Fixed64 z = snappedMin.z; z <= snappedMax.z; z += voxelSize)
                    TryPartitionVoxel(currentGrid, collider, partitionedCoordinates, new Vector3d(x, y, z), voxelSize);
            }
        }
    }

    private void TryPartitionVoxel(
        VoxelGrid currentGrid,
        LSCollider collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Vector3d position,
        Fixed64 voxelSize)
    {
        if (!currentGrid.TryGetVoxel(position, out Voxel? voxel)
            || !_redundancyChecker.Add(voxel!.SpawnToken)
            || !collider.IsPositionInBounds(voxelSize, voxel.WorldPosition))
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
        }

        partitionedCoordinates.Add(voxel.WorldIndex);
        if (collider.Body != null && collider.Body.Immovable)
            partition!.AddStaticObject(collider.Id);
        else
            partition!.AddDynamicObject(collider.Id);
    }

    private static void GetSpatialCellBounds(
        GridWorld world,
        Vector3d min,
        Vector3d max,
        out int xMin,
        out int yMin,
        out int zMin,
        out int xMax,
        out int yMax,
        out int zMax)
    {
        SnapToSpatialGrid(world, min, out xMin, out yMin, out zMin);
        SnapToSpatialGrid(world, max, out xMax, out yMax, out zMax);

        if (xMin > xMax)
            (xMin, xMax) = (xMax, xMin);
        if (yMin > yMax)
            (yMin, yMax) = (yMax, yMin);
        if (zMin > zMax)
            (zMin, zMax) = (zMax, zMin);
    }

    private static void SnapToSpatialGrid(GridWorld world, Vector3d position, out int x, out int y, out int z)
    {
        x = (position.x.Abs() / world.SpatialGridCellSize).FloorToInt() * position.x.Sign();
        y = (position.y.Abs() / world.SpatialGridCellSize).FloorToInt() * position.y.Sign();
        z = (position.z.Abs() / world.SpatialGridCellSize).FloorToInt() * position.z.Sign();
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

        foreach (PhysicsPartition partition in _activePartitions)
            partition.Distribute();
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

        partition.ResetForPool();
        _inactivePartitionPool.Push(partition);
    }
}
