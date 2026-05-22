using FixedMathSharp;
using Gravitas.Colliders;
using GridForge;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using SwiftCollections.Pool;

//TODO: pool partitions similiar to gridmanager

namespace Gravitas;

public static class CollisionManager
{
    private static uint _version = 1;
    public static uint Version => _version;

    /// <summary>
    /// Stores all Partitions that contain "active" objects that need to perform collision detection
    /// </summary>
    private static SwiftBucket<PhysicsPartition>? _activePartitions;

    /// <summary>
    /// Used to pool unused Partitions (replace with pooling!)
    /// </summary>
    private static readonly SwiftObjectPool<PhysicsPartition> _inactivePartitionPool = new(
            createFunc: () => new PhysicsPartition(),
            actionOnRelease: partition => partition.Reset()
        );

    private static SwiftHashSet<int>? _redundancyChecker;

    public static void Setup()
    {
        _activePartitions = new SwiftBucket<PhysicsPartition>();
        _redundancyChecker = new SwiftHashSet<int>();

        _version = 1;
    }

    public static void Initialize()
    {
        _activePartitions ??= new();
        _redundancyChecker ??= new();

        _activePartitions?.Clear();
        _redundancyChecker?.Clear();
    }

    public static void Deactivate()
    {
        _activePartitions?.Clear();
        _redundancyChecker?.Clear();
        _inactivePartitionPool.Clear();
    }

    public static bool PartitionObject(
        LSCollider collider,
        ref SwiftList<WorldVoxelIndex> partitionedCoordinates)
    {
        if (collider.IsPartitioned || collider.World == null)
            return false;

        partitionedCoordinates.FastClear();
        _redundancyChecker ??= new();

        GridWorld world = collider.World;
        Fixed64 voxelSize = world.VoxelSize;
        foreach (GridVoxelSet gridVoxelSet in GridTracer.GetCoveredVoxels(world, collider.BoundsMin, collider.BoundsMax, Fixed64.Half))
        {
            foreach (Voxel voxel in gridVoxelSet.Voxels)
            {
                if (!_redundancyChecker.Add(voxel.SpawnToken) || !collider.IsPositionInBounds(voxelSize, voxel.WorldPosition))
                    continue;

                if (!voxel.TryGetPartition(out PhysicsPartition? partition))
                {
                    partition = _inactivePartitionPool.Rent();
                    voxel.TryAddPartition(partition);
                    // TODO: should we also call node.addobstacle here to imitiate GridForge.DynamicBlocker?
                    // maybe add a setting to LSBody
                    // per old DynamicBlocker - A dynamic blocker that moves with an object, updating grid obstacles as needed.
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

    public static bool ClearPartitionedObject(LSCollider collider, bool force = false)
    {
        GridWorld? world = collider.World;
        if (world == null || !collider.IsPartitioned)
        {
            GravitasLogger.Channel.Error($"Attempted to clear partitions for a non-partitioned collider! - {collider}");
            return false;
        }

        (Vector3d snappedMin, Vector3d snappedMax) =
            world.SnapBoundsToVoxelSize(collider.BoundsMin, collider.BoundsMax, Fixed64.Half);

        if (!force && collider.LastGridBoundsMin == snappedMin && collider.LastGridBoundsMax == snappedMax)
            return false;

        bool isStatic = collider.Body != null && collider.Body.Immovable;

        _redundancyChecker ??= new();

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

            if (partition!.ContainedDynamicObjects?.Count == 0 && partition.ContainedStaticObjects?.Count == 0)
                PoolNodePartition(voxel, partition); // We don't need this node anymore then
        }

        _redundancyChecker.Clear();

        return true;
    }

    public static int ActivatePartitions(PhysicsPartition partition)
    {
        _activePartitions ??= new();
        return _activePartitions.Add(partition);
    }

    public static void RemoveActivatedNode(int id) => _activePartitions?.TryRemoveAt(id);

    public static void PoolNodePartition(Voxel node, PhysicsPartition partition)
    {
        node.TryRemovePartition<PhysicsPartition>();
        _inactivePartitionPool.Release(partition); // release and reset partition back to the pool
    }

    public static void CheckAndDistributeCollisions()
    {
        _version++;

        if (_activePartitions == null)
            return;

        foreach (PhysicsPartition partition in _activePartitions)
            partition.Distribute();
    }
}