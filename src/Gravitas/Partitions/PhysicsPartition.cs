using Gravitas.CollisionHandling;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;

namespace Gravitas;

public class PhysicsPartition : IVoxelPartition
{
    public WorldVoxelIndex WorldIndex { get; set; }

    public bool IsPartitioned { get; set; }

    /// <summary>
    /// Stores dynamic bodies' PhysicsManager IDs.
    /// </summary>
    public SwiftList<int>? ContainedDynamicObjects;

    public SwiftList<int>? ContainedStaticObjects;

    public int ActivationId { get; private set; }

    private static int _id1, _id2;

    private static CollisionPair? _pair;

    public bool IsAllocated => ActivationId != -1;

    public PhysicsPartition() { }

    public void OnAddToVoxel(Voxel voxel) => WorldIndex = voxel.WorldIndex;

    public void OnChange() { }

    public void Distribute()
    {
        int dynamicCount = ContainedDynamicObjects?.Count ?? 0;
        if (ContainedDynamicObjects == null || dynamicCount == 0)
            return;

        int staticCount = ContainedStaticObjects?.Count ?? 0;

        // only distribute when there are dynamic objects on the same partition
        for (int j = 0; j < dynamicCount; j++)
        {
            _id1 = ContainedDynamicObjects[j];
            for (int k = j + 1; k < dynamicCount; k++)
            {
                _id2 = ContainedDynamicObjects[k];
                if (_id1 != _id2)
                    ProcessPair();
            }

            for (int k = 0; k < staticCount; k++)
            {
                _id2 = ContainedStaticObjects![k];
                ProcessPair();
            }
        }
    }

    private void ProcessPair()
    {
        _pair = PhysicsManager.GetCollisionPair(_id1, _id2);

        //Ensures collision pairs are not run twice
        if (_pair == null || _pair.PartitionVersion == CollisionManager.Version)
            return;

        _pair.PartitionVersion = CollisionManager.Version;
        _pair.UpdateCollision();
    }

    public void AddDynamicObject(int item)
    {
        if (ContainedDynamicObjects?.Contains(item) == true)
            return;

        if (ContainedDynamicObjects?.Count == 0)
            ActivationId = CollisionManager.ActivatePartitions(this);

        ContainedDynamicObjects ??= new();
        ContainedDynamicObjects.Add(item);
    }

    public void AddStaticObject(int item)
    {
        if (ContainedStaticObjects?.Contains(item) == true)
            return;

        ContainedStaticObjects ??= new();
        ContainedStaticObjects.Add(item);
    }

    public void RemoveDynamicObject(int item)
    {
        if (ActivationId == -1)
            return;

        //todo get rid of this linear search
        if (ContainedDynamicObjects?.Remove(item) == false)
        {
            GravitasLogger.DebugChannel.Info($"Dynamic item not removed - {item}");
            return;
        }

        if (ContainedDynamicObjects?.Count > 0)
            return;

        // If there are no more dynamic objects, we can deactivate the partition to save on future checks until it's needed again.
        CollisionManager.RemoveActivatedNode(ActivationId);
        ActivationId = -1;
    }

    public void RemoveStaticObject(int item)
    {
        if (ContainedStaticObjects?.Remove(item) == false)
            GravitasLogger.DebugChannel.Info($"Static item not removed - {item}");
    }

    public void OnRemoveFromVoxel(Voxel voxel) => Reset();

    public void Reset()
    {
        ContainedDynamicObjects?.FastClear();
        ContainedStaticObjects?.FastClear();

        if (ActivationId != -1)
            CollisionManager.RemoveActivatedNode(ActivationId);

        ActivationId = -1;
    }

    /// <summary>
    /// Sets the parent index for the current voxel in the world.
    /// </summary>
    /// <param name="parentIndex">The index to assign as the parent of the current voxel.</param>
    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;
}