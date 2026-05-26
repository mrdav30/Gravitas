using Gravitas.CollisionHandling;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;

namespace Gravitas;

public class PhysicsPartition : IVoxelPartition
{
    private GravitasCollisionService? _owner;

    public WorldVoxelIndex WorldIndex { get; set; }

    public bool IsPartitioned { get; set; }

    /// <summary>
    /// Stores context-local dynamic body IDs.
    /// </summary>
    public SwiftSparseSet? ContainedDynamicObjects;

    public SwiftSparseSet? ContainedStaticObjects;

    public int ActivationId { get; private set; }

    public bool IsAllocated => ActivationId != -1;

    public GravitasCollisionService Owner
    {
        get
        {
            SwiftThrowHelper.ThrowIfTrue(
                _owner == null,
                nameof(PhysicsPartition),
                "PhysicsPartition is missing its owner collision service.");
            return _owner!;
        }
    }

    public PhysicsPartition()
    {
        ActivationId = -1;
    }

    public void OnAddToVoxel(Voxel voxel)
    {
        SwiftThrowHelper.ThrowIfTrue(_owner == null, nameof(PhysicsPartition), "PhysicsPartition is missing its owner collision service.");
        WorldIndex = voxel.WorldIndex;
        IsPartitioned = true;
    }

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
            int id1 = ContainedDynamicObjects.DenseKeys[j];
            for (int k = j + 1; k < dynamicCount; k++)
            {
                int id2 = ContainedDynamicObjects.DenseKeys[k];
                if (id1 != id2)
                    ProcessPair(id1, id2);
            }

            for (int k = 0; k < staticCount; k++)
            {
                int id2 = ContainedStaticObjects!.DenseKeys[k];
                ProcessPair(id1, id2);
            }
        }
    }

    private void ProcessPair(int id1, int id2)
    {
        GravitasCollisionService owner = Owner;
        CollisionPair? pair = owner.Context.Physics.GetCollisionPair(id1, id2);

        //Ensures collision pairs are not run twice
        if (pair == null || pair.PartitionVersion == owner.Version)
            return;

        pair.PartitionVersion = owner.Version;
        pair.UpdateCollision();
    }

    public void AddDynamicObject(int item)
    {
        ContainedDynamicObjects ??= new();
        if (!ContainedDynamicObjects.Add(item))
            return;

        if (ContainedDynamicObjects.Count == 1)
            ActivationId = Owner.ActivatePartition(this);
    }

    public void AddStaticObject(int item)
    {
        ContainedStaticObjects ??= new();
        ContainedStaticObjects.Add(item);
    }

    public void RemoveDynamicObject(int item)
    {
        if (ContainedDynamicObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"Dynamic item not removed - {item}");
            return;
        }

        if (ContainedDynamicObjects?.Count > 0)
            return;

        // If there are no more dynamic objects, we can deactivate the partition to save on future checks until it's needed again.
        Owner.DeactivatePartition(ActivationId);
        ActivationId = -1;
    }

    public void RemoveStaticObject(int item)
    {
        if (ContainedStaticObjects?.Remove(item) != true)
            GravitasLogger.DebugChannel.Info($"Static item not removed - {item}");
    }

    public void OnRemoveFromVoxel(Voxel voxel)
    {
        Owner.ReleasePartition(this);
    }

    internal void SetOwner(GravitasCollisionService owner)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            _owner != null && !ReferenceEquals(_owner, owner),
            nameof(owner),
            "PhysicsPartition is already owned by a different collision service.");

        _owner = owner;
    }

    internal void ResetForPool()
    {
        ContainedDynamicObjects?.Clear();
        ContainedStaticObjects?.Clear();

        if (ActivationId != -1)
            _owner?.DeactivatePartition(ActivationId);

        _owner = null;
        ActivationId = -1;
        IsPartitioned = false;
    }

    /// <summary>
    /// Sets the parent index for the current voxel in the world.
    /// </summary>
    /// <param name="parentIndex">The index to assign as the parent of the current voxel.</param>
    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;
}
