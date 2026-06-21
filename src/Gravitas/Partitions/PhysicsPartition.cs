//=======================================================================
// PhysicsPartition.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal enum PhysicsPartitionMobilityKind
{
    Dynamic = 0,
    Kinematic = 1,
    Static = 2
}

public class PhysicsPartition : IVoxelPartition
{
    private GravitasCollisionService? _owner;
    private int _emptySinceFrame = -1;
    private int _retainedIndex = -1;

    public WorldVoxelIndex WorldIndex { get; set; }

    public bool IsPartitioned { get; set; }

    /// <summary>
    /// Stores context-local dynamic body IDs.
    /// </summary>
    public SwiftSparseSet? ContainedDynamicObjects;

    /// <summary>
    /// Stores the subset of dynamic collider IDs whose bodies can currently drive collision work.
    /// </summary>
    public SwiftSparseSet? ContainedAwakeDynamicObjects;

    public SwiftSparseSet? ContainedKinematicObjects;

    public SwiftSparseSet? ContainedStaticObjects;

    public int ActivationId { get; private set; }

    public bool IsAllocated => ActivationId != -1;

    internal bool IsEmpty =>
        (ContainedDynamicObjects?.Count ?? 0) == 0
        && (ContainedKinematicObjects?.Count ?? 0) == 0
        && (ContainedStaticObjects?.Count ?? 0) == 0;

    internal int EmptySinceFrame => _emptySinceFrame;

    internal int RetainedIndex => _retainedIndex;

    /// <summary>
    /// Gets the number of awake dynamic IDs currently in this partition.
    /// </summary>
    public int AwakeDynamicObjectCount => ContainedAwakeDynamicObjects?.Count ?? 0;

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

    internal void Distribute(
        SwiftList<int> dynamicIds,
        SwiftList<int> awakeDynamicIds,
        SwiftList<int> staticIds)
    {
        int dynamicCount = ContainedDynamicObjects?.Count ?? 0;
        int awakeDynamicCount = ContainedAwakeDynamicObjects?.Count ?? 0;
        if (ContainedDynamicObjects == null || dynamicCount == 0 || ContainedAwakeDynamicObjects == null || awakeDynamicCount == 0)
            return;

        CopySortedIds(ContainedDynamicObjects, dynamicIds);
        CopySortedIds(ContainedAwakeDynamicObjects, awakeDynamicIds);
        CopySortedStaticStyleIds(staticIds);

        // Sleeping bodies stay query-visible in dynamic membership, while awake membership gates partition work.
        // Once an active partition is selected, all local dynamic links are emitted so the discrete
        // island builder can wake and solve connected bodies in a stable graph instead of a flat pair pass.
        for (int j = 0; j < dynamicIds.Count; j++)
        {
            int id1 = dynamicIds[j];
            for (int k = j + 1; k < dynamicIds.Count; k++)
                ProcessPair(id1, dynamicIds[k]);

            for (int k = 0; k < staticIds.Count; k++)
            {
                int id2 = staticIds[k];
                ProcessPair(id1, id2);
            }
        }
    }

    private static void CopySortedIds(SwiftSparseSet? source, SwiftList<int> destination)
    {
        destination.FastClear();
        if (source == null)
            return;

        for (int i = 0; i < source.Count; i++)
            destination.Add(source.DenseKeys[i]);

        SortColliderIdsIfNeeded(destination);
    }

    private void CopySortedStaticStyleIds(SwiftList<int> destination)
    {
        destination.FastClear();
        CopyIds(ContainedKinematicObjects, destination);
        CopyIds(ContainedStaticObjects, destination);
        SortColliderIdsIfNeeded(destination);
    }

    internal void CopyAllColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        CopyIds(ContainedDynamicObjects, destination);
        CopyIds(ContainedKinematicObjects, destination);
        CopyIds(ContainedStaticObjects, destination);
        SortColliderIdsIfNeeded(destination);
    }

    internal void CopyStaticStyleColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        CopyIds(ContainedKinematicObjects, destination);
        CopyIds(ContainedStaticObjects, destination);
        SortColliderIdsIfNeeded(destination);
    }

    private static void CopyIds(SwiftSparseSet? source, SwiftList<int> destination)
    {
        if (source == null)
            return;

        for (int i = 0; i < source.Count; i++)
            destination.Add(source.DenseKeys[i]);
    }

    private static void SortColliderIdsIfNeeded(SwiftList<int> ids)
    {
        for (int i = 1; i < ids.Count; i++)
        {
            int value = ids[i];
            int j = i - 1;
            while (j >= 0 && ids[j] > value)
            {
                ids[j + 1] = ids[j];
                j--;
            }

            ids[j + 1] = value;
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
        pair.UpdateCollisionDeferred();
    }

    public void AddDynamicObject(int item)
    {
        ContainedDynamicObjects ??= new();
        if (!ContainedDynamicObjects.Add(item))
            return;

        MarkOccupied();
        SetDynamicObjectAwake(item, IsDynamicObjectAwake(item));

        if (ContainedDynamicObjects.Count == 1)
            ActivationId = Owner.ActivatePartition(this);
    }

    public void AddStaticObject(int item)
    {
        ContainedStaticObjects ??= new();
        if (ContainedStaticObjects.Add(item))
            MarkOccupied();
    }

    public void AddKinematicObject(int item)
    {
        ContainedKinematicObjects ??= new();
        if (ContainedKinematicObjects.Add(item))
            MarkOccupied();
    }

    public void RemoveDynamicObject(int item)
    {
        if (ContainedDynamicObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"Dynamic item not removed - {item}");
            return;
        }

        ContainedAwakeDynamicObjects?.Remove(item);

        if (ContainedDynamicObjects?.Count > 0)
            return;

        // If there are no more dynamic objects, we can deactivate the partition to save on future checks until it's needed again.
        Owner.DeactivatePartition(ActivationId);
        ActivationId = -1;
        MarkEmptyIfUnoccupied();
    }

    public void RemoveStaticObject(int item)
    {
        if (ContainedStaticObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"Static item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    public void RemoveKinematicObject(int item)
    {
        if (ContainedKinematicObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"Kinematic item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    /// <summary>
    /// Returns true when the supplied dynamic collider ID is marked awake in this partition.
    /// </summary>
    public bool ContainsAwakeDynamicObject(int item) => ContainedAwakeDynamicObjects?.Contains(item) == true;

    /// <summary>
    /// Updates the awake dynamic subset for a collider already present in dynamic membership.
    /// </summary>
    public void SetDynamicObjectAwake(int item, bool awake)
    {
        if (ContainedDynamicObjects?.Contains(item) != true)
            return;

        if (awake)
        {
            ContainedAwakeDynamicObjects ??= new();
            ContainedAwakeDynamicObjects.Add(item);
            return;
        }

        ContainedAwakeDynamicObjects?.Remove(item);
    }

    private bool IsDynamicObjectAwake(int item)
    {
        if (!Owner.Context.Physics.TryGetColliderById(item, out Gravitas.Colliders.LSCollider? collider))
            return true;

        StiffBody? body = collider!.Body;
        return body == null || body.IsAwakeForCollision;
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

    internal bool IsOwnedBy(GravitasCollisionService owner) => ReferenceEquals(_owner, owner);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void SetRetainedIndex(int index)
    {
        SwiftThrowHelper.ThrowIfNegative(index, nameof(index));
        _retainedIndex = index;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearRetainedIndex() => _retainedIndex = -1;

    internal void ResetRetainedMembership()
    {
        ContainedDynamicObjects?.Clear();
        ContainedAwakeDynamicObjects?.Clear();
        ContainedKinematicObjects?.Clear();
        ContainedStaticObjects?.Clear();
        ActivationId = -1;
        MarkEmpty(_owner?.Context.FrameCount ?? 0);
    }

    internal void ResetForPool()
    {
        ContainedDynamicObjects?.Clear();
        ContainedAwakeDynamicObjects?.Clear();
        ContainedKinematicObjects?.Clear();
        ContainedStaticObjects?.Clear();

        if (ActivationId != -1)
            _owner?.DeactivatePartition(ActivationId);

        _owner = null;
        ActivationId = -1;
        IsPartitioned = false;
        _emptySinceFrame = -1;
        _retainedIndex = -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void MarkOccupied() => _emptySinceFrame = -1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void MarkEmptyIfUnoccupied()
    {
        if (IsEmpty && _emptySinceFrame < 0)
            MarkEmpty(Owner.Context.FrameCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void MarkEmpty(int frame) => _emptySinceFrame = frame;

    /// <summary>
    /// Sets the parent index for the current voxel in the world.
    /// </summary>
    /// <param name="parentIndex">The index to assign as the parent of the current voxel.</param>
    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;
}
