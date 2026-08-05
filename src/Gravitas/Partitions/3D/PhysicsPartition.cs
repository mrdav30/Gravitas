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

/// <summary>
/// Stores the 3D collider IDs assigned to one GridForge voxel.
/// </summary>
public class PhysicsPartition : IVoxelPartition, IRetainedPhysicsPartition<GravitasCollisionService>
{
    private GravitasCollisionService? _owner;
    private int _emptySinceFrame = -1;
    private int _retainedIndex = -1;

    /// <summary>Gets or sets the world voxel occupied by this partition.</summary>
    public WorldVoxelIndex WorldIndex { get; set; }

    /// <summary>Gets or sets whether this partition is attached to a voxel.</summary>
    public bool IsPartitioned { get; set; }

    /// <summary>
    /// Stores context-local dynamic body IDs.
    /// </summary>
    public SwiftSparseSet? ContainedDynamicObjects;

    /// <summary>
    /// Stores the subset of dynamic collider IDs whose bodies can currently drive collision work.
    /// </summary>
    public SwiftSparseSet? ContainedAwakeDynamicObjects;

    /// <summary>Stores context-local kinematic collider IDs.</summary>
    public SwiftSparseSet? ContainedKinematicObjects;

    /// <summary>Stores context-local static collider IDs.</summary>
    public SwiftSparseSet? ContainedStaticObjects;

    /// <summary>Gets the active-partition slot, or <c>-1</c> when inactive.</summary>
    public int ActivationId { get; private set; }

    /// <summary>Gets whether this partition has an active-partition slot.</summary>
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

    /// <summary>Gets the collision service that owns this partition.</summary>
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

    /// <summary>Creates an inactive 3D physics partition.</summary>
    public PhysicsPartition()
    {
        ActivationId = -1;
    }

    /// <summary>Attaches this partition to a voxel.</summary>
    public void OnAddToVoxel(Voxel voxel)
    {
        SwiftThrowHelper.ThrowIfTrue(_owner == null, nameof(PhysicsPartition), "PhysicsPartition is missing its owner collision service.");
        WorldIndex = voxel.WorldIndex;
        IsPartitioned = true;
    }

    internal void Distribute(
        SwiftList<int> dynamicIds,
        SwiftList<int> staticIds)
    {
        int dynamicCount = ContainedDynamicObjects?.Count ?? 0;
        int awakeDynamicCount = ContainedAwakeDynamicObjects?.Count ?? 0;
        if (ContainedDynamicObjects == null || dynamicCount == 0 || ContainedAwakeDynamicObjects == null || awakeDynamicCount == 0)
            return;

        ContainedDynamicObjects.CopySortedKeysTo(dynamicIds);
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

    private void CopySortedStaticStyleIds(SwiftList<int> destination)
    {
        destination.FastClear();
        CopyIds(ContainedKinematicObjects, destination);
        CopyIds(ContainedStaticObjects, destination);
        destination.SortInPlace();
    }

    internal void CopyAllColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        CopyIds(ContainedDynamicObjects, destination);
        CopyIds(ContainedKinematicObjects, destination);
        CopyIds(ContainedStaticObjects, destination);
        destination.SortInPlace();
    }

    internal void CopyStaticStyleColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        CopyIds(ContainedKinematicObjects, destination);
        CopyIds(ContainedStaticObjects, destination);
        destination.SortInPlace();
    }

    private static void CopyIds(SwiftSparseSet? source, SwiftList<int> destination)
    {
        if (source == null)
            return;

        for (int i = 0; i < source.Count; i++)
            destination.Add(source.DenseKeys[i]);
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

    /// <summary>Adds a dynamic collider ID to this partition.</summary>
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

    /// <summary>Adds a static collider ID to this partition.</summary>
    public void AddStaticObject(int item)
    {
        ContainedStaticObjects ??= new();
        if (ContainedStaticObjects.Add(item))
            MarkOccupied();
    }

    /// <summary>Adds a kinematic collider ID to this partition.</summary>
    public void AddKinematicObject(int item)
    {
        ContainedKinematicObjects ??= new();
        if (ContainedKinematicObjects.Add(item))
            MarkOccupied();
    }

    /// <summary>Removes a dynamic collider ID from this partition.</summary>
    public void RemoveDynamicObject(int item)
    {
        if (ContainedDynamicObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"Dynamic item not removed - {item}");
            return;
        }

        ContainedAwakeDynamicObjects?.Remove(item);

        if (ContainedDynamicObjects.Count > 0)
            return;

        // If there are no more dynamic objects, we can deactivate the partition to save on future checks until it's needed again.
        Owner.DeactivatePartition(ActivationId);
        ActivationId = -1;
        MarkEmptyIfUnoccupied();
    }

    /// <summary>Removes a static collider ID from this partition.</summary>
    public void RemoveStaticObject(int item)
    {
        if (ContainedStaticObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"Static item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    /// <summary>Removes a kinematic collider ID from this partition.</summary>
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

        SolidBody? body = collider!.Body;
        return body == null || body.IsAwakeForCollision;
    }

    /// <summary>Releases this partition when it is removed from a voxel.</summary>
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
            Owner.DeactivatePartition(ActivationId);

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
        if (IsEmpty)
            MarkEmpty(Owner.Context.FrameCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void MarkEmpty(int frame) => _emptySinceFrame = frame;

    /// <summary>
    /// Sets the parent index for the current voxel in the world.
    /// </summary>
    /// <param name="parentIndex">The index to assign as the parent of the current voxel.</param>
    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;

    int IRetainedPhysicsPartition<GravitasCollisionService>.RetainedIndex => RetainedIndex;

    bool IRetainedPhysicsPartition<GravitasCollisionService>.IsEmpty => IsEmpty;

    int IRetainedPhysicsPartition<GravitasCollisionService>.EmptySinceFrame => EmptySinceFrame;

    bool IRetainedPhysicsPartition<GravitasCollisionService>.IsOwnedBy(GravitasCollisionService owner) => IsOwnedBy(owner);

    void IRetainedPhysicsPartition<GravitasCollisionService>.SetRetainedIndex(int index) => SetRetainedIndex(index);

    void IRetainedPhysicsPartition<GravitasCollisionService>.ClearRetainedIndex() => ClearRetainedIndex();
}
