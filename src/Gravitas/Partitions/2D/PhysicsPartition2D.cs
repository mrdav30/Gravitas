//=======================================================================
// PhysicsPartition2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Stores the pure 2D collider IDs assigned to one GridForge voxel.
/// </summary>
public sealed class PhysicsPartition2D : IVoxelPartition, IRetainedPhysicsPartition<GravitasCollision2DService>
{
    private GravitasCollision2DService? _owner;
    private int _emptySinceFrame = -1;
    private int _retainedIndex = -1;

    /// <summary>Creates an inactive 2D physics partition.</summary>
    public PhysicsPartition2D()
    {
        ActivationId = -1;
    }

    /// <summary>Gets or sets the world voxel occupied by this partition.</summary>
    public WorldVoxelIndex WorldIndex { get; set; }

    /// <summary>Gets or sets whether this partition is attached to a voxel.</summary>
    public bool IsPartitioned { get; set; }

    /// <summary>Stores context-local dynamic collider IDs.</summary>
    public SwiftSparseSet? ContainedDynamicObjects;

    /// <summary>Stores the awake subset of the dynamic collider IDs.</summary>
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

    /// <summary>Gets the number of awake dynamic IDs in this partition.</summary>
    public int AwakeDynamicObjectCount => ContainedAwakeDynamicObjects?.Count ?? 0;

    /// <summary>Gets the collision service that owns this partition.</summary>
    public GravitasCollision2DService Owner
    {
        get
        {
            SwiftThrowHelper.ThrowIfTrue(
                _owner == null,
                nameof(PhysicsPartition2D),
                "PhysicsPartition2D is missing its owner collision service.");
            return _owner!;
        }
    }

    /// <summary>Attaches this partition to a voxel.</summary>
    public void OnAddToVoxel(Voxel voxel)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _owner == null,
            nameof(PhysicsPartition2D),
            "PhysicsPartition2D is missing its owner collision service.");
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

        for (int j = 0; j < dynamicIds.Count; j++)
        {
            int id1 = dynamicIds[j];
            for (int k = j + 1; k < dynamicIds.Count; k++)
                Owner.Context.Physics2D.ProcessPartitionCandidate(id1, dynamicIds[k], WorldIndex);

            for (int k = 0; k < staticIds.Count; k++)
                Owner.Context.Physics2D.ProcessPartitionCandidate(id1, staticIds[k], WorldIndex);
        }
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

    private void CopySortedStaticStyleIds(SwiftList<int> destination)
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
            GravitasLogger.DebugChannel.Info($"2D dynamic item not removed - {item}");
            return;
        }

        ContainedAwakeDynamicObjects?.Remove(item);

        if (ContainedDynamicObjects.Count > 0)
            return;

        Owner.DeactivatePartition(ActivationId);
        ActivationId = -1;
        MarkEmptyIfUnoccupied();
    }

    /// <summary>Removes a static collider ID from this partition.</summary>
    public void RemoveStaticObject(int item)
    {
        if (ContainedStaticObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"2D static item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    /// <summary>Removes a kinematic collider ID from this partition.</summary>
    public void RemoveKinematicObject(int item)
    {
        if (ContainedKinematicObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"2D kinematic item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    /// <summary>Returns whether a dynamic collider ID is marked awake.</summary>
    public bool ContainsAwakeDynamicObject(int item) => ContainedAwakeDynamicObjects?.Contains(item) == true;

    /// <summary>Updates the awake state for a dynamic collider in this partition.</summary>
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
        if (!Owner.Context.Physics2D.TryGetColliderById(item, out LSCollider2D? collider))
            return true;

        SolidBody2D? body = collider!.Body;
        return body == null || body.IsAwakeForCollision;
    }

    /// <summary>Releases this partition when it is removed from a voxel.</summary>
    public void OnRemoveFromVoxel(Voxel voxel)
    {
        Owner.ReleasePartition(this);
    }

    internal void SetOwner(GravitasCollision2DService owner)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            _owner != null && !ReferenceEquals(_owner, owner),
            nameof(owner),
            "PhysicsPartition2D is already owned by a different collision service.");

        _owner = owner;
    }

    internal bool IsOwnedBy(GravitasCollision2DService owner) => ReferenceEquals(_owner, owner);

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

    /// <summary>Updates the world voxel index used by this partition.</summary>
    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;

    int IRetainedPhysicsPartition<GravitasCollision2DService>.RetainedIndex => RetainedIndex;

    bool IRetainedPhysicsPartition<GravitasCollision2DService>.IsEmpty => IsEmpty;

    int IRetainedPhysicsPartition<GravitasCollision2DService>.EmptySinceFrame => EmptySinceFrame;

    bool IRetainedPhysicsPartition<GravitasCollision2DService>.IsOwnedBy(GravitasCollision2DService owner) => IsOwnedBy(owner);

    void IRetainedPhysicsPartition<GravitasCollision2DService>.SetRetainedIndex(int index) => SetRetainedIndex(index);

    void IRetainedPhysicsPartition<GravitasCollision2DService>.ClearRetainedIndex() => ClearRetainedIndex();
}
