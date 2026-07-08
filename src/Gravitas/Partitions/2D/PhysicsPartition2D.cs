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

public sealed class PhysicsPartition2D : IVoxelPartition
{
    private GravitasCollision2DService? _owner;
    private int _emptySinceFrame = -1;
    private int _retainedIndex = -1;

    public PhysicsPartition2D()
    {
        ActivationId = -1;
    }

    public WorldVoxelIndex WorldIndex { get; set; }

    public bool IsPartitioned { get; set; }

    public SwiftSparseSet? ContainedDynamicObjects;

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

    public int AwakeDynamicObjectCount => ContainedAwakeDynamicObjects?.Count ?? 0;

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

    public void OnAddToVoxel(Voxel voxel)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _owner == null,
            nameof(PhysicsPartition2D),
            "PhysicsPartition2D is missing its owner collision service.");
        WorldIndex = voxel.WorldIndex;
        IsPartitioned = true;
    }

    public void OnChange() { }

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

    public void RemoveStaticObject(int item)
    {
        if (ContainedStaticObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"2D static item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    public void RemoveKinematicObject(int item)
    {
        if (ContainedKinematicObjects?.Remove(item) != true)
        {
            GravitasLogger.DebugChannel.Info($"2D kinematic item not removed - {item}");
            return;
        }

        MarkEmptyIfUnoccupied();
    }

    public bool ContainsAwakeDynamicObject(int item) => ContainedAwakeDynamicObjects?.Contains(item) == true;

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

    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;
}
