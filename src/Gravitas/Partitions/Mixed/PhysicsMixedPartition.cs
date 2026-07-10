//=======================================================================
// PhysicsMixedPartition.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal enum MixedPartitionMobilityKind
{
    Dynamic = 0,
    Kinematic = 1,
    Static = 2
}

/// <summary>
/// GridForge voxel partition that stores cross-dimensional broad-phase memberships.
/// </summary>
internal sealed class PhysicsMixedPartition : IVoxelPartition, IRetainedPhysicsPartition<GravitasMixedCollisionService>
{
    private GravitasMixedCollisionService? _owner;
    private int _emptySinceFrame = -1;
    private int _retainedIndex = -1;

    public PhysicsMixedPartition()
    {
        ActivationId = -1;
    }

    public WorldVoxelIndex WorldIndex { get; set; }

    public bool IsPartitioned { get; set; }

    public SwiftSparseSet? ContainedDynamic3DObjects;

    public SwiftSparseSet? ContainedAwakeDynamic3DObjects;

    public SwiftSparseSet? ContainedKinematic3DObjects;

    public SwiftSparseSet? ContainedStatic3DObjects;

    public SwiftSparseSet? ContainedDynamic2DObjects;

    public SwiftSparseSet? ContainedAwakeDynamic2DObjects;

    public SwiftSparseSet? ContainedKinematic2DObjects;

    public SwiftSparseSet? ContainedStatic2DObjects;

    public int ActivationId { get; private set; }

    public bool IsAllocated => ActivationId != -1;

    internal bool IsEmpty =>
        (ContainedDynamic3DObjects?.Count ?? 0) == 0
        && (ContainedKinematic3DObjects?.Count ?? 0) == 0
        && (ContainedStatic3DObjects?.Count ?? 0) == 0
        && (ContainedDynamic2DObjects?.Count ?? 0) == 0
        && (ContainedKinematic2DObjects?.Count ?? 0) == 0
        && (ContainedStatic2DObjects?.Count ?? 0) == 0;

    internal int EmptySinceFrame => _emptySinceFrame;

    internal int RetainedIndex => _retainedIndex;

    internal int AwakeDynamicObjectCount =>
        (ContainedAwakeDynamic3DObjects?.Count ?? 0) + (ContainedAwakeDynamic2DObjects?.Count ?? 0);

    private int MovableDynamicObjectCount =>
        (ContainedDynamic3DObjects?.Count ?? 0) + (ContainedDynamic2DObjects?.Count ?? 0);

    public GravitasMixedCollisionService Owner
    {
        get
        {
            SwiftThrowHelper.ThrowIfTrue(
                _owner == null,
                nameof(PhysicsMixedPartition),
                "PhysicsMixedPartition is missing its owner collision service.");
            return _owner!;
        }
    }

    public void OnAddToVoxel(Voxel voxel)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _owner == null,
            nameof(PhysicsMixedPartition),
            "PhysicsMixedPartition is missing its owner collision service.");
        WorldIndex = voxel.WorldIndex;
        IsPartitioned = true;
    }

    public void OnChange() { }

    internal void Distribute(
        SwiftList<int> dynamic3DIds,
        SwiftList<int> kinematic3DIds,
        SwiftList<int> static3DIds,
        SwiftList<int> dynamic2DIds,
        SwiftList<int> kinematic2DIds,
        SwiftList<int> static2DIds)
    {
        int total3DCount = (ContainedDynamic3DObjects?.Count ?? 0)
            + (ContainedKinematic3DObjects?.Count ?? 0)
            + (ContainedStatic3DObjects?.Count ?? 0);
        int total2DCount = (ContainedDynamic2DObjects?.Count ?? 0)
            + (ContainedKinematic2DObjects?.Count ?? 0)
            + (ContainedStatic2DObjects?.Count ?? 0);
        if (total3DCount == 0 || total2DCount == 0 || AwakeDynamicObjectCount == 0)
            return;

        CopySortedIds(ContainedDynamic3DObjects, dynamic3DIds);
        CopySortedIds(ContainedKinematic3DObjects, kinematic3DIds);
        CopySortedIds(ContainedStatic3DObjects, static3DIds);
        CopySortedIds(ContainedDynamic2DObjects, dynamic2DIds);
        CopySortedIds(ContainedKinematic2DObjects, kinematic2DIds);
        CopySortedIds(ContainedStatic2DObjects, static2DIds);

        GravitasMixedCollisionService owner = Owner;
        for (int i = 0; i < dynamic3DIds.Count; i++)
        {
            int id3D = dynamic3DIds[i];
            for (int j = 0; j < dynamic2DIds.Count; j++)
                owner.ProcessPartitionCandidate(id3D, dynamic2DIds[j]);

            for (int j = 0; j < kinematic2DIds.Count; j++)
                owner.ProcessPartitionCandidate(id3D, kinematic2DIds[j]);

            for (int j = 0; j < static2DIds.Count; j++)
                owner.ProcessPartitionCandidate(id3D, static2DIds[j]);
        }

        for (int i = 0; i < dynamic2DIds.Count; i++)
        {
            int id2D = dynamic2DIds[i];
            for (int j = 0; j < kinematic3DIds.Count; j++)
                owner.ProcessPartitionCandidate(kinematic3DIds[j], id2D);

            for (int j = 0; j < static3DIds.Count; j++)
                owner.ProcessPartitionCandidate(static3DIds[j], id2D);
        }
    }

    public void AddDynamic3DObject(int id)
    {
        ContainedDynamic3DObjects ??= new();
        bool shouldActivate = MovableDynamicObjectCount == 0;
        if (!ContainedDynamic3DObjects.Add(id))
            return;

        MarkOccupied();
        SetDynamic3DObjectAwake(id, IsDynamic3DObjectAwake(id));
        if (shouldActivate)
            ActivationId = Owner.ActivatePartition(this);
    }

    public void AddStatic3DObject(int id)
    {
        ContainedStatic3DObjects ??= new();
        if (ContainedStatic3DObjects.Add(id))
            MarkOccupied();
    }

    public void AddKinematic3DObject(int id)
    {
        ContainedKinematic3DObjects ??= new();
        if (ContainedKinematic3DObjects.Add(id))
            MarkOccupied();
    }

    public void AddDynamic2DObject(int id)
    {
        ContainedDynamic2DObjects ??= new();
        bool shouldActivate = MovableDynamicObjectCount == 0;
        if (!ContainedDynamic2DObjects.Add(id))
            return;

        MarkOccupied();
        SetDynamic2DObjectAwake(id, IsDynamic2DObjectAwake(id));
        if (shouldActivate)
            ActivationId = Owner.ActivatePartition(this);
    }

    public void AddStatic2DObject(int id)
    {
        ContainedStatic2DObjects ??= new();
        if (ContainedStatic2DObjects.Add(id))
            MarkOccupied();
    }

    public void AddKinematic2DObject(int id)
    {
        ContainedKinematic2DObjects ??= new();
        if (ContainedKinematic2DObjects.Add(id))
            MarkOccupied();
    }

    public void RemoveDynamic3DObject(int id)
    {
        if (ContainedDynamic3DObjects?.Remove(id) != true)
            return;

        ContainedAwakeDynamic3DObjects?.Remove(id);
        DeactivateIfNoDynamicMembers();
        MarkEmptyIfUnoccupied();
    }

    public void RemoveStatic3DObject(int id)
    {
        if (ContainedStatic3DObjects?.Remove(id) == true)
            MarkEmptyIfUnoccupied();
    }

    public void RemoveKinematic3DObject(int id)
    {
        if (ContainedKinematic3DObjects?.Remove(id) == true)
            MarkEmptyIfUnoccupied();
    }

    public void RemoveDynamic2DObject(int id)
    {
        if (ContainedDynamic2DObjects?.Remove(id) != true)
            return;

        ContainedAwakeDynamic2DObjects?.Remove(id);
        DeactivateIfNoDynamicMembers();
        MarkEmptyIfUnoccupied();
    }

    public void RemoveStatic2DObject(int id)
    {
        if (ContainedStatic2DObjects?.Remove(id) == true)
            MarkEmptyIfUnoccupied();
    }

    public void RemoveKinematic2DObject(int id)
    {
        if (ContainedKinematic2DObjects?.Remove(id) == true)
            MarkEmptyIfUnoccupied();
    }

    public void SetDynamic3DObjectAwake(int id, bool awake)
    {
        if (ContainedDynamic3DObjects?.Contains(id) != true)
            return;

        if (awake)
        {
            ContainedAwakeDynamic3DObjects ??= new();
            ContainedAwakeDynamic3DObjects.Add(id);
            return;
        }

        ContainedAwakeDynamic3DObjects?.Remove(id);
    }

    public void SetDynamic2DObjectAwake(int id, bool awake)
    {
        if (ContainedDynamic2DObjects?.Contains(id) != true)
            return;

        if (awake)
        {
            ContainedAwakeDynamic2DObjects ??= new();
            ContainedAwakeDynamic2DObjects.Add(id);
            return;
        }

        ContainedAwakeDynamic2DObjects?.Remove(id);
    }

    public void OnRemoveFromVoxel(Voxel voxel)
    {
        Owner.ReleasePartition(this);
    }

    internal void SetOwner(GravitasMixedCollisionService owner)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            _owner != null && !ReferenceEquals(_owner, owner),
            nameof(owner),
            "PhysicsMixedPartition is already owned by a different collision service.");

        _owner = owner;
    }

    internal bool IsOwnedBy(GravitasMixedCollisionService owner) => ReferenceEquals(_owner, owner);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void SetRetainedIndex(int index)
    {
        SwiftThrowHelper.ThrowIfNegative(index, nameof(index));
        _retainedIndex = index;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearRetainedIndex() => _retainedIndex = -1;

    int IRetainedPhysicsPartition<GravitasMixedCollisionService>.RetainedIndex => RetainedIndex;

    bool IRetainedPhysicsPartition<GravitasMixedCollisionService>.IsEmpty => IsEmpty;

    int IRetainedPhysicsPartition<GravitasMixedCollisionService>.EmptySinceFrame => EmptySinceFrame;

    bool IRetainedPhysicsPartition<GravitasMixedCollisionService>.IsOwnedBy(GravitasMixedCollisionService owner) => IsOwnedBy(owner);

    void IRetainedPhysicsPartition<GravitasMixedCollisionService>.SetRetainedIndex(int index) => SetRetainedIndex(index);

    void IRetainedPhysicsPartition<GravitasMixedCollisionService>.ClearRetainedIndex() => ClearRetainedIndex();

    internal void ResetRetainedMembership()
    {
        ContainedDynamic3DObjects?.Clear();
        ContainedAwakeDynamic3DObjects?.Clear();
        ContainedKinematic3DObjects?.Clear();
        ContainedStatic3DObjects?.Clear();
        ContainedDynamic2DObjects?.Clear();
        ContainedAwakeDynamic2DObjects?.Clear();
        ContainedKinematic2DObjects?.Clear();
        ContainedStatic2DObjects?.Clear();
        ActivationId = -1;
        MarkEmpty(_owner?.Context.FrameCount ?? 0);
    }

    internal void ResetForPool()
    {
        ContainedDynamic3DObjects?.Clear();
        ContainedAwakeDynamic3DObjects?.Clear();
        ContainedKinematic3DObjects?.Clear();
        ContainedStatic3DObjects?.Clear();
        ContainedDynamic2DObjects?.Clear();
        ContainedAwakeDynamic2DObjects?.Clear();
        ContainedKinematic2DObjects?.Clear();
        ContainedStatic2DObjects?.Clear();

        if (ActivationId != -1)
            Owner.DeactivatePartition(ActivationId);

        _owner = null;
        ActivationId = -1;
        IsPartitioned = false;
        _emptySinceFrame = -1;
        _retainedIndex = -1;
    }

    public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;

    internal void Copy3DColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        AppendIds(ContainedDynamic3DObjects, destination);
        AppendIds(ContainedKinematic3DObjects, destination);
        AppendIds(ContainedStatic3DObjects, destination);
        destination.SortInPlace();
    }

    internal void Copy2DColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        AppendIds(ContainedDynamic2DObjects, destination);
        AppendIds(ContainedKinematic2DObjects, destination);
        AppendIds(ContainedStatic2DObjects, destination);
        destination.SortInPlace();
    }

    internal void CopyStaticStyle3DColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        AppendIds(ContainedKinematic3DObjects, destination);
        AppendIds(ContainedStatic3DObjects, destination);
        destination.SortInPlace();
    }

    internal void CopyStaticStyle2DColliderIds(SwiftList<int> destination)
    {
        destination.FastClear();
        AppendIds(ContainedKinematic2DObjects, destination);
        AppendIds(ContainedStatic2DObjects, destination);
        destination.SortInPlace();
    }

    private static void AppendIds(SwiftSparseSet? source, SwiftList<int> destination)
    {
        if (source == null)
            return;

        for (int i = 0; i < source.Count; i++)
            destination.Add(source.DenseKeys[i]);
    }

    private static void CopySortedIds(SwiftSparseSet? source, SwiftList<int> destination)
    {
        if (source == null)
        {
            destination.FastClear();
            return;
        }

        source.CopySortedKeysTo(destination);
    }

    private bool IsDynamic3DObjectAwake(int id)
    {
        if (!Owner.Context.Physics.TryGetColliderById(id, out Gravitas.Colliders.LSCollider? collider))
            return true;

        SolidBody? body = collider!.Body;
        return body == null || body.IsAwakeForCollision;
    }

    private bool IsDynamic2DObjectAwake(int id)
    {
        if (!Owner.Context.Physics2D.TryGetColliderById(id, out Gravitas.Colliders.LSCollider2D? collider))
            return true;

        SolidBody2D? body = collider!.Body;
        return body == null || body.IsAwakeForCollision;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DeactivateIfNoDynamicMembers()
    {
        if (MovableDynamicObjectCount > 0)
            return;

        Owner.DeactivatePartition(ActivationId);
        ActivationId = -1;
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
}
