//=======================================================================
// RetainedPartitionLifecycle.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;

namespace Gravitas;

internal interface IRetainedPhysicsPartition<in TOwner> : IVoxelPartition
{
    int RetainedIndex { get; }

    // Dense eligible-empty slot, independent of the retained expiry-sweep slot.
    int EmptyIndex { get; set; }

    bool IsEmpty { get; }

    bool IsAllocated { get; }

    long EmptySinceFrame { get; }

    bool IsOwnedBy(TOwner owner);

    void SetRetainedIndex(int index);

    void ClearRetainedIndex();
}

internal static class RetainedPartitionLifecycle
{
    internal static void DetachAll<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
        SwiftList<TPartition> emptyPartitions,
        GridWorld world,
        TOwner owner,
        Action<TPartition> releasePartition,
        string partitionName,
        string detachError)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        // Reset is a context boundary; retained GridForge payloads are a runtime cache, not replay state.
        while (retainedPartitions.Count > 0)
        {
            TPartition partition = retainedPartitions[retainedPartitions.Count - 1];
            if (!partition.IsOwnedBy(owner))
            {
                int disabledCursor = -1;
                Untrack(retainedPartitions, emptyPartitions, owner, partition, ref disabledCursor);
                continue;
            }

            if (world.TryGetVoxel(partition.WorldIndex, out Voxel? voxel)
                && voxel!.TryGetPartition(out TPartition? attachedPartition)
                && ReferenceEquals(attachedPartition, partition))
            {
                bool removed = voxel.TryRemovePartition<TPartition>();
                SwiftThrowHelper.ThrowIfTrue(!removed, partitionName, detachError);

                if (partition.IsOwnedBy(owner))
                    releasePartition(partition);

                continue;
            }

            releasePartition(partition);
        }
    }

    internal static void Track<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
        SwiftList<TPartition> emptyPartitions,
        TOwner owner,
        TPartition partition,
        string partitionName)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        // Tracking runs once per newly occupied voxel. Format validation details
        // only on failure, rather than allocating a message for every payload.
        if (partition.RetainedIndex >= 0)
            throw new ArgumentException($"{partitionName} is already tracked as retained.", nameof(partition));

        partition.SetRetainedIndex(retainedPartitions.Count);
        retainedPartitions.Add(partition);
        // Every retained payload can become empty later. Reserve those slots
        // during registration so ordinary membership removal cannot allocate.
        emptyPartitions.EnsureCapacity(retainedPartitions.Count);
        RefreshEmptyEligibility(emptyPartitions, owner, partition);
    }

    internal static void Untrack<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
        SwiftList<TPartition> emptyPartitions,
        TOwner owner,
        TPartition partition,
        ref int retirementCursor)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        UntrackEmpty<TPartition, TOwner>(emptyPartitions, partition);
        int index = FindIndex(retainedPartitions, owner, partition);
        if (index < 0)
        {
            partition.ClearRetainedIndex();
            return;
        }

        int lastIndex = retainedPartitions.Count - 1;
        if (index != lastIndex)
        {
            TPartition movedPartition = retainedPartitions[lastIndex];
            retainedPartitions[index] = movedPartition;
            movedPartition.SetRetainedIndex(index);
        }

        retainedPartitions.RemoveAt(lastIndex);
        partition.ClearRetainedIndex();

        if (retirementCursor < 0)
            return;

        if (retirementCursor > index)
            retirementCursor--;
        if (retirementCursor >= retainedPartitions.Count)
            retirementCursor = 0;
    }

    internal static void RetireExpired<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
        GridWorld world,
        TOwner owner,
        int budget,
        long currentFrame,
        int timeToKillFrames,
        Action<TPartition> releasePartition,
        ref int retirementCursor)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        if (budget <= 0 || retainedPartitions.Count == 0)
            return;

        int inspected = 0;
        while (inspected < budget && retainedPartitions.Count > 0)
        {
            if (retirementCursor >= retainedPartitions.Count)
                retirementCursor = 0;

            TPartition partition = retainedPartitions[retirementCursor];
            inspected++;

            if (!ShouldRetire(partition, owner, currentFrame, timeToKillFrames))
            {
                retirementCursor++;
                continue;
            }

            Retire(world, owner, partition, releasePartition);
        }
    }

    internal static bool TryRetireEmptyForReuse<TPartition, TOwner>(
        SwiftList<TPartition> emptyPartitions,
        SwiftStack<TPartition> inactivePartitionPool,
        GridWorld world,
        TOwner owner,
        Action<TPartition> releasePartition)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        // Membership transitions maintain this dense list. Occupied cold rentals
        // never scan retained payloads; reuse takes the last dense eligible entry.
        while (emptyPartitions.Count > 0)
        {
            int lastIndex = emptyPartitions.Count - 1;
            TPartition partition = emptyPartitions[lastIndex];
            RemoveEmptyAt<TPartition, TOwner>(emptyPartitions, lastIndex);

            // Recheck ownership and eligibility before detaching a selected payload,
            // without rediscovering occupied payloads or trusting stale cache entries.
            if (partition.RetainedIndex < 0 || !partition.IsOwnedBy(owner) || !partition.IsEmpty || partition.IsAllocated)
                continue;

            int poolCount = inactivePartitionPool.Count;
            Retire(world, owner, partition, releasePartition);
            if (inactivePartitionPool.Count > poolCount)
                return true;
        }

        return false;
    }

    internal static void RefreshEmptyEligibility<TPartition, TOwner>(
        SwiftList<TPartition> emptyPartitions,
        TOwner owner,
        TPartition partition)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        bool eligible = partition.RetainedIndex >= 0 && partition.IsOwnedBy(owner)
            && partition.IsEmpty && !partition.IsAllocated;
        if (!eligible)
        {
            UntrackEmpty<TPartition, TOwner>(emptyPartitions, partition);
            return;
        }

        if ((uint)partition.EmptyIndex < (uint)emptyPartitions.Count
            && ReferenceEquals(emptyPartitions[partition.EmptyIndex], partition))
            return;

        partition.EmptyIndex = emptyPartitions.Count;
        emptyPartitions.Add(partition);
    }

    private static void UntrackEmpty<TPartition, TOwner>(
        SwiftList<TPartition> emptyPartitions,
        TPartition partition)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        int index = partition.EmptyIndex;
        if ((uint)index < (uint)emptyPartitions.Count && ReferenceEquals(emptyPartitions[index], partition))
            RemoveEmptyAt<TPartition, TOwner>(emptyPartitions, index);
        else
            partition.EmptyIndex = -1;
    }

    private static void RemoveEmptyAt<TPartition, TOwner>(SwiftList<TPartition> emptyPartitions, int index)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        // Swap-back keeps transitions constant-time; update the moved payload's
        // slot so a later reoccupation removes it without searching the list.
        TPartition partition = emptyPartitions[index];
        int lastIndex = emptyPartitions.Count - 1;
        if (index != lastIndex)
        {
            TPartition movedPartition = emptyPartitions[lastIndex];
            emptyPartitions[index] = movedPartition;
            movedPartition.EmptyIndex = index;
        }

        emptyPartitions.RemoveAt(lastIndex);
        partition.EmptyIndex = -1;
    }

    private static int FindIndex<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
        TOwner owner,
        TPartition partition)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        _ = owner;
        int index = partition.RetainedIndex;
        return (uint)index < (uint)retainedPartitions.Count && ReferenceEquals(retainedPartitions[index], partition)
            ? index
            : -1;
    }

    private static bool ShouldRetire<TPartition, TOwner>(
        TPartition partition,
        TOwner owner,
        long currentFrame,
        int timeToKillFrames)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        if (!partition.IsOwnedBy(owner) || !partition.IsEmpty || partition.IsAllocated || partition.EmptySinceFrame < 0)
            return false;

        long idleFrames = currentFrame - partition.EmptySinceFrame;
        return idleFrames >= timeToKillFrames;
    }

    private static void Retire<TPartition, TOwner>(
        GridWorld world,
        TOwner owner,
        TPartition partition,
        Action<TPartition> releasePartition)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        _ = owner;
        if (!world.TryGetVoxel(partition.WorldIndex, out Voxel? voxel))
        {
            releasePartition(partition);
            return;
        }

        if (!voxel!.TryGetPartition(out TPartition? attachedPartition)
            || !ReferenceEquals(attachedPartition, partition))
        {
            releasePartition(partition);
            return;
        }

        _ = voxel.TryRemovePartition<TPartition>();
    }
}
