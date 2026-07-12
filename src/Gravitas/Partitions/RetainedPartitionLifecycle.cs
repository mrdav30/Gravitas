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

    bool IsEmpty { get; }

    bool IsAllocated { get; }

    int EmptySinceFrame { get; }

    bool IsOwnedBy(TOwner owner);

    void SetRetainedIndex(int index);

    void ClearRetainedIndex();
}

internal static class RetainedPartitionLifecycle
{
    internal static void DetachAll<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
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
                Untrack(retainedPartitions, owner, partition, ref disabledCursor);
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
        TOwner owner,
        TPartition partition,
        string partitionName)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            partition.RetainedIndex >= 0,
            nameof(partition),
            $"{partitionName} is already tracked as retained.");

        partition.SetRetainedIndex(retainedPartitions.Count);
        retainedPartitions.Add(partition);
    }

    internal static void Untrack<TPartition, TOwner>(
        SwiftList<TPartition> retainedPartitions,
        TOwner owner,
        TPartition partition,
        ref int retirementCursor)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
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
        int currentFrame,
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
        SwiftList<TPartition> retainedPartitions,
        SwiftStack<TPartition> inactivePartitionPool,
        GridWorld world,
        TOwner owner,
        Action<TPartition> releasePartition,
        ref int retirementCursor)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        int inspected = 0;
        while (inspected < retainedPartitions.Count && retainedPartitions.Count > 0)
        {
            if (retirementCursor >= retainedPartitions.Count)
                retirementCursor = 0;

            TPartition partition = retainedPartitions[retirementCursor];
            inspected++;

            if (!partition.IsOwnedBy(owner) || !partition.IsEmpty || partition.IsAllocated)
            {
                retirementCursor++;
                continue;
            }

            int poolCount = inactivePartitionPool.Count;
            Retire(world, owner, partition, releasePartition);
            if (inactivePartitionPool.Count > poolCount)
                return true;

            retirementCursor++;
        }

        return false;
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
        int currentFrame,
        int timeToKillFrames)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
    {
        if (!partition.IsOwnedBy(owner) || !partition.IsEmpty || partition.IsAllocated || partition.EmptySinceFrame < 0)
            return false;

        int idleFrames = currentFrame - partition.EmptySinceFrame;
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
