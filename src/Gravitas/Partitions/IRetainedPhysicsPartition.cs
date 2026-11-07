//=======================================================================
// IRetainedPhysicsPartition.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using GridForge.Spatial;

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
