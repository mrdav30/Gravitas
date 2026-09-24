//=======================================================================
// GravitasWorldContext.QueryPartitions.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using GridForge.Grids;

namespace Gravitas;

/// <content>Reconciles physical grid membership at stable sampling boundaries.</content>
public sealed partial class GravitasWorldContext
{
    private ulong _observedGridChangeSequence;
    private bool _queryPartitionTopologyChanged;

    private void OnQueryPartitionWorldChange(GridEventInfo change)
    {
        _observedGridChangeSequence = change.ChangeSequence;
        if (change.ChangeKind is GridEventKind.GridAdded or GridEventKind.GridRemoved
            or GridEventKind.GridChanged or GridEventKind.SparseVoxelAdded
            or GridEventKind.SparseVoxelRemoved or GridEventKind.WorldReset)
        {
            _queryPartitionTopologyChanged = Physics.ColliderCount > 0 || Physics2D.ColliderCount > 0;
        }
    }

    internal bool RefreshQueryPartitions()
    {
        if (!IsBetweenFixedSteps || !World.IsActive
            || _observedGridChangeSequence != World.ChangeSequence)
        {
            return false;
        }

        if (_queryPartitionTopologyChanged)
        {
            // Registration scans occur once per physical topology batch, never per
            // ordinary query, obstacle event, or unchanged simulation frame.
            Collisions.RebuildPartitionsAfterWorldChange();
            Collisions2D.RebuildPartitionsAfterWorldChange();
            _queryPartitionTopologyChanged = false;
        }

        Collisions2D.RefreshDeferredColliderPartitions();
        return true;
    }
}
