//=======================================================================
// PhysicsQueryScratch.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Threading;
using FixedMathSharp;
using GridForge.Grids;

namespace Gravitas.Queries;

/// <summary>Provides fixed, exclusively borrowed storage for a bounded physics query.</summary>
public sealed class PhysicsQueryScratch
{
    private readonly Voxel?[] _voxels;
    private readonly int[] _candidateIds;
    private int _candidateCount;
    private int _borrowed;

    /// <summary>Creates scratch with the requested voxel and unique collider capacities.</summary>
    public PhysicsQueryScratch(int traversalCapacity, int candidateCapacity)
    {
        ChronicleGuard.ThrowIfNegative(traversalCapacity, nameof(traversalCapacity));
        ChronicleGuard.ThrowIfNegative(candidateCapacity, nameof(candidateCapacity));
        _voxels = new Voxel?[traversalCapacity];
        _candidateIds = new int[candidateCapacity];
    }

    /// <summary>Gets the maximum covered voxels that discovery can retain.</summary>
    public int TraversalCapacity => _voxels.Length;

    /// <summary>Gets the maximum distinct collider identities the query can retain.</summary>
    public int CandidateCapacity => _candidateIds.Length;

    internal Span<Voxel?> Voxels => _voxels;

    internal void Borrow()
    {
        ChronicleGuard.ThrowIfTrue(Interlocked.CompareExchange(ref _borrowed, 1, 0) != 0,
            nameof(PhysicsQueryScratch), "Query scratch is already in use.");
    }

    internal void Release(int voxelCount)
    {
        // Discovery prepaid release of every retained voxel. IDs carry no references.
        _voxels.AsSpan(0, voxelCount).Clear();
        _candidateCount = 0;
        Volatile.Write(ref _borrowed, 0);
    }

    internal PhysicsQueryExecutionStatus RememberCandidate(int id, ref PhysicsQueryMeter meter, out bool isNew)
    {
        isNew = false;
        for (int i = 0; i < _candidateCount; i++)
        {
            if (!meter.ChargeCandidate())
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (_candidateIds[i] == id)
                return PhysicsQueryExecutionStatus.Completed;
        }
        if (_candidateCount == _candidateIds.Length)
            return PhysicsQueryExecutionStatus.ScratchCapacityExceeded;
        _candidateIds[_candidateCount++] = id;
        isNew = true;
        return PhysicsQueryExecutionStatus.Completed;
    }

    internal PhysicsQueryExecutionStatus Discover(
        GridWorld world, Vector3d min, Vector3d max, bool planar,
        ref PhysicsQueryMeter meter, out int voxelCount, out ulong changeSequence)
    {
        var status = world.CollectCoveredVoxelsBounded(min, max, planar,
            meter.DiscoveryRemaining, _voxels, out ulong spent, out voxelCount, out changeSequence);
        meter.AddDiscovery(spent);
        return MapDiscoveryStatus(status);
    }

    internal static PhysicsQueryExecutionStatus CheckSnapshot(GridWorld world, ulong changeSequence) =>
        world.CheckBoundedVoxelSnapshot(changeSequence) == GridBoundedVoxelStatus.Completed
            ? PhysicsQueryExecutionStatus.Completed : PhysicsQueryExecutionStatus.Stale;

    internal static PhysicsQueryExecutionStatus MapDiscoveryStatus(GridBoundedVoxelStatus status) => status switch
    {
        GridBoundedVoxelStatus.Completed => PhysicsQueryExecutionStatus.Completed,
        GridBoundedVoxelStatus.WorkBudgetExceeded => PhysicsQueryExecutionStatus.WorkBudgetExceeded,
        GridBoundedVoxelStatus.CapacityExceeded => PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
        GridBoundedVoxelStatus.Stale => PhysicsQueryExecutionStatus.Stale,
        GridBoundedVoxelStatus.Unrepresentable => PhysicsQueryExecutionStatus.Unrepresentable,
        GridBoundedVoxelStatus.UnsupportedGrid => PhysicsQueryExecutionStatus.UnsupportedTopology,
        _ => PhysicsQueryExecutionStatus.WorldNotReady
    };
}
