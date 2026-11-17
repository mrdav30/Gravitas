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
using GridForge.Spatial;

namespace Gravitas.Queries;

/// <summary>Provides fixed, exclusively borrowed storage for a bounded physics query.</summary>
public sealed partial class PhysicsQueryScratch
{
    private readonly Voxel?[] _voxels;
    private readonly int[] _candidateIds;
    private int _candidateCount;
    private int _borrowed;

    /// <summary>Creates scratch with the requested voxel and unique collider capacities.</summary>
    public PhysicsQueryScratch(int traversalCapacity, int candidateCapacity)
        : this(traversalCapacity, candidateCapacity, 0)
    {
    }

    /// <summary>Creates scratch with space for voxels, unique colliders, and one mesh at a time.</summary>
    /// <param name="traversalCapacity">The number of covered voxels discovery can retain.</param>
    /// <param name="candidateCapacity">The number of distinct collider identities the query can retain.</param>
    /// <param name="meshTriangleCapacity">The maximum triangle count of any admitted mesh.</param>
    public PhysicsQueryScratch(int traversalCapacity, int candidateCapacity, int meshTriangleCapacity)
    {
        ChronicleGuard.ThrowIfNegative(traversalCapacity, nameof(traversalCapacity));
        ChronicleGuard.ThrowIfNegative(candidateCapacity, nameof(candidateCapacity));
        ChronicleGuard.ThrowIfArgumentOutOfRange(
            meshTriangleCapacity < 0 || meshTriangleCapacity > Colliders.PhysicsMesh.MaxTriangleCount,
            meshTriangleCapacity, nameof(meshTriangleCapacity), "Capacity must be within the supported mesh triangle range.");
        _voxels = new Voxel?[traversalCapacity];
        _candidateIds = new int[candidateCapacity];
        if (meshTriangleCapacity != 0)
            Mesh = new MeshQueryScratch(meshTriangleCapacity);
    }

    /// <summary>Gets the maximum covered voxels that discovery can retain.</summary>
    public int TraversalCapacity => _voxels.Length;

    /// <summary>Gets the maximum distinct collider identities the query can retain.</summary>
    public int CandidateCapacity => _candidateIds.Length;

    /// <summary>Gets the maximum triangle count of any mesh the query can examine.</summary>
    public int MeshTriangleCapacity => Mesh?.Capacity ?? 0;

    internal MeshQueryScratch? Mesh { get; }

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
        int first = 0;
        int last = _candidateCount - 1;
        while (first <= last)
        {
            if (!meter.ChargeCandidate())
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            int middle = first + (last - first) / 2;
            int candidate = _candidateIds[middle];
            if (candidate == id)
                return PhysicsQueryExecutionStatus.Completed;
            if (candidate < id)
                first = middle + 1;
            else
                last = middle - 1;
        }
        if (_candidateCount == _candidateIds.Length)
            return PhysicsQueryExecutionStatus.ScratchCapacityExceeded;
        int shifted = _candidateCount - first;
        if (!meter.ChargeCandidates((ulong)shifted))
            return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
        if (shifted != 0)
            Array.Copy(_candidateIds, first, _candidateIds, first + 1, shifted);
        _candidateIds[first] = id;
        _candidateCount++;
        isNew = true;
        return PhysicsQueryExecutionStatus.Completed;
    }

    internal PhysicsQueryExecutionStatus Discover(
        GravitasWorldContext context, Vector3d min, Vector3d max, bool planar,
        ref PhysicsQueryMeter meter, out int voxelCount, out ulong changeSequence)
    {
        var status = context.World.CollectCoveredVoxelsBounded(min, max, planar,
            meter.DiscoveryRemaining, _voxels, out ulong spent, out voxelCount, out changeSequence);
        meter.AddDiscovery(spent);
        return status == GridBoundedVoxelStatus.Completed
            ? ObservedViewStatus(context, changeSequence, PhysicsQueryExecutionStatus.Stale)
            : MapDiscoveryFailure(status);
    }

    internal static PhysicsQueryExecutionStatus CheckView(GravitasWorldContext context, ulong sequence,
        ref PhysicsQueryMeter meter, PhysicsQueryExecutionStatus unavailable = PhysicsQueryExecutionStatus.Stale)
    {
        var observed = ObservedViewStatus(context, sequence, unavailable);
        if (observed != PhysicsQueryExecutionStatus.Completed)
            return observed;
        if (meter.DiscoveryRemaining == 0)
            return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
        meter.AddDiscovery(1);
        return context.World.CheckBoundedVoxelSnapshot(sequence) == GridBoundedVoxelStatus.Completed
            ? ObservedViewStatus(context, sequence, unavailable) : unavailable;
    }

    private static PhysicsQueryExecutionStatus ObservedViewStatus(
        GravitasWorldContext context, ulong sequence, PhysicsQueryExecutionStatus unavailable) =>
        context.IsBoundedQueryViewCurrent(sequence) ? PhysicsQueryExecutionStatus.Completed : unavailable;

    internal static PhysicsQueryExecutionStatus GetNativePartition<T, TOtherDimension>(
        Voxel voxel, ref PhysicsQueryMeter meter, out T? partition)
        where T : class, IVoxelPartition
        where TOtherDimension : class, IVoxelPartition
    {
        var status = voxel.TryGetPartitionBounded<T, TOtherDimension, PhysicsMixedPartition>(
            meter.DiscoveryRemaining, out ulong work, out partition);
        meter.AddDiscovery(work);
        return status == GridBoundedVoxelStatus.Completed
            ? PhysicsQueryExecutionStatus.Completed : MapDiscoveryFailure(status);
    }

    private static PhysicsQueryExecutionStatus MapDiscoveryFailure(GridBoundedVoxelStatus status) => status switch
    {
        GridBoundedVoxelStatus.WorkBudgetExceeded => PhysicsQueryExecutionStatus.WorkBudgetExceeded,
        GridBoundedVoxelStatus.CapacityExceeded => PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
        GridBoundedVoxelStatus.Unrepresentable => PhysicsQueryExecutionStatus.Unrepresentable,
        _ => PhysicsQueryExecutionStatus.WorldNotReady
    };
}
