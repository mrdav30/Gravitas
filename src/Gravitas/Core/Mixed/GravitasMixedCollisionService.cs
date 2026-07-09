//=======================================================================
// GravitasMixedCollisionService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;
using GridForge.Grids;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>
/// Owns mixed 2D/3D collision lifecycle and broad-phase state for one <see cref="GravitasWorldContext"/>.
/// </summary>
internal sealed partial class GravitasMixedCollisionService
{
    private const int DefaultPartitionPoolCapacity = 1024;
    private static readonly PhysicsMixedPartitionOrderComparer PartitionOrderComparer = new();
    private static readonly MixedColliderKeyComparer CandidatePairComparer = new();
    private static readonly MixedResponsePairComparer ResponsePairComparer = new();
    private static readonly MixedIslandNodeComparer IslandNodeComparer = new();
    private static readonly MixedIslandConstraintComparer IslandConstraintComparer = new();
    private static readonly Collider2DIdComparer Collider2DIdOrderComparer = new();
    private static readonly Collider3DIdComparer Collider3DIdOrderComparer = new();

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsMixedPartition> _activePartitions = new(DefaultPartitionPoolCapacity);
    private readonly SwiftStack<PhysicsMixedPartition> _inactivePartitionPool = new(DefaultPartitionPoolCapacity);
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly SwiftHashSet<ulong> _processedPairKeys = new();
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly GridTraceScratch _traceScratch = new();
    private readonly SwiftList<PhysicsMixedPartition> _retainedPartitions = new();
    private readonly SwiftList<PhysicsMixedPartition> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamic3DIds = new();
    private readonly SwiftList<int> _distributionKinematic3DIds = new();
    private readonly SwiftList<int> _distributionStatic3DIds = new();
    private readonly SwiftList<int> _distributionDynamic2DIds = new();
    private readonly SwiftList<int> _distributionKinematic2DIds = new();
    private readonly SwiftList<int> _distributionStatic2DIds = new();
    private readonly SwiftList<MixedColliderKey> _candidatePairs = new();
    private readonly SwiftList<PhysicsMixedPartition> _queryPartitions = new();
    private readonly SwiftList<int> _queryColliderIds = new();
    private readonly SwiftHashSet<int> _queryColliderRedundancy = new();
    private readonly SwiftDictionary<ulong, CollisionPairMixed> _pairs = new();
    private readonly SwiftList<ulong> _pairsToRemove = new();
    private readonly SwiftStack<CollisionPairMixed> _cachedPairs = new();
    private readonly SwiftList<CollisionPairMixed> _mixedResponsePairs = new();
    private readonly SwiftList<MixedIslandNode> _mixedIslandNodes = new();
    private readonly SwiftList<MixedIslandConstraint> _mixedIslandConstraints = new();
    private readonly Action<PhysicsMixedPartition> _releaseRetainedPartition;

    private int _retainedPartitionRetirementCursor;
    private int _cached3DQueryRefreshFrame = int.MinValue;
    private int _cached3DQueryRefreshLateToken = int.MinValue;
    private int _cached2DQueryRefreshFrame = int.MinValue;
    private int _cached2DQueryRefreshLateToken = int.MinValue;

    internal GravitasMixedCollisionService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
        _releaseRetainedPartition = ReleasePartition;
    }

    internal GravitasWorldContext Context => _context;

    internal uint Version { get; private set; } = 1;

    internal int ActivePartitionCount => _activePartitions.Count;

    internal int InactivePartitionCount => _inactivePartitionPool.Count;

    internal int RetainedPartitionCount => _retainedPartitions.Count;

    internal int LastBroadPhaseCandidateCount { get; private set; }

    internal int ActivePairCount => _pairs.Count;

    internal int PooledPairCount => _cachedPairs.Count;

    internal int SimulateCount { get; private set; }

    internal int LateSimulateCount { get; private set; }

    internal int VisualizeCount { get; private set; }

    internal MixedColliderKey GetCandidate(int index) => _candidatePairs[index];

    internal void Simulate()
    {
        SimulateCount++;
        LastBroadPhaseCandidateCount = 0;
    }

    internal void LateSimulate()
    {
        LateSimulateCount++;
        ProcessMixedContacts();
    }

    private void ProcessMixedContacts()
    {
        LastBroadPhaseCandidateCount = 0;
        _candidatePairs.FastClear();
        _mixedResponsePairs.FastClear();
        _processedPairKeys.Clear();

        Refresh3DColliderPartitions();
        Refresh2DColliderPartitions(rebuildShapes: false);

        Version++;
        _distributionPartitions.FastClear();
        foreach (PhysicsMixedPartition partition in _activePartitions)
            _distributionPartitions.Add(partition);

        _distributionPartitions.SortInPlace(PartitionOrderComparer);
        for (int i = 0; i < _distributionPartitions.Count; i++)
        {
            _distributionPartitions[i].Distribute(
                _distributionDynamic3DIds,
                _distributionKinematic3DIds,
                _distributionStatic3DIds,
                _distributionDynamic2DIds,
                _distributionKinematic2DIds,
                _distributionStatic2DIds);
        }

        _candidatePairs.SortInPlace(CandidatePairComparer);
        LastBroadPhaseCandidateCount = _candidatePairs.Count;
        int frame = _context.FrameCount;
        for (int i = 0; i < _candidatePairs.Count; i++)
            ProcessCandidate(_candidatePairs[i], frame);

        SolveMixedResponsePairs();
        CleanupUntouchedPairs(frame);
        RetireExpiredRetainedPartitions();
    }

    internal void Visualize()
    {
        VisualizeCount++;
    }

    internal void Reset()
    {
        DetachRetainedPartitions();
        _activePartitions.Clear();
        _inactivePartitionPool.Clear();
        _redundancyChecker.Clear();
        _processedPairKeys.Clear();
        _coveredVoxels.FastClear();
        _traceScratch.Clear();
        _distributionPartitions.FastClear();
        _distributionDynamic3DIds.FastClear();
        _distributionKinematic3DIds.FastClear();
        _distributionStatic3DIds.FastClear();
        _distributionDynamic2DIds.FastClear();
        _distributionKinematic2DIds.FastClear();
        _distributionStatic2DIds.FastClear();
        _candidatePairs.FastClear();
        _queryPartitions.FastClear();
        _queryColliderIds.FastClear();
        _queryColliderRedundancy.Clear();
        _pairs.Clear();
        _pairsToRemove.FastClear();
        _cachedPairs.Clear();
        _mixedResponsePairs.FastClear();
        _mixedIslandNodes.FastClear();
        _mixedIslandConstraints.FastClear();
        _cached3DQueryRefreshFrame = int.MinValue;
        _cached3DQueryRefreshLateToken = int.MinValue;
        _cached2DQueryRefreshFrame = int.MinValue;
        _cached2DQueryRefreshLateToken = int.MinValue;
        Version = 1;
        LastBroadPhaseCandidateCount = 0;
        SimulateCount = 0;
        LateSimulateCount = 0;
        VisualizeCount = 0;
        _retainedPartitionRetirementCursor = 0;
    }

    internal void Deactivate() => Reset();

}
