//=======================================================================
// GravitasMixedCollisionService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns mixed 2D/3D collision lifecycle and broad-phase state for one <see cref="GravitasWorldContext"/>.
/// </summary>
internal sealed class GravitasMixedCollisionService
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

    private int _retainedPartitionRetirementCursor;
    private int _cached3DQueryRefreshFrame = int.MinValue;
    private int _cached3DQueryRefreshLateToken = int.MinValue;
    private int _cached2DQueryRefreshFrame = int.MinValue;
    private int _cached2DQueryRefreshLateToken = int.MinValue;

    internal GravitasMixedCollisionService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
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

    internal bool Refresh3DColliderPartition(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "3D collider must belong to this mixed collision service context.");

        if (!collider.IsActive || collider.Shape == ColliderType.None)
        {
            if (collider.IsMixedPartitioned)
                ClearPartitioned3DCollider(collider, force: true);

            return false;
        }

        Get3DCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind kind = Get3DMobilityKind(collider);
        if (collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)kind))
        {
            Refresh3DPartitionAwakeState(collider);
            return false;
        }

        if (collider.IsMixedPartitioned)
            ClearPartitioned3DCollider(collider, force: true);

        return Partition3DCollider(collider, coverageMin, coverageMax);
    }

    internal bool Refresh2DColliderPartition(LSCollider2D collider) =>
        Refresh2DColliderPartition(collider, rebuildShape: true);

    private bool Refresh2DColliderPartition(LSCollider2D collider, bool rebuildShape)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this mixed collision service context.");

        if (rebuildShape)
            collider.Rebuild();

        if (!collider.IsActive || collider.Shape == ColliderType2D.None)
        {
            if (collider.IsMixedPartitioned)
                ClearPartitioned2DCollider(collider, force: true);

            return false;
        }

        Get2DMixedCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind kind = Get2DMobilityKind(collider);
        if (collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)kind))
        {
            Refresh2DPartitionAwakeState(collider);
            return false;
        }

        if (collider.IsMixedPartitioned)
            ClearPartitioned2DCollider(collider, force: true);

        return Partition2DCollider(collider, coverageMin, coverageMax);
    }

    internal bool ClearPartitioned3DCollider(LSCollider collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned)
            return false;

        Get3DCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind currentKind = Get3DMobilityKind(collider);
        if (!force && collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)currentKind))
            return false;

        SwiftList<WorldVoxelIndex>? coordinates = collider.MixedPartitionCoordinates;
        if (coordinates == null)
            return false;

        GridWorld world = _context.World;
        MixedPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.MixedPartitionKind);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                Remove3DObject(partition!, collider.Id, partitionKind);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }

        collider.MarkMixedUnpartitioned();
        collider.ClearMixedPartitionCoordinates();
        return true;
    }

    internal bool ClearPartitioned2DCollider(LSCollider2D collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned)
            return false;

        Get2DMixedCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind currentKind = Get2DMobilityKind(collider);
        if (!force && collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)currentKind))
            return false;

        SwiftList<WorldVoxelIndex>? coordinates = collider.MixedPartitionCoordinates;
        if (coordinates == null)
            return false;

        GridWorld world = _context.World;
        MixedPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.MixedPartitionKind);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                Remove2DObject(partition!, collider.Id, partitionKind);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }

        collider.MarkMixedUnpartitioned();
        collider.ClearMixedPartitionCoordinates();
        return true;
    }

    internal void Refresh3DPartitionAwakeState(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned || collider.MixedPartitionCoordinates == null)
            return;

        StiffBody? body = collider.Body;
        if (body == null || body.Immovable)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;
        try
        {
            for (int i = 0; i < collider.MixedPartitionCoordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = collider.MixedPartitionCoordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                partition!.SetDynamic3DObjectAwake(collider.Id, awake);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal void Refresh2DPartitionAwakeState(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned || collider.MixedPartitionCoordinates == null)
            return;

        StiffBody2D? body = collider.Body;
        if (body == null || body.Immovable)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;
        try
        {
            for (int i = 0; i < collider.MixedPartitionCoordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = collider.MixedPartitionCoordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                partition!.SetDynamic2DObjectAwake(collider.Id, awake);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal void ProcessPartitionCandidate(int collider3DId, int collider2DId)
    {
        if (!_context.Physics.TryGetColliderById(collider3DId, out LSCollider? collider3D)
            || !_context.Physics2D.TryGetColliderById(collider2DId, out LSCollider2D? collider2D)
            || !RequireCollisionPair(collider3D!, collider2D!)
            || !MixedBoundsOverlap(collider3D!, collider2D!))
        {
            return;
        }

        ulong key = MixedColliderKey.CreateKey(collider3DId, collider2DId);
        if (!_processedPairKeys.Add(key))
            return;

        _candidatePairs.Add(new MixedColliderKey(collider3DId, collider2DId));
    }

    internal void Collect2DCandidatesInMixedBounds(
        Vector3d min,
        Vector3d max,
        PhysicsLayerMask layerMask,
        SwiftList<LSCollider2D> candidates,
        bool staticStyleOnly = false,
        bool cachePartitionRefresh = false)
    {
        SwiftThrowHelper.ThrowIfNull(candidates, nameof(candidates));
        Refresh2DColliderPartitionsForQuery(cachePartitionRefresh);
        candidates.FastClear();
        CollectCoveredMixedQueryPartitions(min, max, _queryPartitions);
        _queryPartitions.SortInPlace(PartitionOrderComparer);
        _queryColliderRedundancy.Clear();

        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsMixedPartition partition = _queryPartitions[i];
            if (staticStyleOnly)
                partition.CopyStaticStyle2DColliderIds(_queryColliderIds);
            else
                partition.Copy2DColliderIds(_queryColliderIds);

            for (int j = 0; j < _queryColliderIds.Count; j++)
            {
                int colliderId = _queryColliderIds[j];
                if (!_queryColliderRedundancy.Add(colliderId)
                    || !_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider)
                    || !Is2DQueryCandidate(collider!, min, max, layerMask))
                {
                    continue;
                }

                candidates.Add(collider!);
            }
        }

        _queryColliderRedundancy.Clear();
        candidates.SortInPlace(Collider2DIdOrderComparer);
    }

    internal void Collect3DCandidatesInMixedBounds(
        Vector3d min,
        Vector3d max,
        PhysicsLayerMask layerMask,
        SwiftList<LSCollider> candidates,
        bool staticStyleOnly = false,
        bool cachePartitionRefresh = false)
    {
        SwiftThrowHelper.ThrowIfNull(candidates, nameof(candidates));
        Refresh3DColliderPartitionsForQuery(cachePartitionRefresh);
        candidates.FastClear();
        CollectCoveredMixedQueryPartitions(min, max, _queryPartitions);
        _queryPartitions.SortInPlace(PartitionOrderComparer);
        _queryColliderRedundancy.Clear();

        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsMixedPartition partition = _queryPartitions[i];
            if (staticStyleOnly)
                partition.CopyStaticStyle3DColliderIds(_queryColliderIds);
            else
                partition.Copy3DColliderIds(_queryColliderIds);

            for (int j = 0; j < _queryColliderIds.Count; j++)
            {
                int colliderId = _queryColliderIds[j];
                if (!_queryColliderRedundancy.Add(colliderId)
                    || !_context.Physics.TryGetColliderById(colliderId, out LSCollider? collider)
                    || !Is3DQueryCandidate(collider!, min, max, layerMask))
                {
                    continue;
                }

                candidates.Add(collider!);
            }
        }

        _queryColliderRedundancy.Clear();
        candidates.SortInPlace(Collider3DIdOrderComparer);
    }

    internal void RemovePairsFor3DCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (!ReferenceEquals(pair.Collider3D, collider))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        RemoveQueuedPairs();
    }

    internal void RemovePairsFor2DCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (!ReferenceEquals(pair.Collider2D, collider))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        RemoveQueuedPairs();
    }

    internal bool RequireCollisionPair(LSCollider collider3D, LSCollider2D collider2D)
    {
        return collider3D.IsActive && collider2D.IsActive
            && collider3D.Shape != ColliderType.None && collider2D.Shape != ColliderType2D.None
            && (collider3D.Body != null || collider2D.Body != null)
            && !ReferenceEquals(collider3D.AgentOrNull, collider2D.AgentOrNull)
            && !collider3D.ExcludesMixedCollisionWith(collider2D)
            && !_context.Physics.IsLayerCollisionDisabled(collider3D.Layer, collider2D.Layer);
    }

    private void ProcessCandidate(MixedColliderKey candidate, int frame)
    {
        if (!_context.Physics.TryGetColliderById(candidate.Collider3DId, out LSCollider? collider3D)
            || !_context.Physics2D.TryGetColliderById(candidate.Collider2DId, out LSCollider2D? collider2D)
            || !RequireCollisionPair(collider3D!, collider2D!)
            || !MixedBoundsOverlap(collider3D!, collider2D!))
        {
            return;
        }

        LSCollider resolved3D = collider3D!;
        LSCollider2D resolved2D = collider2D!;
        bool hasPair = _pairs.TryGetValue(candidate.Key, out CollisionPairMixed pair);
        bool triggerPair = resolved3D.IsTrigger || resolved2D.IsTrigger;
        bool hasAwakeMovableParticipant = HasAwakeMovableParticipant(resolved3D, resolved2D);
        if (!hasPair && !triggerPair && !hasAwakeMovableParticipant)
            return;

        if (!CollisionDetectionMixed.TryCollide(resolved3D, resolved2D, out MixedContact contact))
            return;

        if (!triggerPair && !hasAwakeMovableParticipant)
        {
            if (hasPair)
            {
                pair!.MarkResting(frame, contact);
                _mixedResponsePairs.Add(pair);
            }

            return;
        }

        if (!hasPair)
        {
            pair = CreatePair(resolved3D, resolved2D);
            _pairs.Add(candidate.Key, pair);
        }

        pair!.MarkColliding(frame, contact);
        if (!triggerPair)
            _mixedResponsePairs.Add(pair);
    }

    private void CleanupUntouchedPairs(int frame)
    {
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (pair.LastFrame == frame)
                continue;

            if (TryKeepRestingPair(pair, frame))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        RemoveQueuedPairs();
    }

    private bool TryKeepRestingPair(CollisionPairMixed pair, int frame)
    {
        LSCollider collider3D = pair.Collider3D;
        LSCollider2D collider2D = pair.Collider2D;
        if (collider3D.IsTrigger
            || collider2D.IsTrigger
            || HasAwakeMovableParticipant(collider3D, collider2D)
            || !RequireCollisionPair(collider3D, collider2D)
            || !MixedBoundsOverlap(collider3D, collider2D)
            || !CollisionDetectionMixed.TryCollide(collider3D, collider2D, out _))
        {
            return false;
        }

        pair.MarkResting(frame);
        return true;
    }

    private CollisionPairMixed CreatePair(LSCollider collider3D, LSCollider2D collider2D)
    {
        if (_context.Settings.PoolingEnabled && _cachedPairs.Count > 0)
        {
            CollisionPairMixed pair = _cachedPairs.Pop();
            pair.Initialize(collider3D, collider2D);
            return pair;
        }

        return new CollisionPairMixed(collider3D, collider2D);
    }

    private void RemoveQueuedPairs()
    {
        for (int i = 0; i < _pairsToRemove.Count; i++)
        {
            ulong key = _pairsToRemove[i];
            if (_pairs.TryGetValue(key, out CollisionPairMixed pair))
                RecyclePair(pair);

            _pairs.Remove(key);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RecyclePair(CollisionPairMixed pair)
    {
        if (_context.Settings.PoolingEnabled)
            _cachedPairs.Push(pair);
    }

    private void SolveMixedResponsePairs()
    {
        if (_mixedResponsePairs.Count == 0)
            return;

        if (_mixedResponsePairs.Count == 1)
        {
            CollisionPairMixed pair = _mixedResponsePairs[0];
            if (!HasAwakeResponseParticipant(pair))
                return;

            pair.WakeSleepingBodiesForCollision();
            CollisionResponseMixed.Resolve(pair, pair.Contact);
            return;
        }

        _mixedResponsePairs.SortInPlace(ResponsePairComparer);
        BuildMixedIslands();
        if (_mixedIslandConstraints.Count == 0)
            return;

        _mixedIslandConstraints.SortInPlace(IslandConstraintComparer);

        int start = 0;
        while (start < _mixedIslandConstraints.Count)
        {
            int rootKey = _mixedIslandConstraints[start].RootKey;
            int end = start + 1;
            while (end < _mixedIslandConstraints.Count
                && _mixedIslandConstraints[end].RootKey == rootKey)
            {
                end++;
            }

            if (WakeMixedIslandBodies(rootKey))
                SolveMixedIslandRange(rootKey, start, end);

            start = end;
        }
    }

    private void BuildMixedIslands()
    {
        _mixedIslandNodes.FastClear();
        _mixedIslandConstraints.FastClear();

        for (int i = 0; i < _mixedResponsePairs.Count; i++)
        {
            CollisionPairMixed pair = _mixedResponsePairs[i];
            AddMixedIslandNodeIfMovable(pair.Collider3D.Body);
            AddMixedIslandNodeIfMovable(pair.Collider2D.Body);
        }

        SortAndDeduplicateMixedIslandNodes();
        if (_mixedIslandNodes.Count == 0)
            return;

        for (int i = 0; i < _mixedResponsePairs.Count; i++)
        {
            CollisionPairMixed pair = _mixedResponsePairs[i];
            int node3D = FindMixedIslandNode(pair.Collider3D.Body);
            int node2D = FindMixedIslandNode(pair.Collider2D.Body);
            if (node3D >= 0 && node2D >= 0)
                UnionMixedIslandNodes(node3D, node2D);
        }

        CompressMixedIslandRoots();

        for (int i = 0; i < _mixedResponsePairs.Count; i++)
        {
            CollisionPairMixed pair = _mixedResponsePairs[i];
            int node3D = FindMixedIslandNode(pair.Collider3D.Body);
            int node2D = FindMixedIslandNode(pair.Collider2D.Body);
            int rootKey = ResolveMixedConstraintRootKey(node3D, node2D);
            if (rootKey < 0)
                continue;

            _mixedIslandConstraints.Add(new MixedIslandConstraint(pair, rootKey, pair.Key));
        }
    }

    private void AddMixedIslandNodeIfMovable(StiffBody? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return;

        _mixedIslandNodes.Add(new MixedIslandNode(Create3DBodyKey(body!), body!, null));
    }

    private void AddMixedIslandNodeIfMovable(StiffBody2D? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return;

        _mixedIslandNodes.Add(new MixedIslandNode(Create2DBodyKey(body!), null, body!));
    }

    private void SortAndDeduplicateMixedIslandNodes()
    {
        if (_mixedIslandNodes.Count == 0)
            return;

        if (_mixedIslandNodes.Count == 1)
        {
            MixedIslandNode singleNode = _mixedIslandNodes[0];
            singleNode.ParentIndex = 0;
            singleNode.RootKey = singleNode.BodyKey;
            _mixedIslandNodes[0] = singleNode;
            return;
        }

        _mixedIslandNodes.SortInPlace(IslandNodeComparer);

        int writeIndex = 0;
        int previousKey = -1;
        for (int readIndex = 0; readIndex < _mixedIslandNodes.Count; readIndex++)
        {
            MixedIslandNode node = _mixedIslandNodes[readIndex];
            if (node.BodyKey == previousKey)
                continue;

            node.ParentIndex = writeIndex;
            node.RootKey = node.BodyKey;
            _mixedIslandNodes[writeIndex++] = node;
            previousKey = node.BodyKey;
        }

        while (_mixedIslandNodes.Count > writeIndex)
            _mixedIslandNodes.RemoveAt(_mixedIslandNodes.Count - 1);
    }

    private int FindMixedIslandNode(StiffBody? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return -1;

        return FindMixedIslandNode(Create3DBodyKey(body!));
    }

    private int FindMixedIslandNode(StiffBody2D? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return -1;

        return FindMixedIslandNode(Create2DBodyKey(body!));
    }

    private int FindMixedIslandNode(int key)
    {
        int low = 0;
        int high = _mixedIslandNodes.Count - 1;
        while (low <= high)
        {
            int mid = low + ((high - low) >> 1);
            int midKey = _mixedIslandNodes[mid].BodyKey;
            if (midKey == key)
                return mid;

            if (midKey < key)
                low = mid + 1;
            else
                high = mid - 1;
        }

        return -1;
    }

    private void UnionMixedIslandNodes(int nodeA, int nodeB)
    {
        int rootA = FindMixedIslandRoot(nodeA);
        int rootB = FindMixedIslandRoot(nodeB);
        if (rootA == rootB)
            return;

        int keyA = _mixedIslandNodes[rootA].BodyKey;
        int keyB = _mixedIslandNodes[rootB].BodyKey;
        int parent = keyA <= keyB ? rootA : rootB;
        int child = parent == rootA ? rootB : rootA;

        MixedIslandNode childNode = _mixedIslandNodes[child];
        childNode.ParentIndex = parent;
        childNode.RootKey = _mixedIslandNodes[parent].BodyKey;
        _mixedIslandNodes[child] = childNode;
    }

    private int FindMixedIslandRoot(int index)
    {
        int root = index;
        while (_mixedIslandNodes[root].ParentIndex != root)
            root = _mixedIslandNodes[root].ParentIndex;

        while (index != root)
        {
            MixedIslandNode node = _mixedIslandNodes[index];
            int parent = node.ParentIndex;
            node.ParentIndex = root;
            node.RootKey = _mixedIslandNodes[root].BodyKey;
            _mixedIslandNodes[index] = node;
            index = parent;
        }

        return root;
    }

    private void CompressMixedIslandRoots()
    {
        for (int i = 0; i < _mixedIslandNodes.Count; i++)
        {
            int root = FindMixedIslandRoot(i);
            MixedIslandNode node = _mixedIslandNodes[i];
            node.RootKey = _mixedIslandNodes[root].BodyKey;
            _mixedIslandNodes[i] = node;
        }
    }

    private int ResolveMixedConstraintRootKey(int node3D, int node2D)
    {
        if (node3D >= 0)
            return _mixedIslandNodes[node3D].RootKey;

        return node2D >= 0 ? _mixedIslandNodes[node2D].RootKey : -1;
    }

    private bool WakeMixedIslandBodies(int rootKey)
    {
        bool hasAwakeBody = false;
        for (int i = 0; i < _mixedIslandNodes.Count; i++)
        {
            MixedIslandNode node = _mixedIslandNodes[i];
            if (node.RootKey == rootKey && node.IsAwakeForCollision)
            {
                hasAwakeBody = true;
                break;
            }
        }

        if (!hasAwakeBody)
            return false;

        for (int i = 0; i < _mixedIslandNodes.Count; i++)
        {
            MixedIslandNode node = _mixedIslandNodes[i];
            if (node.RootKey == rootKey)
                node.WakeFromCollision();
        }

        return true;
    }

    private void SolveMixedIslandRange(int rootKey, int start, int end)
    {
        if (end - start == 1)
        {
            CollisionPairMixed pair = _mixedIslandConstraints[start].Pair;
            CollisionResponseMixed.Resolve(pair, pair.Contact);
            return;
        }

        int iterationLimit = _context.Settings.DiscreteSolverIterations;
        int iterationsUsed = 0;
        for (int iteration = 0; iteration < iterationLimit; iteration++)
        {
            bool applyPositionCorrection = iteration == 0;
            for (int i = start; i < end; i++)
            {
                CollisionPairMixed pair = _mixedIslandConstraints[i].Pair;
                CollisionResponseMixed.Resolve(
                    pair,
                    pair.Contact,
                    iteration,
                    iterationLimit,
                    applyPositionCorrection);
            }

            iterationsUsed = iteration + 1;
        }

        _context.Diagnostics.EmitMixedResponseIsland(
            rootKey,
            end - start,
            iterationsUsed,
            iterationsUsed >= iterationLimit);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeMovableParticipant(LSCollider collider3D, LSCollider2D collider2D) =>
        IsAwakeMovable(collider3D.Body) || IsAwakeMovable(collider2D.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeResponseParticipant(CollisionPairMixed pair) =>
        IsAwakeMovable(pair.Collider3D.Body) || IsAwakeMovable(pair.Collider2D.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody? body) =>
        body != null && body.Active && !body.Immovable && !body.IsKinematic && !body.IsSleeping && body.InverseMass > Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody2D? body) =>
        body != null && body.CanTranslate && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableMixedIslandBody(StiffBody? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableMixedIslandBody(StiffBody2D? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Create3DBodyKey(StiffBody body) =>
        body.DynamicId << 1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Create2DBodyKey(StiffBody2D body) =>
        (body.DynamicId << 1) | 1;

    internal int ActivatePartition(PhysicsMixedPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "Mixed partition must belong to this collision service.");

        return _activePartitions.Add(partition);
    }

    internal void DeactivatePartition(int activationId)
    {
        if (activationId < 0)
            return;

        _activePartitions.TryRemoveAt(activationId);
    }

    internal PhysicsMixedPartition RentPartition()
    {
        PhysicsMixedPartition partition = _inactivePartitionPool.Count > 0
            ? _inactivePartitionPool.Pop()
            : new PhysicsMixedPartition();
        partition.SetOwner(this);
        return partition;
    }

    internal void ReleasePartition(PhysicsMixedPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "Mixed partition must be released through its owning collision service.");

        UntrackRetainedPartition(partition);
        partition.ResetForPool();
        _inactivePartitionPool.Push(partition);
    }

    private void Refresh3DColliderPartitions()
    {
        int peak = _context.Physics.PeakColliderCount;
        for (int id = 1; id <= peak; id++)
            if (_context.Physics.TryGetColliderById(id, out LSCollider? collider))
                Refresh3DColliderPartition(collider!);
    }

    private void Refresh3DColliderPartitionsForQuery(bool cachePartitionRefresh)
    {
        if (!cachePartitionRefresh)
        {
            Refresh3DColliderPartitions();
            return;
        }

        int frame = _context.FrameCount;
        int lateToken = _context.LateSimulateToken;
        // CCD batches query stable opposite-dimension targets many times in one late-sim phase.
        // Cache only the partition refresh; every sweep still gathers and sorts its own hits.
        if (_cached3DQueryRefreshFrame == frame && _cached3DQueryRefreshLateToken == lateToken)
            return;

        Refresh3DColliderPartitions();
        _cached3DQueryRefreshFrame = frame;
        _cached3DQueryRefreshLateToken = lateToken;
    }

    private void Refresh2DColliderPartitions(bool rebuildShapes = true)
    {
        int count = _context.Physics2D.ColliderCount;
        for (int i = 0; i < count; i++)
            if (_context.Physics2D.TryGetColliderByServiceIndex(i, out LSCollider2D? collider))
                Refresh2DColliderPartition(collider!, rebuildShapes);
    }

    private void Refresh2DColliderPartitionsForQuery(bool cachePartitionRefresh)
    {
        if (!cachePartitionRefresh)
        {
            Refresh2DColliderPartitions();
            return;
        }

        int frame = _context.FrameCount;
        int lateToken = _context.LateSimulateToken;
        // CCD batches query stable opposite-dimension targets many times in one late-sim phase.
        // Cache only the partition refresh; every sweep still gathers and sorts its own hits.
        if (_cached2DQueryRefreshFrame == frame && _cached2DQueryRefreshLateToken == lateToken)
            return;

        Refresh2DColliderPartitions();
        _cached2DQueryRefreshFrame = frame;
        _cached2DQueryRefreshLateToken = lateToken;
    }

    private bool Partition3DCollider(LSCollider collider, Vector3d coverageMin, Vector3d coverageMax)
    {
        if (collider.IsMixedPartitioned || !collider.IsActive)
            return false;

        MixedPartitionMobilityKind kind = Get3DMobilityKind(collider);
        SwiftList<WorldVoxelIndex> coordinates = collider.GetOrCreateMixedPartitionCoordinates();
        coordinates.FastClear();

        try
        {
            ScanCovered3DVoxels(collider, coverageMin, coverageMax, coordinates, kind);
            if (coordinates.Count == 0)
                return false;

            collider.MarkMixedPartitioned(coverageMin, coverageMax, (int)kind);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private bool Partition2DCollider(LSCollider2D collider, Vector3d coverageMin, Vector3d coverageMax)
    {
        if (collider.IsMixedPartitioned || !collider.IsActive)
            return false;

        MixedPartitionMobilityKind kind = Get2DMobilityKind(collider);
        SwiftList<WorldVoxelIndex> coordinates = collider.GetOrCreateMixedPartitionCoordinates();
        coordinates.FastClear();

        try
        {
            ScanCovered2DMixedVoxels(collider, coverageMin, coverageMax, coordinates, kind);
            if (coordinates.Count == 0)
                return false;

            collider.MarkMixedPartitioned(coverageMin, coverageMax, (int)kind);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private void ScanCovered3DVoxels(
        LSCollider collider,
        Vector3d coverageMin,
        Vector3d coverageMax,
        SwiftList<WorldVoxelIndex> coordinates,
        MixedPartitionMobilityKind kind)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            coverageMin,
            coverageMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartition3DVoxel(collider, coordinates, _coveredVoxels[i], ref traversal, kind);
    }

    private void ScanCovered2DMixedVoxels(
        LSCollider2D collider,
        Vector3d coverageMin,
        Vector3d coverageMax,
        SwiftList<WorldVoxelIndex> coordinates,
        MixedPartitionMobilityKind kind)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            coverageMin,
            coverageMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartition2DMixedVoxel(collider, coordinates, _coveredVoxels[i], ref traversal, kind);
    }

    private void CollectCoveredMixedQueryPartitions(
        Vector3d min,
        Vector3d max,
        SwiftList<PhysicsMixedPartition> partitions)
    {
        partitions.FastClear();

        try
        {
            ScanCoveredMixedQueryPartitions(min, max, partitions);
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private void ScanCoveredMixedQueryPartitions(
        Vector3d queryMin,
        Vector3d queryMax,
        SwiftList<PhysicsMixedPartition> partitions)
    {
        GridTracer.GetCoveredVoxelsInto(
            _context.World,
            queryMin,
            queryMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        VisitCoveredVoxelsForMixedQuery(queryMin, queryMax, partitions);
    }

    private void VisitCoveredVoxelsForMixedQuery(
        Vector3d queryMin,
        Vector3d queryMax,
        SwiftList<PhysicsMixedPartition> partitions)
    {
        GridWorld world = _context.World;
        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];

            if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
                || !GridTraversal.IsWorldPositionInPaddedBounds(queryMin, queryMax, cellEdge, voxel.WorldPosition)
                || !voxel.TryGetPartition(out PhysicsMixedPartition? partition)
                || partition!.IsEmpty)
            {
                continue;
            }

            partitions.Add(partition);
        }
    }

    private void TryPartition3DVoxel(
        LSCollider collider,
        SwiftList<WorldVoxelIndex> coordinates,
        Voxel voxel,
        ref GridTraversalState traversal,
        MixedPartitionMobilityKind kind)
    {
        if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
            || !collider.IsPositionInBounds(cellEdge, voxel.WorldPosition))
        {
            return;
        }

        PhysicsMixedPartition partition = GetOrCreatePartition(voxel);
        coordinates.Add(voxel.WorldIndex);
        Add3DObject(partition, collider.Id, kind);
    }

    private void TryPartition2DMixedVoxel(
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> coordinates,
        Voxel voxel,
        ref GridTraversalState traversal,
        MixedPartitionMobilityKind kind)
    {
        if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
            || !collider.IsPositionInMixedBounds(cellEdge, voxel.WorldPosition))
        {
            return;
        }

        PhysicsMixedPartition partition = GetOrCreatePartition(voxel);
        coordinates.Add(voxel.WorldIndex);
        Add2DObject(partition, collider.Id, kind);
    }

    private PhysicsMixedPartition GetOrCreatePartition(Voxel voxel)
    {
        if (voxel.TryGetPartition(out PhysicsMixedPartition? partition))
            return partition!;

        partition = RentPartition();
        if (voxel.TryAddPartition(partition))
        {
            TrackRetainedPartition(partition);
            return partition;
        }

        ReleasePartition(partition);
        SwiftThrowHelper.ThrowIfTrue(
            true,
            nameof(GravitasMixedCollisionService),
            "Unable to attach mixed physics partition to voxel.");
        return partition;
    }

    private void Get3DCoverageBounds(LSCollider collider, out Vector3d coverageMin, out Vector3d coverageMax)
    {
        coverageMin = collider.BoundsMin;
        coverageMax = collider.BoundsMax;
    }

    private void Get2DMixedCoverageBounds(LSCollider2D collider, out Vector3d coverageMin, out Vector3d coverageMax)
    {
        FixedBoundBox bounds = collider.MixedBounds3D;
        coverageMin = bounds.Min;
        coverageMax = bounds.Max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static MixedPartitionMobilityKind Get3DMobilityKind(LSCollider collider)
    {
        StiffBody? body = collider.Body;
        if (body == null || body.Immovable)
            return MixedPartitionMobilityKind.Static;

        return body.IsKinematic ? MixedPartitionMobilityKind.Kinematic : MixedPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static MixedPartitionMobilityKind Get2DMobilityKind(LSCollider2D collider)
    {
        StiffBody2D? body = collider.Body;
        if (body == null || body.Immovable)
            return MixedPartitionMobilityKind.Static;

        return body.IsKinematic ? MixedPartitionMobilityKind.Kinematic : MixedPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static MixedPartitionMobilityKind GetStoredMobilityKind(int partitionKind)
    {
        return partitionKind == (int)MixedPartitionMobilityKind.Kinematic
            ? MixedPartitionMobilityKind.Kinematic
            : partitionKind == (int)MixedPartitionMobilityKind.Static
                ? MixedPartitionMobilityKind.Static
                : MixedPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Add3DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.AddStatic3DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.AddKinematic3DObject(id);
            return;
        }

        partition.AddDynamic3DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Add2DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.AddStatic2DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.AddKinematic2DObject(id);
            return;
        }

        partition.AddDynamic2DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Remove3DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.RemoveStatic3DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.RemoveKinematic3DObject(id);
            return;
        }

        partition.RemoveDynamic3DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Remove2DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.RemoveStatic2DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.RemoveKinematic2DObject(id);
            return;
        }

        partition.RemoveDynamic2DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool MixedBoundsOverlap(LSCollider collider3D, LSCollider2D collider2D)
    {
        FixedBoundBox bounds2D = collider2D.MixedBounds3D;
        return collider3D.BoundsMax.X >= bounds2D.Min.X
            && collider3D.BoundsMin.X <= bounds2D.Max.X
            && collider3D.BoundsMax.Y >= bounds2D.Min.Y
            && collider3D.BoundsMin.Y <= bounds2D.Max.Y
            && collider3D.BoundsMax.Z >= bounds2D.Min.Z
            && collider3D.BoundsMin.Z <= bounds2D.Max.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool Is2DQueryCandidate(LSCollider2D collider, Vector3d min, Vector3d max, PhysicsLayerMask layerMask)
    {
        if (!collider.IsActive || !layerMask.Includes(collider.Layer))
            return false;

        FixedBoundBox bounds = collider.MixedBounds3D;
        return BoundsOverlap(bounds.Min, bounds.Max, min, max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool Is3DQueryCandidate(LSCollider collider, Vector3d min, Vector3d max, PhysicsLayerMask layerMask)
    {
        return collider.IsActive
            && layerMask.Includes(collider.Layer)
            && BoundsOverlap(collider.BoundsMin, collider.BoundsMax, min, max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlap(Vector3d firstMin, Vector3d firstMax, Vector3d secondMin, Vector3d secondMax)
    {
        return firstMax.X >= secondMin.X
            && firstMin.X <= secondMax.X
            && firstMax.Y >= secondMin.Y
            && firstMin.Y <= secondMax.Y
            && firstMax.Z >= secondMin.Z
            && firstMin.Z <= secondMax.Z;
    }

    private void DetachRetainedPartitions()
    {
        // Reset is a context boundary; retained GridForge payloads are a runtime cache, not replay state.
        while (_retainedPartitions.Count > 0)
        {
            PhysicsMixedPartition partition = _retainedPartitions[_retainedPartitions.Count - 1];
            if (!partition.IsOwnedBy(this))
            {
                UntrackRetainedPartition(partition);
                continue;
            }

            if (_context.World.TryGetVoxel(partition.WorldIndex, out Voxel? voxel)
                && voxel!.TryGetPartition(out PhysicsMixedPartition? attachedPartition)
                && ReferenceEquals(attachedPartition, partition))
            {
                bool removed = voxel.TryRemovePartition<PhysicsMixedPartition>();
                SwiftThrowHelper.ThrowIfTrue(
                    !removed,
                    nameof(PhysicsMixedPartition),
                    "Unable to detach retained mixed physics partition from its voxel during reset.");

                if (partition.IsOwnedBy(this))
                    ReleasePartition(partition);

                continue;
            }

            ReleasePartition(partition);
        }
    }

    private void TrackRetainedPartition(PhysicsMixedPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            partition.RetainedIndex >= 0,
            nameof(partition),
            "PhysicsMixedPartition is already tracked as retained.");

        partition.SetRetainedIndex(_retainedPartitions.Count);
        _retainedPartitions.Add(partition);
    }

    private void UntrackRetainedPartition(PhysicsMixedPartition partition)
    {
        int index = FindRetainedPartitionIndex(partition);
        if (index < 0)
        {
            partition.ClearRetainedIndex();
            return;
        }

        int lastIndex = _retainedPartitions.Count - 1;
        if (index != lastIndex)
        {
            PhysicsMixedPartition movedPartition = _retainedPartitions[lastIndex];
            _retainedPartitions[index] = movedPartition;
            movedPartition.SetRetainedIndex(index);
        }

        _retainedPartitions.RemoveAt(lastIndex);
        partition.ClearRetainedIndex();

        if (_retainedPartitionRetirementCursor > index)
            _retainedPartitionRetirementCursor--;
        if (_retainedPartitionRetirementCursor >= _retainedPartitions.Count)
            _retainedPartitionRetirementCursor = 0;
    }

    private int FindRetainedPartitionIndex(PhysicsMixedPartition partition)
    {
        int index = partition.RetainedIndex;
        if ((uint)index < (uint)_retainedPartitions.Count && ReferenceEquals(_retainedPartitions[index], partition))
            return index;

        for (int i = 0; i < _retainedPartitions.Count; i++)
            if (ReferenceEquals(_retainedPartitions[i], partition))
                return i;

        return -1;
    }

    private void RetireExpiredRetainedPartitions()
    {
        int budget = _context.Settings.RetainedPartitionRetirementSweepBudget;
        if (budget <= 0 || _retainedPartitions.Count == 0)
            return;

        int inspected = 0;
        while (inspected < budget && _retainedPartitions.Count > 0)
        {
            if (_retainedPartitionRetirementCursor >= _retainedPartitions.Count)
                _retainedPartitionRetirementCursor = 0;

            PhysicsMixedPartition partition = _retainedPartitions[_retainedPartitionRetirementCursor];
            inspected++;

            if (!ShouldRetireRetainedPartition(partition))
            {
                _retainedPartitionRetirementCursor++;
                continue;
            }

            if (!RetireRetainedPartition(partition))
                _retainedPartitionRetirementCursor++;
        }
    }

    private bool ShouldRetireRetainedPartition(PhysicsMixedPartition partition)
    {
        if (!partition.IsOwnedBy(this) || !partition.IsEmpty || partition.IsAllocated || partition.EmptySinceFrame < 0)
            return false;

        int idleFrames = _context.FrameCount - partition.EmptySinceFrame;
        return idleFrames >= _context.Settings.RetainedPartitionTimeToKillFrames;
    }

    private bool RetireRetainedPartition(PhysicsMixedPartition partition)
    {
        if (!_context.World.TryGetVoxel(partition.WorldIndex, out Voxel? voxel))
        {
            ReleasePartition(partition);
            return true;
        }

        if (!voxel!.TryGetPartition(out PhysicsMixedPartition? attachedPartition)
            || !ReferenceEquals(attachedPartition, partition))
        {
            ReleasePartition(partition);
            return true;
        }

        return voxel.TryRemovePartition<PhysicsMixedPartition>();
    }

    private sealed class PhysicsMixedPartitionOrderComparer : IComparer<PhysicsMixedPartition>
    {
        public int Compare(PhysicsMixedPartition? left, PhysicsMixedPartition? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            WorldVoxelIndex leftIndex = left.WorldIndex;
            WorldVoxelIndex rightIndex = right.WorldIndex;

            int compare = leftIndex.GridIndex.CompareTo(rightIndex.GridIndex);
            if (compare != 0)
                return compare;

            compare = leftIndex.GridSpawnToken.CompareTo(rightIndex.GridSpawnToken);
            if (compare != 0)
                return compare;

            compare = leftIndex.VoxelIndex.x.CompareTo(rightIndex.VoxelIndex.x);
            if (compare != 0)
                return compare;

            compare = leftIndex.VoxelIndex.y.CompareTo(rightIndex.VoxelIndex.y);
            if (compare != 0)
                return compare;

            return leftIndex.VoxelIndex.z.CompareTo(rightIndex.VoxelIndex.z);
        }
    }

    private sealed class MixedColliderKeyComparer : IComparer<MixedColliderKey>
    {
        public int Compare(MixedColliderKey left, MixedColliderKey right) =>
            left.Key.CompareTo(right.Key);
    }

    private sealed class MixedResponsePairComparer : IComparer<CollisionPairMixed>
    {
        public int Compare(CollisionPairMixed? left, CollisionPairMixed? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            return left.Key.CompareTo(right.Key);
        }
    }

    private sealed class MixedIslandNodeComparer : IComparer<MixedIslandNode>
    {
        public int Compare(MixedIslandNode left, MixedIslandNode right) =>
            left.BodyKey.CompareTo(right.BodyKey);
    }

    private sealed class MixedIslandConstraintComparer : IComparer<MixedIslandConstraint>
    {
        public int Compare(MixedIslandConstraint left, MixedIslandConstraint right)
        {
            int compare = left.RootKey.CompareTo(right.RootKey);
            if (compare != 0)
                return compare;

            return left.PairKey.CompareTo(right.PairKey);
        }
    }

    private sealed class Collider2DIdComparer : IComparer<LSCollider2D>
    {
        public int Compare(LSCollider2D? left, LSCollider2D? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            return left.Id.CompareTo(right.Id);
        }
    }

    private sealed class Collider3DIdComparer : IComparer<LSCollider>
    {
        public int Compare(LSCollider? left, LSCollider? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            return left.Id.CompareTo(right.Id);
        }
    }

    private struct MixedIslandNode
    {
        public MixedIslandNode(int bodyKey, StiffBody? body3D, StiffBody2D? body2D)
        {
            BodyKey = bodyKey;
            Body3D = body3D;
            Body2D = body2D;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey { get; }

        public StiffBody? Body3D { get; }

        public StiffBody2D? Body2D { get; }

        public int ParentIndex { get; set; }

        public int RootKey { get; set; }

        public bool IsAwakeForCollision =>
            Body3D?.IsAwakeForCollision ?? Body2D!.IsAwakeForCollision;

        public void WakeFromCollision()
        {
            if (Body3D != null)
            {
                Body3D.WakeFromCollision();
                return;
            }

            Body2D!.WakeFromCollision();
        }
    }

    private readonly struct MixedIslandConstraint
    {
        public MixedIslandConstraint(CollisionPairMixed pair, int rootKey, ulong pairKey)
        {
            Pair = pair;
            RootKey = rootKey;
            PairKey = pairKey;
        }

        public CollisionPairMixed Pair { get; }

        public int RootKey { get; }

        public ulong PairKey { get; }
    }
}
