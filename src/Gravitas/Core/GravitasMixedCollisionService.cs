using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using SwiftCollections.Utility;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns mixed 2D/3D collision lifecycle and broad-phase state for one <see cref="GravitasWorldContext"/>.
/// </summary>
internal sealed class GravitasMixedCollisionService
{
    private const int DefaultPartitionPoolCapacity = 1024;

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
    private readonly SwiftList<int> _distributionAwakeDynamic3DIds = new();
    private readonly SwiftList<int> _distributionKinematic3DIds = new();
    private readonly SwiftList<int> _distributionStatic3DIds = new();
    private readonly SwiftList<int> _distributionDynamic2DIds = new();
    private readonly SwiftList<int> _distributionAwakeDynamic2DIds = new();
    private readonly SwiftList<int> _distributionKinematic2DIds = new();
    private readonly SwiftList<int> _distributionStatic2DIds = new();
    private readonly SwiftList<MixedColliderKey> _candidatePairs = new();
    private readonly SwiftList<PhysicsMixedPartition> _queryPartitions = new();
    private readonly SwiftList<int> _queryColliderIds = new();
    private readonly SwiftHashSet<int> _queryColliderRedundancy = new();
    private readonly SwiftDictionary<ulong, CollisionPairMixed> _pairs = new();
    private readonly SwiftList<ulong> _pairsToRemove = new();
    private readonly SwiftStack<CollisionPairMixed> _cachedPairs = new();

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
        _candidatePairs.FastClear();
        _processedPairKeys.Clear();

        Refresh3DColliderPartitions();
        Refresh2DColliderPartitions();

        Version++;
        _distributionPartitions.FastClear();
        foreach (PhysicsMixedPartition partition in _activePartitions)
            _distributionPartitions.Add(partition);

        SortPartitions(_distributionPartitions);
        for (int i = 0; i < _distributionPartitions.Count; i++)
        {
            _distributionPartitions[i].Distribute(
                _distributionDynamic3DIds,
                _distributionAwakeDynamic3DIds,
                _distributionKinematic3DIds,
                _distributionStatic3DIds,
                _distributionDynamic2DIds,
                _distributionAwakeDynamic2DIds,
                _distributionKinematic2DIds,
                _distributionStatic2DIds);
        }

        SortCandidatePairs(_candidatePairs);
        LastBroadPhaseCandidateCount = _candidatePairs.Count;
        int frame = _context.FrameCount;
        for (int i = 0; i < _candidatePairs.Count; i++)
            ProcessCandidate(_candidatePairs[i], frame);

        CleanupUntouchedPairs(frame);
        RetireExpiredRetainedPartitions();
    }

    internal void LateSimulate()
    {
        LateSimulateCount++;
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
        _distributionAwakeDynamic3DIds.FastClear();
        _distributionKinematic3DIds.FastClear();
        _distributionStatic3DIds.FastClear();
        _distributionDynamic2DIds.FastClear();
        _distributionAwakeDynamic2DIds.FastClear();
        _distributionKinematic2DIds.FastClear();
        _distributionStatic2DIds.FastClear();
        _candidatePairs.FastClear();
        _queryPartitions.FastClear();
        _queryColliderIds.FastClear();
        _queryColliderRedundancy.Clear();
        _pairs.Clear();
        _pairsToRemove.FastClear();
        _cachedPairs.Clear();
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

    internal bool Refresh2DColliderPartition(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this mixed collision service context.");

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
        SortPartitions(_queryPartitions);
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
        Sort2DCollidersById(candidates);
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
        SortPartitions(_queryPartitions);
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
        Sort3DCollidersById(candidates);
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
            || !MixedBoundsOverlap(collider3D!, collider2D!)
            || !CollisionDetectionMixed.TryCollide(collider3D!, collider2D!, out MixedContact contact))
        {
            return;
        }

        LSCollider resolved3D = collider3D!;
        LSCollider2D resolved2D = collider2D!;
        bool hasPair = _pairs.TryGetValue(candidate.Key, out CollisionPairMixed pair);
        bool triggerPair = resolved3D.IsTrigger || resolved2D.IsTrigger;
        if (!triggerPair && !HasAwakeMovableParticipant(resolved3D, resolved2D))
        {
            if (hasPair)
                pair!.MarkResting(frame);

            return;
        }

        if (!hasPair)
        {
            pair = CreatePair(resolved3D, resolved2D);
            _pairs.Add(candidate.Key, pair);
        }

        pair!.MarkColliding(frame, contact);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeMovableParticipant(LSCollider collider3D, LSCollider2D collider2D) =>
        IsAwakeMovable(collider3D.Body) || IsAwakeMovable(collider2D.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody? body) =>
        body != null && body.Active && !body.Immovable && !body.IsKinematic && !body.IsSleeping && body.InverseMass > Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody2D? body) =>
        body != null && body.CanTranslate && !body.IsSleeping;

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

    private void Refresh2DColliderPartitions()
    {
        int count = _context.Physics2D.ColliderCount;
        for (int i = 0; i < count; i++)
            if (_context.Physics2D.TryGetColliderByServiceIndex(i, out LSCollider2D? collider))
                Refresh2DColliderPartition(collider!);
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

    private static void Sort2DCollidersById(SwiftList<LSCollider2D> colliders)
    {
        for (int i = 1; i < colliders.Count; i++)
        {
            LSCollider2D value = colliders[i];
            int index = i - 1;
            while (index >= 0 && colliders[index].Id > value.Id)
            {
                colliders[index + 1] = colliders[index];
                index--;
            }

            colliders[index + 1] = value;
        }
    }

    private static void Sort3DCollidersById(SwiftList<LSCollider> colliders)
    {
        for (int i = 1; i < colliders.Count; i++)
        {
            LSCollider value = colliders[i];
            int index = i - 1;
            while (index >= 0 && colliders[index].Id > value.Id)
            {
                colliders[index + 1] = colliders[index];
                index--;
            }

            colliders[index + 1] = value;
        }
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

    private static void SortPartitions(SwiftList<PhysicsMixedPartition> partitions)
    {
        if (partitions.Count < 2)
            return;

        QuickSortPartitions(partitions, 0, partitions.Count - 1);
    }

    private static void QuickSortPartitions(SwiftList<PhysicsMixedPartition> partitions, int left, int right)
    {
        while (left < right)
        {
            if (right - left <= 16)
            {
                InsertionSortPartitions(partitions, left, right);
                return;
            }

            int i = left;
            int j = right;
            PhysicsMixedPartition pivot = partitions[left + ((right - left) / 2)];
            while (i <= j)
            {
                while (ComparePartitions(partitions[i], pivot) < 0)
                    i++;
                while (ComparePartitions(partitions[j], pivot) > 0)
                    j--;

                if (i > j)
                    continue;

                if (i != j)
                    (partitions[i], partitions[j]) = (partitions[j], partitions[i]);

                i++;
                j--;
            }

            if (j - left < right - i)
            {
                if (left < j)
                    QuickSortPartitions(partitions, left, j);

                left = i;
            }
            else
            {
                if (i < right)
                    QuickSortPartitions(partitions, i, right);

                right = j;
            }
        }
    }

    private static void InsertionSortPartitions(SwiftList<PhysicsMixedPartition> partitions, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            PhysicsMixedPartition value = partitions[i];
            int index = i - 1;
            while (index >= left && ComparePartitions(partitions[index], value) > 0)
            {
                partitions[index + 1] = partitions[index];
                index--;
            }

            partitions[index + 1] = value;
        }
    }

    private static int ComparePartitions(PhysicsMixedPartition left, PhysicsMixedPartition right)
    {
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

    private static void SortCandidatePairs(SwiftList<MixedColliderKey> pairs)
    {
        if (pairs.Count < 2)
            return;

        QuickSortCandidatePairs(pairs, 0, pairs.Count - 1);
    }

    private static void QuickSortCandidatePairs(SwiftList<MixedColliderKey> pairs, int left, int right)
    {
        while (left < right)
        {
            if (right - left <= 16)
            {
                InsertionSortCandidatePairs(pairs, left, right);
                return;
            }

            int i = left;
            int j = right;
            MixedColliderKey pivot = pairs[left + ((right - left) / 2)];
            while (i <= j)
            {
                while (pairs[i].Key < pivot.Key)
                    i++;
                while (pairs[j].Key > pivot.Key)
                    j--;

                if (i > j)
                    continue;

                if (i != j)
                    (pairs[i], pairs[j]) = (pairs[j], pairs[i]);

                i++;
                j--;
            }

            if (j - left < right - i)
            {
                if (left < j)
                    QuickSortCandidatePairs(pairs, left, j);

                left = i;
            }
            else
            {
                if (i < right)
                    QuickSortCandidatePairs(pairs, i, right);

                right = j;
            }
        }
    }

    private static void InsertionSortCandidatePairs(SwiftList<MixedColliderKey> pairs, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            MixedColliderKey value = pairs[i];
            int index = i - 1;
            while (index >= left && pairs[index].Key > value.Key)
            {
                pairs[index + 1] = pairs[index];
                index--;
            }

            pairs[index + 1] = value;
        }
    }
}
