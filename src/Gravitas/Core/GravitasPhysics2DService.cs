//=======================================================================
// GravitasPhysics2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Spatial;
using SwiftCollections;
using SwiftCollections.Query;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Context-owned pure 2D body, broad-phase, narrow-phase, response, and query service.
/// </summary>
public sealed class GravitasPhysics2DService
{
    private static readonly CollisionPair2DStableKeyComparer ResponsePairComparer = new();
    private static readonly DiscreteIslandNode2DComparer IslandNodeComparer = new();
    private static readonly DiscreteIslandConstraint2DComparer IslandConstraintComparer = new();

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<StiffBody2D> _dynamicBodies = new();
    private readonly SwiftList<LSCollider2D> _colliders = new();
    private readonly SwiftDictionary<int, LSCollider2D> _collidersById = new();
    private readonly SwiftHashSet<ulong> _processedPairKeys = new();
    private readonly SwiftDictionary<ulong, CollisionPair2D> _pairs = new();
    private readonly SwiftList<ulong> _pairsToRemove = new();
    private readonly SwiftStack<CollisionPair2D> _cachedPairs = new();
    private readonly SwiftList<CollisionPair2D> _discreteResponsePairs = new();
    private readonly SwiftHashSet<int> _discreteResponseBodyKeys = new();
    private readonly SwiftList<int> _discreteResponseBodyQueue = new();
    private readonly SwiftList<DiscreteIslandNode2D> _discreteIslandNodes = new();
    private readonly SwiftList<DiscreteIslandConstraint2D> _discreteIslandConstraints = new();
    private readonly DynamicCcdCandidateIndex2D _planarContinuousCollisionCandidates = new();
    private readonly DynamicCcdCandidateIndex _mixedContinuousCollisionCandidates = new();
    private readonly SwiftList<int> _continuousCollisionCandidateIds = new();
    private readonly SwiftHashSet<int> _processedContinuousCollisionBodyIds = new();
    private readonly SwiftHashSet<int> _queuedContinuousCollisionHandoffIds = new();
    private readonly SwiftList<int> _continuousCollisionHandoffQueue = new();
    private int _continuousCollisionPreparedToken = int.MinValue;
    private bool _continuousCollisionPreparedMixedIndex;
    private int _nextColliderId = 1;

    public GravitasPhysics2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    public int BodyCount { get; private set; }

    public int ColliderCount => _colliders.Count;

    public bool SimulatePhysics { get; set; } = true;

    internal int LastBroadPhaseCandidateCount { get; private set; }

    internal int LastContinuousCollisionIslandCount { get; private set; }

    internal int LastContinuousCollisionIslandIterationCount { get; private set; }

    internal bool LastContinuousCollisionIslandLimitReached { get; private set; }

    internal void AssimilateBody(StiffBody2D body, bool isDynamic)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(body.Context, _context),
            nameof(body),
            "2D body must belong to this physics service context.");

        if (isDynamic)
        {
            body.DynamicId = _dynamicBodies.Add(body);
            BodyCount++;
        }

        AssimilateCollider(body.Collider);
    }

    internal void AssimilateCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this physics service context.");

        collider.SetPhysicsState(_nextColliderId++, _colliders.Count);
        _colliders.Add(collider);
        _collidersById.Add(collider.Id, collider);
        _context.Collisions2D.PartitionCollider(collider);
    }

    internal void DessimilateBody(StiffBody2D body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        if (body.DynamicId >= 0 && _dynamicBodies.TryRemoveAt(body.DynamicId) && BodyCount > 0)
            BodyCount--;

        LSCollider2D collider = body.Collider;
        DessimilateCollider(collider);
    }

    internal void DessimilateCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        RemovePairsForCollider(collider);
        _context.MixedCollisions.RemovePairsFor2DCollider(collider);
        _context.MixedCollisions.ClearPartitioned2DCollider(collider, force: true);
        _context.Collisions2D.ClearPartitionedCollider(collider, force: true);
        RemoveCollider(collider);
        _collidersById.Remove(collider.Id);
        collider.ClearPhysicsState();
    }

    public void Simulate()
    {
        if (!SimulatePhysics)
            return;

        LastBroadPhaseCandidateCount = 0;
    }

    public void LateSimulate() => LateSimulate(continuousCollisionFramePrepared: false);

    internal void LateSimulate(bool continuousCollisionFramePrepared)
    {
        if (!SimulatePhysics)
            return;

        if (!continuousCollisionFramePrepared)
            _context.AdvanceLateSimulateToken();

        PrepareContinuousCollisionFrame();
        BeginContinuousCollisionHandoffFrame();

        foreach (StiffBody2D body in _dynamicBodies)
        {
            body.LateSimulate(updateSleepState: false, updateColliderState: false);
            _processedContinuousCollisionBodyIds.Add(body.DynamicId);
        }

        ProcessQueuedContinuousCollisionHandoffs();
        PrepareCollisionPartitions();
        RunDiscreteCollisionStep();
        UpdateSleepStatesAfterPhysicsStep();
    }

    private void PrepareCollisionPartitions()
    {
        foreach (StiffBody2D body in _dynamicBodies)
            body.Collider.Simulate();
    }

    private void RunDiscreteCollisionStep()
    {
        LastBroadPhaseCandidateCount = 0;
        EnsureFrameCapacity();
        _processedPairKeys.Clear();
        _discreteResponsePairs.FastClear();
        int frame = _context.FrameCount;
        _context.Collisions2D.CheckAndDistributeCollisions();
        ExpandDiscreteResponsePairs(frame);
        SolveDiscreteResponsePairs();
        CleanupUntouchedPairs(frame);
    }

    private void UpdateSleepStatesAfterPhysicsStep()
    {
        foreach (StiffBody2D body in _dynamicBodies)
            body.UpdateSleepStateAfterPhysicsStep();
    }

    internal void PrepareContinuousCollisionFrame()
    {
        int token = _context.LateSimulateToken;
        bool buildMixedIndex = _context.Settings.RuntimeMode.RunsMixedContacts();
        if (_continuousCollisionPreparedToken == token
            && _continuousCollisionPreparedMixedIndex == buildMixedIndex)
        {
            return;
        }

        _planarContinuousCollisionCandidates.Clear();
        _mixedContinuousCollisionCandidates.Clear();
        foreach (StiffBody2D body in _dynamicBodies)
        {
            body.EnsureContinuousCollisionFramePrepared(token);
            AddContinuousCollisionCandidate(body, buildMixedIndex);
        }

        _planarContinuousCollisionCandidates.Sort();
        if (buildMixedIndex)
            _mixedContinuousCollisionCandidates.Sort();

        _continuousCollisionPreparedToken = token;
        _continuousCollisionPreparedMixedIndex = buildMixedIndex;
    }

    private void AddContinuousCollisionCandidate(StiffBody2D body, bool buildMixedIndex)
    {
        if (!body.Active
            || body.Immovable
            || body.IsKinematic
            || body.Collider.IsTrigger)
        {
            return;
        }

        Fixed64 planarRadius = body.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
        if (planarRadius <= Fixed64.Epsilon)
            return;

        _planarContinuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                body.ContinuousCollisionFrameStart,
                body.ContinuousCollisionFrameDisplacement,
                planarRadius));

        if (!buildMixedIndex)
            return;

        Vector2d mixedStart2D = body.ContinuousCollisionFrameStart;
        Vector2d mixedDisplacement2D = body.ContinuousCollisionFrameDisplacement;
        Fixed64 mixedRadius = FixedMath.Max(planarRadius, body.Collider.MixedHalfThickness);
        _mixedContinuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d(mixedStart2D.X, body.Collider.MixedSlabCenterY, mixedStart2D.Y),
                new Vector3d(mixedDisplacement2D.X, Fixed64.Zero, mixedDisplacement2D.Y),
                mixedRadius));
    }

    public void Visualize()
    {
        foreach (StiffBody2D body in _dynamicBodies)
            body.OnVisualize();
    }

    public void Reset()
    {
        _dynamicBodies.Clear();
        _colliders.FastClear();
        _collidersById.Clear();
        _processedPairKeys.Clear();
        _pairs.Clear();
        _pairsToRemove.FastClear();
        _cachedPairs.Clear();
        _discreteResponsePairs.FastClear();
        _discreteResponseBodyKeys.Clear();
        _discreteResponseBodyQueue.FastClear();
        _discreteIslandNodes.FastClear();
        _discreteIslandConstraints.FastClear();
        _planarContinuousCollisionCandidates.Clear();
        _mixedContinuousCollisionCandidates.Clear();
        _continuousCollisionCandidateIds.FastClear();
        _processedContinuousCollisionBodyIds.Clear();
        _queuedContinuousCollisionHandoffIds.Clear();
        _continuousCollisionHandoffQueue.FastClear();
        _continuousCollisionPreparedToken = int.MinValue;
        _continuousCollisionPreparedMixedIndex = false;
        _nextColliderId = 1;
        BodyCount = 0;
        LastBroadPhaseCandidateCount = 0;
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;
    }

    internal bool TryGetColliderById(int colliderId, out LSCollider2D? collider)
    {
        return _collidersById.TryGetValue(colliderId, out collider);
    }

    internal bool TryGetColliderByServiceIndex(int serviceIndex, out LSCollider2D? collider)
    {
        if (serviceIndex < 0 || serviceIndex >= _colliders.Count)
        {
            collider = null;
            return false;
        }

        collider = _colliders[serviceIndex];
        return true;
    }

    internal int DynamicBodyPeakCount => _dynamicBodies.PeakCount;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetDynamicBody(int dynamicId, out StiffBody2D body) =>
        _dynamicBodies.TryGetValue(dynamicId, out body);

    internal SwiftList<int> QueryPlanarContinuousCollisionCandidates(DynamicCcdPlanarBounds sourceBounds)
    {
        PrepareContinuousCollisionFrame();
        _planarContinuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        return _continuousCollisionCandidateIds;
    }

    internal SwiftList<int> QueryMixedContinuousCollisionCandidates(FixedBoundVolume sourceBounds)
    {
        if (!_context.Settings.RuntimeMode.RunsMixedContacts())
        {
            _continuousCollisionCandidateIds.FastClear();
            return _continuousCollisionCandidateIds;
        }

        PrepareContinuousCollisionFrame();
        _mixedContinuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        return _continuousCollisionCandidateIds;
    }

    internal void QueueContinuousCollisionHandoff(StiffBody2D body)
    {
        int dynamicId = body.DynamicId;
        if (dynamicId < 0
            || !_processedContinuousCollisionBodyIds.Contains(dynamicId)
            || !_queuedContinuousCollisionHandoffIds.Add(dynamicId))
        {
            return;
        }

        _continuousCollisionHandoffQueue.Add(dynamicId);
    }

    internal bool ProcessQueuedContinuousCollisionHandoffs() =>
        ProcessQueuedContinuousCollisionHandoffs(_context.Settings.ContinuousCollisionMaxToiIterations) > 0;

    internal int ProcessQueuedContinuousCollisionHandoffs(int iterationBudget)
    {
        if (_continuousCollisionHandoffQueue.Count == 0)
            return 0;

        if (iterationBudget <= 0)
        {
            LastContinuousCollisionIslandLimitReached = true;
            ClearContinuousCollisionHandoffQueue();
            return 0;
        }

        int readIndex = 0;
        int iterations = 0;
        bool processed = false;
        while (readIndex < _continuousCollisionHandoffQueue.Count && iterations < iterationBudget)
        {
            int dynamicId = _continuousCollisionHandoffQueue[readIndex++];
            if (!TryGetDynamicBody(dynamicId, out StiffBody2D body))
                continue;

            if (body.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false))
            {
                processed = true;
                iterations++;
            }
        }

        if (processed)
            LastContinuousCollisionIslandCount++;

        LastContinuousCollisionIslandIterationCount += iterations;
        LastContinuousCollisionIslandLimitReached |= readIndex < _continuousCollisionHandoffQueue.Count;
        ClearContinuousCollisionHandoffQueue();
        return iterations;
    }

    private void BeginContinuousCollisionHandoffFrame()
    {
        _processedContinuousCollisionBodyIds.Clear();
        ClearContinuousCollisionHandoffQueue();
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;
    }

    private void ClearContinuousCollisionHandoffQueue()
    {
        _queuedContinuousCollisionHandoffIds.Clear();
        _continuousCollisionHandoffQueue.FastClear();
    }

    internal void ProcessPartitionCandidate(int firstId, int secondId, WorldVoxelIndex partitionIndex)
    {
        if (!_collidersById.TryGetValue(firstId, out LSCollider2D? first)
            || !_collidersById.TryGetValue(secondId, out LSCollider2D? second))
        {
            return;
        }

        if (!RequireCollisionPair(first!, second!)
            || !CollisionDetection2D.BoundsOverlap(first, second)
            || !IsCanonicalSharedPartition(first, second, partitionIndex))
        {
            return;
        }

        ulong key = CreatePairKey(firstId, secondId);
        if (!_processedPairKeys.Add(key))
            return;

        LastBroadPhaseCandidateCount++;
        ProcessCandidate(first, second, _context.FrameCount);
    }

    private static bool IsCanonicalSharedPartition(LSCollider2D first, LSCollider2D second, WorldVoxelIndex currentIndex)
    {
        SwiftList<WorldVoxelIndex>? firstCoordinates = first.PartitionCoordinates;
        SwiftList<WorldVoxelIndex>? secondCoordinates = second.PartitionCoordinates;
        if (firstCoordinates == null || secondCoordinates == null)
            return true;

        if (!TryGetMinimumVoxelIndexForGrid(firstCoordinates, currentIndex, out VoxelIndex firstMin)
            || !TryGetMinimumVoxelIndexForGrid(secondCoordinates, currentIndex, out VoxelIndex secondMin))
        {
            return true;
        }

        VoxelIndex current = currentIndex.VoxelIndex;
        return current.x == Max(firstMin.x, secondMin.x)
            && current.z == Max(firstMin.z, secondMin.z)
            && current.y == Max(firstMin.y, secondMin.y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Max(int first, int second) => first >= second ? first : second;

    private static bool TryGetMinimumVoxelIndexForGrid(
        SwiftList<WorldVoxelIndex> coordinates,
        WorldVoxelIndex gridIdentity,
        out VoxelIndex minimum)
    {
        minimum = default;
        bool found = false;
        for (int i = 0; i < coordinates.Count; i++)
        {
            WorldVoxelIndex candidate = coordinates[i];
            if (candidate.GridIndex != gridIdentity.GridIndex || candidate.GridSpawnToken != gridIdentity.GridSpawnToken)
                continue;

            VoxelIndex voxel = candidate.VoxelIndex;
            if (!found)
            {
                minimum = voxel;
                found = true;
                continue;
            }

            if (voxel.x < minimum.x)
                minimum.x = voxel.x;
            if (voxel.z < minimum.z)
                minimum.z = voxel.z;
            if (voxel.y < minimum.y)
                minimum.y = voxel.y;
        }

        return found;
    }

    private void ProcessCandidate(LSCollider2D first, LSCollider2D second, int frame)
    {
        if (!RequireCollisionPair(first, second))
            return;

        ulong key = CreatePairKey(first.Id, second.Id);
        bool hasPair = _pairs.TryGetValue(key, out CollisionPair2D pair);
        bool hasAwakeMovableParticipant = HasAwakeMovableParticipant(first, second);
        bool triggerPair = first.IsTrigger || second.IsTrigger;
        if (!hasPair && !hasAwakeMovableParticipant && !triggerPair)
            return;

        bool createdPair = false;
        if (!hasPair)
        {
            pair = CreatePair(first, second);
            createdPair = true;
        }

        if (!CollisionDetection2D.TryCollide(pair!, pair!.Manifold, frame))
        {
            if (createdPair)
                RecyclePair(pair);
            return;
        }

        if (triggerPair)
        {
            if (createdPair)
                RegisterPair(key, pair);

            pair.MarkColliding(frame);
            return;
        }

        if (!hasAwakeMovableParticipant)
        {
            pair.MarkResting(frame);
            _discreteResponsePairs.Add(pair);
            return;
        }

        if (createdPair)
            RegisterPair(key, pair);

        pair.MarkCollidingDeferred(frame);
        _discreteResponsePairs.Add(pair);
    }

    private void RegisterPair(ulong key, CollisionPair2D pair)
    {
        _pairs.Add(key, pair);
        pair.ColliderA.TryAddCollisionPair(pair.Id2, pair);
        pair.ColliderB.TryAddCollisionPairHolder(pair.Id1);
    }

    private void ExpandDiscreteResponsePairs(int frame)
    {
        if (_discreteResponsePairs.Count == 0)
            return;

        _discreteResponseBodyKeys.Clear();
        _discreteResponseBodyQueue.FastClear();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            AddDiscreteResponseBody(pair.ColliderA.Body);
            AddDiscreteResponseBody(pair.ColliderB.Body);
        }

        for (int readIndex = 0; readIndex < _discreteResponseBodyQueue.Count; readIndex++)
        {
            int dynamicId = _discreteResponseBodyQueue[readIndex];
            if (!TryGetDynamicBody(dynamicId, out StiffBody2D body))
                continue;

            AddExistingResponsePairs(body.Collider, frame);
        }
    }

    private void AddExistingResponsePairs(LSCollider2D collider, int frame)
    {
        SwiftDictionary<int, CollisionPair2D>? ownedPairs = collider.CollisionPairs;
        if (ownedPairs != null)
        {
            foreach (var pairEntry in ownedPairs)
                TryAddExistingResponsePair(pairEntry.Value, frame);
        }

        SwiftHashSet<int>? pairHolders = collider.CollisionPairHolders;
        if (pairHolders == null)
            return;

        foreach (int holderId in pairHolders)
        {
            if (!_collidersById.TryGetValue(holderId, out LSCollider2D? holder)
                || holder!.TryGetCollisionPair(collider.Id, out CollisionPair2D? pair) != true
                || pair == null)
            {
                continue;
            }

            TryAddExistingResponsePair(pair, frame);
        }
    }

    private void TryAddExistingResponsePair(CollisionPair2D pair, int frame)
    {
        ulong key = CreatePairKey(pair.Id1, pair.Id2);
        if (!_processedPairKeys.Add(key))
            return;

        LSCollider2D first = pair.ColliderA;
        LSCollider2D second = pair.ColliderB;
        if (first.IsTrigger
            || second.IsTrigger
            || !RequireCollisionPair(first, second)
            || !CollisionDetection2D.BoundsOverlap(first, second)
            || !CollisionDetection2D.TryCollide(pair, pair.Manifold, frame))
        {
            return;
        }

        if (HasAwakeMovableParticipant(first, second))
            pair.MarkCollidingDeferred(frame);
        else
            pair.MarkResting(frame);

        _discreteResponsePairs.Add(pair);
        AddDiscreteResponseBody(first.Body);
        AddDiscreteResponseBody(second.Body);
    }

    private void AddDiscreteResponseBody(StiffBody2D? body)
    {
        if (!IsMovableIslandBody(body) || !_discreteResponseBodyKeys.Add(body!.DynamicId))
            return;

        _discreteResponseBodyQueue.Add(body.DynamicId);
    }

    internal bool RequireCollisionPair(LSCollider2D first, LSCollider2D second)
    {
        return first.IsActive && second.IsActive
            && first.Shape != ColliderType2D.None && second.Shape != ColliderType2D.None
            && (first.Body != null || second.Body != null)
            && !ReferenceEquals(first.AgentOrNull, second.AgentOrNull)
            && !IsLayerCollisionDisabled(first.Layer, second.Layer)
            && !first.IsSibling(second);
    }

    private void CleanupUntouchedPairs(int frame)
    {
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPair2D pair = pairEntry.Value;
            if (pair.LastFrame == frame)
                continue;

            if (TryKeepRestingPair(pair, frame))
                continue;

            pair.MarkSeparated();
            RemovePairReferences(pair);
            _pairsToRemove.Add(pairEntry.Key);
        }

        for (int i = 0; i < _pairsToRemove.Count; i++)
        {
            ulong key = _pairsToRemove[i];
            if (_pairs.TryGetValue(key, out CollisionPair2D pair))
                RecyclePair(pair);

            _pairs.Remove(key);
        }
    }

    private bool TryKeepRestingPair(CollisionPair2D pair, int frame)
    {
        LSCollider2D first = pair.ColliderA;
        LSCollider2D second = pair.ColliderB;
        if (!first.IsActive
            || !second.IsActive
            || first.IsTrigger
            || second.IsTrigger
            || HasAwakeMovableParticipant(first, second)
            || IsLayerCollisionDisabled(first.Layer, second.Layer)
            || !CollisionDetection2D.TryCollide(pair, pair.Manifold, frame))
        {
            return false;
        }

        pair.MarkResting(frame);
        return true;
    }

    private CollisionPair2D CreatePair(LSCollider2D first, LSCollider2D second)
    {
        if (_context.Settings.PoolingEnabled && _cachedPairs.Count > 0)
        {
            CollisionPair2D pair = _cachedPairs.Pop();
            pair.Initialize(first, second);
            return pair;
        }

        return new CollisionPair2D(first, second);
    }

    private void RecyclePair(CollisionPair2D pair)
    {
        if (_context.Settings.PoolingEnabled)
            _cachedPairs.Push(pair);
    }

    private void RemovePairsForCollider(LSCollider2D collider)
    {
        SwiftDictionary<int, CollisionPair2D>? collisionPairs = collider.CollisionPairs;
        if (collisionPairs != null)
        {
            foreach (var pairEntry in collisionPairs)
            {
                CollisionPair2D pair = pairEntry.Value;
                pair.MarkSeparated();
                pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
                _pairs.Remove(CreatePairKey(pair.Id1, pair.Id2));
                RecyclePair(pair);
            }
        }

        SwiftHashSet<int>? collisionPairHolders = collider.CollisionPairHolders;
        if (collisionPairHolders != null)
        {
            foreach (int holderId in collisionPairHolders)
            {
                if (!_collidersById.TryGetValue(holderId, out LSCollider2D? holder)
                    || !holder!.TryRemoveCollisionPair(collider.Id, out CollisionPair2D? pair)
                    || pair == null)
                {
                    continue;
                }

                pair.MarkSeparated();
                _pairs.Remove(CreatePairKey(pair.Id1, pair.Id2));
                RecyclePair(pair);
            }
        }

        collider.ClearCollisionPairState();
        collider.ClearRuntimeRelationships();
    }

    private void RemoveCollider(LSCollider2D collider)
    {
        int index = collider.ServiceIndex;
        if (index < 0 || index >= _colliders.Count || !ReferenceEquals(_colliders[index], collider))
            return;

        int lastIndex = _colliders.Count - 1;
        if (index != lastIndex)
        {
            LSCollider2D moved = _colliders[lastIndex];
            _colliders[index] = moved;
            moved.SetServiceIndex(index);
        }

        _colliders.RemoveAt(lastIndex);
    }

    private void SolveDiscreteResponsePairs()
    {
        if (_discreteResponsePairs.Count == 0)
            return;

        if (_discreteResponsePairs.Count == 1)
        {
            CollisionPair2D pair = _discreteResponsePairs[0];
            if (!HasAwakeResponseParticipant(pair))
                return;

            pair.WakeSleepingBodiesForCollision();
            CollisionResponse2D.Resolve(pair);
            return;
        }

        _discreteResponsePairs.SortInPlace(ResponsePairComparer);
        BuildDiscreteIslands();
        if (_discreteIslandConstraints.Count == 0)
            return;

        _discreteIslandConstraints.SortInPlace(IslandConstraintComparer);

        int start = 0;
        while (start < _discreteIslandConstraints.Count)
        {
            int rootKey = _discreteIslandConstraints[start].RootKey;
            int end = start + 1;
            while (end < _discreteIslandConstraints.Count
                && _discreteIslandConstraints[end].RootKey == rootKey)
            {
                end++;
            }

            if (WakeIslandBodies(rootKey))
                SolveDiscreteIslandRange(start, end);

            start = end;
        }
    }

    private void BuildDiscreteIslands()
    {
        _discreteIslandNodes.FastClear();
        _discreteIslandConstraints.FastClear();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            AddIslandNodeIfMovable(pair.ColliderA.Body);
            AddIslandNodeIfMovable(pair.ColliderB.Body);
        }

        SortAndDeduplicateIslandNodes();
        if (_discreteIslandNodes.Count == 0)
            return;

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body);
            int nodeB = FindIslandNode(pair.ColliderB.Body);
            if (nodeA >= 0 && nodeB >= 0)
                UnionIslandNodes(nodeA, nodeB);
        }

        CompressIslandRoots();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body);
            int nodeB = FindIslandNode(pair.ColliderB.Body);
            int rootKey = ResolveConstraintRootKey(nodeA, nodeB);
            if (rootKey < 0)
                continue;

            GetStablePairKey(pair, out int minColliderId, out int maxColliderId);
            _discreteIslandConstraints.Add(new DiscreteIslandConstraint2D(
                pair,
                rootKey,
                minColliderId,
                maxColliderId));
        }
    }

    private void AddIslandNodeIfMovable(StiffBody2D? body)
    {
        if (!IsMovableIslandBody(body))
            return;

        _discreteIslandNodes.Add(new DiscreteIslandNode2D(body!.DynamicId, body));
    }

    private void SortAndDeduplicateIslandNodes()
    {
        if (_discreteIslandNodes.Count == 0)
            return;

        if (_discreteIslandNodes.Count == 1)
        {
            DiscreteIslandNode2D singleNode = _discreteIslandNodes[0];
            singleNode.ParentIndex = 0;
            singleNode.RootKey = singleNode.BodyKey;
            _discreteIslandNodes[0] = singleNode;
            return;
        }

        _discreteIslandNodes.SortInPlace(IslandNodeComparer);

        int writeIndex = 0;
        int previousKey = -1;
        for (int readIndex = 0; readIndex < _discreteIslandNodes.Count; readIndex++)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[readIndex];
            if (node.BodyKey == previousKey)
                continue;

            node.ParentIndex = writeIndex;
            node.RootKey = node.BodyKey;
            _discreteIslandNodes[writeIndex++] = node;
            previousKey = node.BodyKey;
        }

        while (_discreteIslandNodes.Count > writeIndex)
            _discreteIslandNodes.RemoveAt(_discreteIslandNodes.Count - 1);
    }

    private int FindIslandNode(StiffBody2D? body)
    {
        if (!IsMovableIslandBody(body))
            return -1;

        int key = body!.DynamicId;
        int low = 0;
        int high = _discreteIslandNodes.Count - 1;
        while (low <= high)
        {
            int mid = low + ((high - low) >> 1);
            int midKey = _discreteIslandNodes[mid].BodyKey;
            if (midKey == key)
                return mid;

            if (midKey < key)
                low = mid + 1;
            else
                high = mid - 1;
        }

        return -1;
    }

    private void UnionIslandNodes(int nodeA, int nodeB)
    {
        int rootA = FindIslandRoot(nodeA);
        int rootB = FindIslandRoot(nodeB);
        if (rootA == rootB)
            return;

        int keyA = _discreteIslandNodes[rootA].BodyKey;
        int keyB = _discreteIslandNodes[rootB].BodyKey;
        int parent = keyA <= keyB ? rootA : rootB;
        int child = parent == rootA ? rootB : rootA;

        DiscreteIslandNode2D childNode = _discreteIslandNodes[child];
        childNode.ParentIndex = parent;
        childNode.RootKey = _discreteIslandNodes[parent].BodyKey;
        _discreteIslandNodes[child] = childNode;
    }

    private int FindIslandRoot(int index)
    {
        int root = index;
        while (_discreteIslandNodes[root].ParentIndex != root)
            root = _discreteIslandNodes[root].ParentIndex;

        while (index != root)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[index];
            int parent = node.ParentIndex;
            node.ParentIndex = root;
            node.RootKey = _discreteIslandNodes[root].BodyKey;
            _discreteIslandNodes[index] = node;
            index = parent;
        }

        return root;
    }

    private void CompressIslandRoots()
    {
        for (int i = 0; i < _discreteIslandNodes.Count; i++)
        {
            int root = FindIslandRoot(i);
            DiscreteIslandNode2D node = _discreteIslandNodes[i];
            node.RootKey = _discreteIslandNodes[root].BodyKey;
            _discreteIslandNodes[i] = node;
        }
    }

    private int ResolveConstraintRootKey(int nodeA, int nodeB)
    {
        if (nodeA >= 0)
            return _discreteIslandNodes[nodeA].RootKey;

        return nodeB >= 0 ? _discreteIslandNodes[nodeB].RootKey : -1;
    }

    private bool WakeIslandBodies(int rootKey)
    {
        bool hasAwakeBody = false;
        for (int i = 0; i < _discreteIslandNodes.Count; i++)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[i];
            if (node.RootKey == rootKey && node.Body.IsAwakeForCollision)
            {
                hasAwakeBody = true;
                break;
            }
        }

        if (!hasAwakeBody)
            return false;

        for (int i = 0; i < _discreteIslandNodes.Count; i++)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[i];
            if (node.RootKey == rootKey)
                node.Body.WakeFromCollision();
        }

        return true;
    }

    private void SolveDiscreteIslandRange(int start, int end)
    {
        if (end - start == 1)
        {
            CollisionResponse2D.Resolve(_discreteIslandConstraints[start].Pair);
            return;
        }

        int iterations = _context.Settings.DiscreteSolverIterations;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            bool applyCachedImpulse = iteration == 0;
            bool applyPositionCorrection = iteration == 0;
            for (int i = start; i < end; i++)
            {
                CollisionResponse2D.Resolve(
                    _discreteIslandConstraints[i].Pair,
                    applyCachedImpulse,
                    applyPositionCorrection);
            }
        }
    }

    private void EnsureFrameCapacity()
    {
        int colliderCount = _colliders.Count;
        if (colliderCount <= 0)
            return;

        int expectedPairKeyCapacity = colliderCount * 4;
        _processedPairKeys.EnsureCapacity(expectedPairKeyCapacity);
        _pairsToRemove.EnsureCapacity(colliderCount);
        _discreteResponsePairs.EnsureCapacity(expectedPairKeyCapacity);
        _discreteResponseBodyKeys.EnsureCapacity(colliderCount);
        _discreteResponseBodyQueue.EnsureCapacity(colliderCount);
        _discreteIslandNodes.EnsureCapacity(colliderCount);
        _discreteIslandConstraints.EnsureCapacity(expectedPairKeyCapacity);
        if (_context.Settings.PoolingEnabled)
            _cachedPairs.EnsureCapacity(colliderCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeMovableParticipant(LSCollider2D first, LSCollider2D second) =>
        IsAwakeMovable(first.Body) || IsAwakeMovable(second.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeResponseParticipant(CollisionPair2D pair) =>
        IsAwakeIslandBody(pair.ColliderA.Body) || IsAwakeIslandBody(pair.ColliderB.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody2D? body) =>
        body != null && body.CanTranslate && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableIslandBody(StiffBody2D? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeIslandBody(StiffBody2D? body) =>
        IsMovableIslandBody(body) && body!.IsAwakeForCollision;

    private bool IsLayerCollisionDisabled(PhysicsLayer layer1, PhysicsLayer layer2)
    {
        bool[,] matrix = _context.Settings.CollisionMatrix;
        int layerIndex1 = layer1.Index;
        int layerIndex2 = layer2.Index;
        if (layerIndex1 >= matrix.GetLength(0) || layerIndex2 >= matrix.GetLength(1))
            return false;

        return !matrix[layerIndex1, layerIndex2];
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RemovePairReferences(CollisionPair2D pair)
    {
        pair.ColliderA.TryRemoveCollisionPair(pair.Id2, out _);
        pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong CreatePairKey(int firstId, int secondId)
    {
        if (firstId > secondId)
            (firstId, secondId) = (secondId, firstId);

        return ((ulong)(uint)firstId << 32) | (uint)secondId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetStablePairKey(CollisionPair2D pair, out int minColliderId, out int maxColliderId)
    {
        int idA = pair.ColliderA.Id;
        int idB = pair.ColliderB.Id;
        if (idA <= idB)
        {
            minColliderId = idA;
            maxColliderId = idB;
            return;
        }

        minColliderId = idB;
        maxColliderId = idA;
    }

    private sealed class CollisionPair2DStableKeyComparer : IComparer<CollisionPair2D>
    {
        public int Compare(CollisionPair2D? left, CollisionPair2D? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            GetStablePairKey(left, out int leftMin, out int leftMax);
            GetStablePairKey(right, out int rightMin, out int rightMax);

            int compare = leftMin.CompareTo(rightMin);
            return compare != 0 ? compare : leftMax.CompareTo(rightMax);
        }
    }

    private sealed class DiscreteIslandNode2DComparer : IComparer<DiscreteIslandNode2D>
    {
        public int Compare(DiscreteIslandNode2D left, DiscreteIslandNode2D right) =>
            left.BodyKey.CompareTo(right.BodyKey);
    }

    private sealed class DiscreteIslandConstraint2DComparer : IComparer<DiscreteIslandConstraint2D>
    {
        public int Compare(DiscreteIslandConstraint2D left, DiscreteIslandConstraint2D right)
        {
            int compare = left.RootKey.CompareTo(right.RootKey);
            if (compare != 0)
                return compare;

            compare = left.MinColliderId.CompareTo(right.MinColliderId);
            return compare != 0 ? compare : left.MaxColliderId.CompareTo(right.MaxColliderId);
        }
    }

    private struct DiscreteIslandNode2D
    {
        public DiscreteIslandNode2D(int bodyKey, StiffBody2D body)
        {
            BodyKey = bodyKey;
            Body = body;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey;
        public StiffBody2D Body;
        public int ParentIndex;
        public int RootKey;
    }

    private readonly struct DiscreteIslandConstraint2D
    {
        public DiscreteIslandConstraint2D(
            CollisionPair2D pair,
            int rootKey,
            int minColliderId,
            int maxColliderId)
        {
            Pair = pair;
            RootKey = rootKey;
            MinColliderId = minColliderId;
            MaxColliderId = maxColliderId;
        }

        public CollisionPair2D Pair { get; }
        public int RootKey { get; }
        public int MinColliderId { get; }
        public int MaxColliderId { get; }
    }
}
