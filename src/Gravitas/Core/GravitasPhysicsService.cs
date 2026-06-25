//=======================================================================
// GravitasPhysicsService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns physics registration and collision-pair state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasPhysicsService
{
    private const int DefaultColliderSize = 2048;
    private const int DefaultBodySize = DefaultColliderSize / 4;
    private const int DefaultColliderIdSize = DefaultColliderSize / 8;
    private static readonly CollisionPairStableKeyComparer ResponsePairComparer = new();
    private static readonly DiscreteIslandNodeComparer IslandNodeComparer = new();
    private static readonly DiscreteIslandConstraintComparer IslandConstraintComparer = new();

    private readonly GravitasWorldContext _context;

    private SwiftBucket<StiffBody> _dynamicBodies = new(DefaultBodySize);
    private LSCollider?[] _colliders = new LSCollider?[DefaultColliderIdSize];
    private SwiftStack<int> _cachedColliderIds = new(DefaultColliderIdSize);
    private SwiftStack<CollisionPair> _cachedCollisionPairs = new();
    private SwiftQueue<CollisionPair> _activeCollisionPairs = new();
    private readonly SwiftList<CollisionPair> _discreteResponsePairs = new();
    private readonly SwiftList<DiscreteIslandNode> _discreteIslandNodes = new();
    private readonly SwiftList<DiscreteIslandConstraint> _discreteIslandConstraints = new();
    private readonly DynamicCcdCandidateIndex _continuousCollisionCandidates = new(DefaultBodySize);
    private readonly SwiftList<int> _continuousCollisionCandidateIds = new(DefaultBodySize);
    private readonly SwiftHashSet<int> _processedContinuousCollisionBodyIds = new(DefaultBodySize);
    private readonly SwiftHashSet<int> _queuedContinuousCollisionHandoffIds = new(DefaultBodySize);
    private readonly SwiftList<int> _continuousCollisionHandoffQueue = new(DefaultBodySize);
    private int _continuousCollisionPreparedToken = int.MinValue;

    /// <summary>
    /// Initializes a new physics service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasPhysicsService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets or sets whether this context should simulate physics.
    /// </summary>
    public bool SimulatePhysics { get; set; } = true;

    /// <summary>
    /// Gets the highest collider ID allocated in this context.
    /// </summary>
    public int PeakColliderCount { get; private set; }

    /// <summary>
    /// Gets the number of dynamic bodies currently registered in this context.
    /// </summary>
    public int AssimilatedBodyCount { get; private set; }

    /// <summary>
    /// Gets the number of colliders currently registered in this context.
    /// </summary>
    public int AssimilatedColliderCount { get; private set; }

    /// <summary>
    /// Gets how many service-owned continuous-collision handoff batches were consumed during the last late step.
    /// </summary>
    public int LastContinuousCollisionIslandCount { get; private set; }

    /// <summary>
    /// Gets how many service-owned continuous-collision handoff iterations were consumed during the last late step.
    /// </summary>
    public int LastContinuousCollisionIslandIterationCount { get; private set; }

    /// <summary>
    /// Gets whether the service-owned continuous-collision handoff queue hit its deterministic iteration cap.
    /// </summary>
    public bool LastContinuousCollisionIslandLimitReached { get; private set; }

    /// <summary>
    /// Resets context-local physics registration state.
    /// </summary>
    public void Initialize() => Reset();

    /// <summary>
    /// Runs late initialization for registered colliders.
    /// </summary>
    public void LateInitialize()
    {
        for (int i = 0; i < _colliders.Length; i++)
            _colliders[i]?.LateInitialize();
    }

    /// <summary>
    /// Runs this context's physics simulation step.
    /// </summary>
    public void Simulate()
    {
        if (!SimulatePhysics)
            return;
    }

    private void PrepareCollisionPartitions()
    {
        foreach (StiffBody body in _dynamicBodies)
            body.Collider.Simulate();
    }

    /// <summary>
    /// Runs this context's late physics step.
    /// </summary>
    public void LateSimulate() => LateSimulate(continuousCollisionFramePrepared: false);

    internal void LateSimulate(bool continuousCollisionFramePrepared)
    {
        if (!SimulatePhysics)
            return;

        if (!continuousCollisionFramePrepared)
            _context.AdvanceLateSimulateToken();

        PrepareContinuousCollisionFrame();
        BeginContinuousCollisionHandoffFrame();

        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
            {
                body.LateSimulate(updateSleepState: false, updateColliderState: false);
                _processedContinuousCollisionBodyIds.Add(body.DynamicId);
            }
        }

        ProcessQueuedContinuousCollisionHandoffs();
        PrepareCollisionPartitions();
        RunDiscreteCollisionStep();
        ProcessActiveCollisionPairs();
        UpdateSleepStatesAfterPhysicsStep();
    }

    private void RunDiscreteCollisionStep()
    {
        _discreteResponsePairs.FastClear();
        _context.Collisions.CheckAndDistributeCollisions();
        SolveDiscreteResponsePairs();
        _context.Collisions.RetireExpiredRetainedPartitions();
    }

    internal void QueueDiscreteResponsePair(CollisionPair pair)
    {
        SwiftThrowHelper.ThrowIfNull(pair, nameof(pair));
        _discreteResponsePairs.Add(pair);
    }

    private void SolveDiscreteResponsePairs()
    {
        if (_discreteResponsePairs.Count == 0)
            return;

        if (_discreteResponsePairs.Count == 1)
        {
            CollisionPair pair = _discreteResponsePairs[0];
            if (!HasAwakeResponseParticipant(pair))
                return;

            pair.WakeSleepingBodiesForCollision();
            CollisionResponse.CalculateImpulse(pair);
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
            CollisionPair pair = _discreteResponsePairs[i];
            AddIslandNodeIfMovable(pair.ColliderA.Body);
            AddIslandNodeIfMovable(pair.ColliderB.Body);
        }

        SortAndDeduplicateIslandNodes();
        if (_discreteIslandNodes.Count == 0)
            return;

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body);
            int nodeB = FindIslandNode(pair.ColliderB.Body);
            if (nodeA >= 0 && nodeB >= 0)
                UnionIslandNodes(nodeA, nodeB);
        }

        CompressIslandRoots();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body);
            int nodeB = FindIslandNode(pair.ColliderB.Body);
            int rootKey = ResolveConstraintRootKey(nodeA, nodeB);
            if (rootKey < 0)
                continue;

            GetStablePairKey(pair, out int minColliderId, out int maxColliderId);
            _discreteIslandConstraints.Add(new DiscreteIslandConstraint(
                pair,
                rootKey,
                minColliderId,
                maxColliderId));
        }
    }

    private void AddIslandNodeIfMovable(StiffBody? body)
    {
        if (!IsMovableIslandBody(body))
            return;

        _discreteIslandNodes.Add(new DiscreteIslandNode(body!.DynamicId, body));
    }

    private void SortAndDeduplicateIslandNodes()
    {
        if (_discreteIslandNodes.Count == 0)
            return;

        if (_discreteIslandNodes.Count == 1)
        {
            DiscreteIslandNode singleNode = _discreteIslandNodes[0];
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
            DiscreteIslandNode node = _discreteIslandNodes[readIndex];
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

    private int FindIslandNode(StiffBody? body)
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

        DiscreteIslandNode childNode = _discreteIslandNodes[child];
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
            DiscreteIslandNode node = _discreteIslandNodes[index];
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
            DiscreteIslandNode node = _discreteIslandNodes[i];
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
            DiscreteIslandNode node = _discreteIslandNodes[i];
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
            DiscreteIslandNode node = _discreteIslandNodes[i];
            if (node.RootKey == rootKey)
                node.Body.WakeFromCollision();
        }

        return true;
    }

    private void SolveDiscreteIslandRange(int start, int end)
    {
        if (end - start == 1)
        {
            CollisionResponse.CalculateImpulse(_discreteIslandConstraints[start].Pair);
            return;
        }

        int iterations = _context.Settings.DiscreteSolverIterations;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            bool applyCachedImpulse = iteration == 0;
            bool applyPositionCorrection = iteration == 0;
            for (int i = start; i < end; i++)
            {
                CollisionResponse.CalculateImpulse(
                    _discreteIslandConstraints[i].Pair,
                    applyCachedImpulse,
                    applyPositionCorrection);
            }
        }
    }

    private void UpdateSleepStatesAfterPhysicsStep()
    {
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
                body.UpdateSleepStateAfterPhysicsStep();
        }
    }

    internal void PrepareContinuousCollisionFrame()
    {
        int token = _context.LateSimulateToken;
        if (_continuousCollisionPreparedToken == token)
            return;

        _continuousCollisionCandidates.Clear();
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
            {
                body.EnsureContinuousCollisionFramePrepared(token);
                AddContinuousCollisionCandidate(body);
            }
        }

        _continuousCollisionCandidates.Sort();
        _continuousCollisionPreparedToken = token;
    }

    private void AddContinuousCollisionCandidate(StiffBody body)
    {
        if (!body.Active
            || body.Immovable
            || body.IsKinematic
            || body.Collider.IsTrigger)
        {
            return;
        }

        Fixed64 radius = body.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
        if (radius <= Fixed64.Epsilon)
            return;

        _continuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                body.ContinuousCollisionFrameStart,
                body.ContinuousCollisionFrameDisplacement,
                radius));
    }

    /// <summary>
    /// Runs this context's visual interpolation step for dynamic bodies.
    /// </summary>
    public void Visualize()
    {
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
                body.OnVisualize();
        }
    }

    /// <summary>
    /// Clears all context-local physics registration state.
    /// </summary>
    public void Reset()
    {
        for (int i = 1; i <= PeakColliderCount && i < _colliders.Length; i++)
            _colliders[i] = null;

        _dynamicBodies.Clear();
        _cachedColliderIds.FastClear();
        _cachedCollisionPairs.FastClear();
        _activeCollisionPairs.FastClear();
        _discreteResponsePairs.FastClear();
        _discreteIslandNodes.FastClear();
        _discreteIslandConstraints.FastClear();
        _continuousCollisionCandidates.Clear();
        _continuousCollisionCandidateIds.FastClear();
        _processedContinuousCollisionBodyIds.Clear();
        _queuedContinuousCollisionHandoffIds.Clear();
        _continuousCollisionHandoffQueue.FastClear();
        _continuousCollisionPreparedToken = int.MinValue;
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;

        PeakColliderCount = 0;
        AssimilatedBodyCount = 0;
        AssimilatedColliderCount = 0;
    }

    /// <summary>
    /// Deactivates the service and clears pooled state.
    /// </summary>
    public void Deactivate() => Reset();

    internal int AssimilateBody(StiffBody body, bool isDynamic)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(body.Context, _context),
            nameof(body),
            "Body must belong to this physics service context.");

        int dynamicId = -1;
        if (isDynamic)
        {
            dynamicId = _dynamicBodies.Add(body);
            AssimilatedBodyCount++;
        }

        return dynamicId;
    }

    internal int AssimilateCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        collider.BindContext(_context);

        int id;
        lock (_cachedColliderIds)
        {
            if (_cachedColliderIds.Count > 0)
                id = _cachedColliderIds.Pop();
            else
            {
                PeakColliderCount++;
                id = PeakColliderCount;
                if (PeakColliderCount == _colliders.Length)
                    Array.Resize(ref _colliders, _colliders.Length * 2);
            }
        }

        collider.SetPhysicsId(id);
        _colliders[id] = collider;
        AssimilatedColliderCount++;
        return id;
    }

    internal void DessimilateBody(StiffBody body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        int dynamicId = body.DynamicId;
        if (!_dynamicBodies.TryGetValue(dynamicId, out _))
            return;

        _dynamicBodies.TryRemoveAt(dynamicId);
        if (AssimilatedBodyCount > 0)
            AssimilatedBodyCount--;
    }

    internal void DessimilateCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        int id = collider.Id;

        if ((uint)id >= (uint)_colliders.Length || _colliders[id] == null)
        {
            GravitasLogger.Channel.Warn($"Object with ID {collider.Id} cannot be dessimilated because it is not assimilated.");
            return;
        }

        _context.MixedCollisions.RemovePairsFor3DCollider(collider);
        _context.MixedCollisions.ClearPartitioned3DCollider(collider, force: true);
        _colliders[id] = null;
        _cachedColliderIds.Push(id);
        AssimilatedColliderCount--;
    }

    /// <summary>
    /// Resolves a context-local collider ID.
    /// </summary>
    /// <param name="id">The context-local collider ID.</param>
    /// <param name="collider">The resolved collider, if present.</param>
    /// <returns>True when the ID resolves in this context.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetColliderById(int id, out LSCollider? collider)
    {
        if (id < 0 || id >= _colliders.Length)
        {
            collider = null;
            return false;
        }

        collider = _colliders[id];
        return collider != null;
    }

    internal int DynamicBodyPeakCount => _dynamicBodies.PeakCount;

    internal bool TryGetDynamicBody(int dynamicId, out StiffBody body) =>
        _dynamicBodies.TryGetValue(dynamicId, out body);

    internal SwiftList<int> QueryContinuousCollisionCandidates(FixedBoundVolume sourceBounds)
    {
        PrepareContinuousCollisionFrame();
        _continuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        return _continuousCollisionCandidateIds;
    }

    internal void QueueContinuousCollisionHandoff(StiffBody body)
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
            if (!TryGetDynamicBody(dynamicId, out StiffBody body))
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

    internal CollisionPair? GetCollisionPair(int id1, int id2)
    {
        if (!TryGetColliderById(id1, out LSCollider? collider1)
            || !TryGetColliderById(id2, out LSCollider? collider2))
        {
            return null;
        }

        if (collider1!.Id > collider2!.Id)
            (collider2, collider1) = (collider1, collider2);

        if (!RequireCollisionPair(collider1, collider2))
            return null;

        if (!collider1.TryGetCollisionPair(collider2.Id, out CollisionPair? pair)
            && !collider2.TryGetCollisionPair(collider1.Id, out pair))
        {
            pair = CreatePair(collider1, collider2);
            pair.ColliderA.TryAddCollisionPair(pair.ColliderB.Id, pair);
            pair.ColliderB.TryAddCollisionPairHolder(pair.ColliderA.Id);
        }

        return pair;
    }

    internal bool RequireCollisionPair(LSCollider collider1, LSCollider collider2)
    {
        return collider1.IsActive && collider2.IsActive
            && collider1.Shape != ColliderType.None && collider2.Shape != ColliderType.None
            && (collider1.Body != null || collider2.Body != null)
            && !IsLayerCollisionDisabled(collider1.Layer, collider2.Layer)
            && !collider1.IsSibling(collider2);
    }

    internal bool IsLayerCollisionDisabled(PhysicsLayer layer1, PhysicsLayer layer2)
    {
        bool[,] matrix = _context.Settings.CollisionMatrix;
        int layerIndex1 = layer1.Index;
        int layerIndex2 = layer2.Index;
        if (layerIndex1 >= matrix.GetLength(0) || layerIndex2 >= matrix.GetLength(1))
            return false;

        return !matrix[layerIndex1, layerIndex2];
    }

    internal void PoolForDeactivation(CollisionPair pair)
    {
        SwiftThrowHelper.ThrowIfNull(pair, nameof(pair));
        lock (_activeCollisionPairs)
            _activeCollisionPairs.Enqueue(pair);
    }

    internal void FullDeactivateCollisionPair(CollisionPair pair)
    {
        if (!pair.Active)
            return;

        TryRemovePairReferences(pair);
        DeactivateAndPoolPair(pair);
    }

    internal void DeactivateAndPoolPair(CollisionPair pair)
    {
        if (!pair.Active)
            return;

        pair.Deactivate();
        if (_context.Settings.PoolingEnabled)
            _cachedCollisionPairs.Push(pair);
    }

    internal bool TryRemovePairReferences(CollisionPair pair)
    {
        return pair.ColliderA.TryRemoveCollisionPair(pair.Id2)
            && pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
    }

    private CollisionPair CreatePair(LSCollider collider1, LSCollider collider2)
    {
        if (_cachedCollisionPairs.Count <= 0)
            return new CollisionPair(collider1, collider2);

        CollisionPair pair = _cachedCollisionPairs.Pop();
        pair.Initialize(collider1, collider2);
        return pair;
    }

    private void ProcessActiveCollisionPairs()
    {
        int collisionCounter = _activeCollisionPairs.Count;
        while (collisionCounter > 0)
        {
            CollisionPair instancePair = _activeCollisionPairs.Dequeue();
            if (instancePair == null || !instancePair.Active)
            {
                collisionCounter--;
                continue;
            }

            bool preservedSleepingContact = instancePair.TryPreserveSleepingRestingContact();
            int passedFrames = _context.FrameCount - instancePair.LastCollidedFrame;
            if (!preservedSleepingContact && passedFrames >= InactiveFrameThreshold)
                FullDeactivateCollisionPair(instancePair);
            else
            {
                if (instancePair.CullCounter <= 0)
                    instancePair.NotifyCollidersOfContact();
                _activeCollisionPairs.Enqueue(instancePair);
            }

            collisionCounter--;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableIslandBody(StiffBody? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeResponseParticipant(CollisionPair pair) =>
        IsAwakeIslandBody(pair.ColliderA.Body) || IsAwakeIslandBody(pair.ColliderB.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeIslandBody(StiffBody? body) =>
        IsMovableIslandBody(body) && body!.IsAwakeForCollision;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetStablePairKey(CollisionPair pair, out int minColliderId, out int maxColliderId)
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

    private sealed class CollisionPairStableKeyComparer : IComparer<CollisionPair>
    {
        public int Compare(CollisionPair? left, CollisionPair? right)
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

    private sealed class DiscreteIslandNodeComparer : IComparer<DiscreteIslandNode>
    {
        public int Compare(DiscreteIslandNode left, DiscreteIslandNode right) =>
            left.BodyKey.CompareTo(right.BodyKey);
    }

    private sealed class DiscreteIslandConstraintComparer : IComparer<DiscreteIslandConstraint>
    {
        public int Compare(DiscreteIslandConstraint left, DiscreteIslandConstraint right)
        {
            int compare = left.RootKey.CompareTo(right.RootKey);
            if (compare != 0)
                return compare;

            compare = left.MinColliderId.CompareTo(right.MinColliderId);
            return compare != 0 ? compare : left.MaxColliderId.CompareTo(right.MaxColliderId);
        }
    }

    private struct DiscreteIslandNode
    {
        public DiscreteIslandNode(int bodyKey, StiffBody body)
        {
            BodyKey = bodyKey;
            Body = body;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey;
        public StiffBody Body;
        public int ParentIndex;
        public int RootKey;
    }

    private readonly struct DiscreteIslandConstraint
    {
        public DiscreteIslandConstraint(
            CollisionPair pair,
            int rootKey,
            int minColliderId,
            int maxColliderId)
        {
            Pair = pair;
            RootKey = rootKey;
            MinColliderId = minColliderId;
            MaxColliderId = maxColliderId;
        }

        public CollisionPair Pair { get; }
        public int RootKey { get; }
        public int MinColliderId { get; }
        public int MaxColliderId { get; }
    }

    private int InactiveFrameThreshold => _context.FrameRate * 8;
}
