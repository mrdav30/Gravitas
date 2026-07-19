//=======================================================================
// GravitasPhysics2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Context-owned pure 2D body, broad-phase, narrow-phase, response, and query service.
/// </summary>
public sealed partial class GravitasPhysics2DService
{
    private static readonly CollisionPair2DStableKeyComparer ResponsePairComparer = new();
    private static readonly IslandNodeKeyComparer<DiscreteIslandNode2D> IslandNodeComparer = new();
    private static readonly DiscreteIslandConstraint2DComparer IslandConstraintComparer = new();

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<SolidBody2D> _dynamicBodies = new();
    private readonly ColliderRegistry<LSCollider2D> _colliders = new();
    private readonly SwiftList<LSCollider2D> _serviceRefreshColliders = new();
    private readonly SwiftHashSet<ulong> _processedPairKeys = new();
    private readonly SwiftDictionary<ulong, CollisionPair2D> _pairs = new();
    private readonly SwiftList<ulong> _pairsToRemove = new();
    private readonly SwiftList<CollisionPair2D> _pairsPendingSeparation = new();
    private readonly SwiftStack<CollisionPair2D> _cachedPairs = new();
    private readonly SwiftList<CollisionPair2D> _discreteResponsePairs = new();
    private readonly SwiftList<CollisionPair2D> _existingResponsePairCandidates = new();
    private readonly SwiftHashSet<int> _discreteResponseBodyKeys = new();
    private readonly SwiftList<int> _discreteResponseBodyQueue = new();
    private readonly SwiftList<DiscreteIslandNode2D> _discreteIslandNodes = new();
    private readonly SwiftList<DiscreteIslandConstraint2D> _discreteIslandConstraints = new();
    private readonly DynamicCcdCandidateIndex2D _planarContinuousCollisionCandidates = new();
    private readonly DynamicCcdCandidateIndex2D _dirtyPlanarContinuousCollisionCandidates =
        new(supportsUpdates: true);
    private readonly SwiftList<SolidBody2D> _dirtyPlanarContinuousCollisionBodies = new();
    private readonly SwiftHashSet<SolidBody2D> _dirtyPlanarContinuousCollisionBodySet = new();
    private readonly DynamicCcdCandidateIndex _mixedContinuousCollisionCandidates = new();
    private readonly DynamicCcdCandidateIndex _dirtyMixedContinuousCollisionCandidates =
        new(supportsUpdates: true);
    private readonly SwiftList<ColliderLifetimeToken2D> _continuousCollisionCandidateLifetimes = new();
    private readonly SwiftList<int> _continuousCollisionCandidateIds = new();
    private readonly SwiftList<int> _dirtyContinuousCollisionCandidateIds = new();
    private readonly SwiftHashSet<SolidBody2D> _processedContinuousCollisionBodies = new();
    private readonly SwiftHashSet<SolidBody2D> _queuedContinuousCollisionHandoffBodies = new();
    private readonly SwiftList<SolidBody2D> _continuousCollisionHandoffQueue = new();
    private int _continuousCollisionPreparedToken = int.MinValue;
    private bool _continuousCollisionPreparedMixedIndex;

    public GravitasPhysics2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    public int BodyCount { get; private set; }

    public int ColliderCount => _colliders.Count;

    public bool SimulatePhysics { get; set; } = true;

    internal int LastBroadPhaseCandidateCount { get; private set; }

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

    internal void AssimilateBody(SolidBody2D body, bool isDynamic)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(body.Context, _context),
            nameof(body),
            "2D body must belong to this physics service context.");

        if (isDynamic)
        {
            body.DynamicId = _dynamicBodies.Add(body);
            while (_continuousCollisionCandidateLifetimes.Count <= body.DynamicId)
                _continuousCollisionCandidateLifetimes.Add(default);
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

        _colliders.Register(collider);
        RefreshColliderServiceRefreshRegistration(collider);
        _context.Collisions2D.PartitionCollider(collider);
    }

    internal void DessimilateBody(SolidBody2D body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        if (body.DynamicId >= 0 && _dynamicBodies.TryRemoveAt(body.DynamicId))
        {
            _continuousCollisionCandidateLifetimes[body.DynamicId] = default;
            ReleaseContinuousCollisionCandidateRefresh(body);
            BodyCount--;
        }

        LSCollider2D collider = body.Collider;
        DessimilateCollider(collider);
    }

    internal void DessimilateCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (collider.Body != null)
            _context.Constraints2D.RemoveJointsForBody(collider.Body);
        RemovePairsForCollider(collider);
        _context.MixedCollisions.RemovePairsFor2DCollider(collider);
        if (collider.IsMixedPartitioned)
            _context.MixedCollisions.ClearPartitioned2DCollider(collider, force: true);
        if (collider.IsPartitioned)
            _context.Collisions2D.ClearPartitionedCollider(collider, force: true);

        RemoveServiceRefreshCollider(collider);
        _colliders.Remove(collider);
    }

    public void Simulate()
    {
        if (!SimulatePhysics)
            return;

        LastBroadPhaseCandidateCount = 0;
    }

    public void LateSimulate()
    {
        if (!BeginLateSimulateBodies(continuousCollisionFramePrepared: false))
            return;

        ProcessQueuedContinuousCollisionHandoffs();
        CompleteLateSimulatePhysicsStep();
    }

    internal bool BeginLateSimulateBodies(bool continuousCollisionFramePrepared)
    {
        if (!SimulatePhysics)
            return false;

        if (!continuousCollisionFramePrepared)
            _context.AdvanceLateSimulateToken();

        PrepareContinuousCollisionFrame();
        BeginContinuousCollisionHandoffFrame();

        foreach (SolidBody2D body in _dynamicBodies)
        {
            body.LateSimulate(updateSleepState: false, updateColliderState: false);
            _processedContinuousCollisionBodies.Add(body);
        }

        return true;
    }

    internal void CompleteLateSimulatePhysicsStep()
    {
        PrepareCollisionPartitions();
        RunDiscreteCollisionStep();
        UpdateSleepStatesAfterPhysicsStep();
    }

    private void PrepareCollisionPartitions()
    {
        foreach (SolidBody2D body in _dynamicBodies)
            body.Collider.Simulate();

        for (int i = 0; i < _serviceRefreshColliders.Count; i++)
            _serviceRefreshColliders[i].Simulate();
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
        RefreshGroundingFromDiscreteResponse(frame);
        CleanupUntouchedPairs(frame);
        _context.Collisions2D.RetireExpiredRetainedPartitions();
    }

    private void UpdateSleepStatesAfterPhysicsStep()
    {
        foreach (SolidBody2D body in _dynamicBodies)
            body.UpdateSleepStateAfterPhysicsStep();
    }

    public void Visualize()
    {
        foreach (SolidBody2D body in _dynamicBodies)
            body.OnVisualize();
    }

    public void Reset()
    {
        DiscardContinuousCollisionHandoffQueue();
        _dynamicBodies.Clear();
        _colliders.Clear();
        _serviceRefreshColliders.FastClear();
        _processedPairKeys.Clear();
        _pairs.Clear();
        _pairsToRemove.FastClear();
        _pairsPendingSeparation.FastClear();
        _cachedPairs.Clear();
        _discreteResponsePairs.FastClear();
        _existingResponsePairCandidates.FastClear();
        _discreteResponseBodyKeys.Clear();
        _discreteResponseBodyQueue.FastClear();
        _discreteIslandNodes.FastClear();
        _discreteIslandConstraints.FastClear();
        _planarContinuousCollisionCandidates.Clear();
        _dirtyPlanarContinuousCollisionCandidates.Clear();
        _dirtyPlanarContinuousCollisionBodies.FastClear();
        _dirtyPlanarContinuousCollisionBodySet.Clear();
        _mixedContinuousCollisionCandidates.Clear();
        _dirtyMixedContinuousCollisionCandidates.Clear();
        _continuousCollisionCandidateLifetimes.Clear();
        _continuousCollisionCandidateIds.FastClear();
        _dirtyContinuousCollisionCandidateIds.FastClear();
        _processedContinuousCollisionBodies.Clear();
        _continuousCollisionPreparedToken = int.MinValue;
        _continuousCollisionPreparedMixedIndex = false;
        BodyCount = 0;
        LastBroadPhaseCandidateCount = 0;
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;
    }

    internal bool TryGetColliderById(int colliderId, out LSCollider2D? collider) =>
        _colliders.TryGetById(colliderId, out collider);

    internal bool TryGetColliderByServiceIndex(int serviceIndex, out LSCollider2D? collider) =>
        _colliders.TryGetByServiceIndex(serviceIndex, out collider);

    internal LSCollider2D GetColliderByServiceIndex(int serviceIndex) => _colliders[serviceIndex];

    internal SwiftList<LSCollider2D> PrepareReplayColliders() => _colliders.PrepareReplayColliders();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetDynamicBody(int dynamicId, out SolidBody2D body) =>
        _dynamicBodies.TryGetValue(dynamicId, out body);

    internal void RefreshColliderServiceRefreshRegistration(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));

        if (collider.RequiresServiceSideRefresh)
            AddServiceRefreshCollider(collider);
        else
            RemoveServiceRefreshCollider(collider);
    }

    private void AddServiceRefreshCollider(LSCollider2D collider)
    {
        collider.SetServiceRefreshIndex(_serviceRefreshColliders.Count);
        _serviceRefreshColliders.Add(collider);
    }

    private void RemoveServiceRefreshCollider(LSCollider2D collider)
    {
        int index = collider.ServiceRefreshIndex;
        if (index < 0 || !ReferenceEquals(_serviceRefreshColliders[index], collider))
        {
            collider.ClearServiceRefreshIndex();
            return;
        }

        int lastIndex = _serviceRefreshColliders.Count - 1;
        if (index != lastIndex)
        {
            LSCollider2D moved = _serviceRefreshColliders[lastIndex];
            _serviceRefreshColliders[index] = moved;
            moved.SetServiceRefreshIndex(index);
        }

        _serviceRefreshColliders.RemoveAt(lastIndex);
        collider.ClearServiceRefreshIndex();
    }

    private void EnsureFrameCapacity()
    {
        int colliderCount = _colliders.Count;
        if (colliderCount <= 0)
            return;

        int expectedPairKeyCapacity = System.Math.Max(colliderCount * 4, _pairs.Count);
        _processedPairKeys.EnsureCapacity(expectedPairKeyCapacity);
        _pairsToRemove.EnsureCapacity(expectedPairKeyCapacity);
        _pairsPendingSeparation.EnsureCapacity(expectedPairKeyCapacity);
        _discreteResponsePairs.EnsureCapacity(expectedPairKeyCapacity);
        _existingResponsePairCandidates.EnsureCapacity(expectedPairKeyCapacity);
        _discreteResponseBodyKeys.EnsureCapacity(colliderCount);
        _discreteResponseBodyQueue.EnsureCapacity(colliderCount);
        _discreteIslandNodes.EnsureCapacity(colliderCount);
        _discreteIslandConstraints.EnsureCapacity(expectedPairKeyCapacity);
        if (_context.Settings.PoolingEnabled)
            _cachedPairs.EnsureCapacity(colliderCount);
    }

}
