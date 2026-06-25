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
public sealed partial class GravitasPhysicsService
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

    private void UpdateSleepStatesAfterPhysicsStep()
    {
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
                body.UpdateSleepStateAfterPhysicsStep();
        }
    }

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

}
