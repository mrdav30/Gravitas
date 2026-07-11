//=======================================================================
// GravitasPhysicsService.cs
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
/// Owns physics registration and collision-pair state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasPhysicsService
{
    private const int DefaultColliderSize = 2048;
    private const int DefaultBodySize = DefaultColliderSize / 4;
    private const int DefaultColliderIdSize = DefaultColliderSize / 8;
    private static readonly CollisionPairStableKeyComparer ResponsePairComparer = new();
    private static readonly IslandNodeKeyComparer<DiscreteIslandNode> IslandNodeComparer = new();
    private static readonly DiscreteIslandConstraintComparer IslandConstraintComparer = new();

    private readonly GravitasWorldContext _context;

    private SwiftBucket<SolidBody> _dynamicBodies = new(DefaultBodySize);
    private readonly ColliderRegistry<LSCollider> _colliders = new(DefaultColliderSize);
    private SwiftList<LSCollider> _serviceRefreshColliders = new(DefaultColliderIdSize);
    private SwiftStack<CollisionPair> _cachedCollisionPairs = new();
    private SwiftQueue<CollisionPair> _activeCollisionPairs = new();
    private readonly SwiftList<CollisionPair> _discreteResponsePairs = new();
    private readonly SwiftList<DiscreteIslandNode> _discreteIslandNodes = new();
    private readonly SwiftList<DiscreteIslandConstraint> _discreteIslandConstraints = new();
    private readonly DynamicCcdCandidateIndex _continuousCollisionCandidates = new(DefaultBodySize);
    private readonly SwiftList<int> _continuousCollisionCandidateIds = new(DefaultBodySize);
    private readonly SwiftHashSet<SolidBody> _processedContinuousCollisionBodies = new(DefaultBodySize);
    private readonly SwiftHashSet<SolidBody> _queuedContinuousCollisionHandoffBodies = new(DefaultBodySize);
    private readonly SwiftList<SolidBody> _continuousCollisionHandoffQueue = new(DefaultBodySize);
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
    public int PeakColliderCount => _colliders.PeakCount;

    /// <summary>
    /// Gets the number of dynamic bodies currently registered in this context.
    /// </summary>
    public int BodyCount { get; private set; }

    /// <summary>
    /// Gets the number of colliders currently registered in this context.
    /// </summary>
    public int ColliderCount => _colliders.Count;

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
    /// Runs this context's physics simulation step.
    /// </summary>
    public void Simulate()
    {
        if (!SimulatePhysics)
            return;
    }

    private void PrepareCollisionPartitions()
    {
        foreach (SolidBody body in _dynamicBodies)
            body.Collider.Simulate();

        for (int i = 0; i < _serviceRefreshColliders.Count; i++)
            _serviceRefreshColliders[i].Simulate();
    }

    /// <summary>
    /// Runs this context's late physics step.
    /// </summary>
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

        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out SolidBody body))
            {
                body.LateSimulate(updateSleepState: false, updateColliderState: false);
                _processedContinuousCollisionBodies.Add(body);
            }
        }

        return true;
    }

    internal void CompleteLateSimulatePhysicsStep()
    {
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
            if (_dynamicBodies.TryGetValue(i, out SolidBody body))
                body.UpdateSleepStateAfterPhysicsStep();
        }
    }

    public void Visualize()
    {
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out SolidBody body))
                body.OnVisualize();
        }
    }

    /// <summary>
    /// Clears all context-local physics registration state.
    /// </summary>
    public void Reset()
    {
        DiscardContinuousCollisionHandoffQueue();
        _dynamicBodies.Clear();
        _colliders.Clear();
        _serviceRefreshColliders.FastClear();
        _cachedCollisionPairs.FastClear();
        _activeCollisionPairs.FastClear();
        _discreteResponsePairs.FastClear();
        _discreteIslandNodes.FastClear();
        _discreteIslandConstraints.FastClear();
        _continuousCollisionCandidates.Clear();
        _continuousCollisionCandidateIds.FastClear();
        _processedContinuousCollisionBodies.Clear();
        _continuousCollisionPreparedToken = int.MinValue;
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;

        BodyCount = 0;
    }

    internal int AssimilateBody(SolidBody body, bool isDynamic)
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
            BodyCount++;
        }

        return dynamicId;
    }

    internal int AssimilateCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        collider.BindContext(_context);

        int id = _colliders.Register(collider);
        RefreshColliderServiceRefreshRegistration(collider);
        return id;
    }

    internal void DessimilateBody(SolidBody body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        int dynamicId = body.DynamicId;
        if (!_dynamicBodies.TryRemoveAt(dynamicId))
            return;

        BodyCount--;
    }

    internal void DessimilateCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        int id = collider.Id;

        if (!_colliders.TryGetById(id, out _))
        {
            GravitasLogger.Channel.Warn($"Object with ID {collider.Id} cannot be dessimilated because it is not assimilated.");
            return;
        }

        _context.Constraints3D.RemoveSuppressionsForCollider(id);
        RemovePairsForCollider(collider);
        if (collider.IsPartitioned)
            _context.Collisions.ClearPartitionedObject(collider, true);

        _context.MixedCollisions.RemovePairsFor3DCollider(collider);
        if (collider.IsMixedPartitioned)
            _context.MixedCollisions.ClearPartitioned3DCollider(collider, force: true);

        RemoveServiceRefreshCollider(collider);
        _colliders.Remove(collider);
    }

    internal void RefreshColliderServiceRefreshRegistration(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));

        if (collider.RequiresServiceSideRefresh)
            AddServiceRefreshCollider(collider);
        else
            RemoveServiceRefreshCollider(collider);
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
        if (id < 0)
        {
            collider = null;
            return false;
        }

        return _colliders.TryGetById(id, out collider);
    }

    internal bool TryGetColliderByServiceIndex(int serviceIndex, out LSCollider? collider) =>
        _colliders.TryGetByServiceIndex(serviceIndex, out collider);

    internal LSCollider GetColliderByServiceIndex(int serviceIndex) => _colliders[serviceIndex];

    internal SwiftList<LSCollider> PrepareReplayColliders() => _colliders.PrepareReplayColliders();

    internal bool TryGetDynamicBody(int dynamicId, out SolidBody body) =>
        _dynamicBodies.TryGetValue(dynamicId, out body);

    private void AddServiceRefreshCollider(LSCollider collider)
    {
        collider.SetServiceRefreshIndex(_serviceRefreshColliders.Count);
        _serviceRefreshColliders.Add(collider);
    }

    private void RemoveServiceRefreshCollider(LSCollider collider)
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
            LSCollider moved = _serviceRefreshColliders[lastIndex];
            _serviceRefreshColliders[index] = moved;
            moved.SetServiceRefreshIndex(index);
        }

        _serviceRefreshColliders.RemoveAt(lastIndex);
        collider.ClearServiceRefreshIndex();
    }

}
