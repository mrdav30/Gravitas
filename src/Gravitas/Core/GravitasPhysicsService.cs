using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using SwiftCollections;
using System;
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

    private readonly GravitasWorldContext _context;

    private SwiftBucket<StiffBody> _dynamicBodies = new(DefaultBodySize);
    private LSCollider?[] _colliders = new LSCollider?[DefaultColliderIdSize];
    private SwiftStack<int> _cachedColliderIds = new(DefaultColliderIdSize);
    private SwiftStack<CollisionPair> _cachedCollisionPairs = new();
    private SwiftQueue<CollisionPair> _activeCollisionPairs = new();

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

        PrepareCollisionPartitions();
        _context.Collisions.CheckAndDistributeCollisions();
    }

    private void PrepareCollisionPartitions()
    {
        foreach (StiffBody body in _dynamicBodies)
            body.Collider.Simulate();
    }

    /// <summary>
    /// Runs this context's late physics step.
    /// </summary>
    public void LateSimulate()
    {
        if (!SimulatePhysics)
            return;

        ProcessActiveCollisionPairs();

        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
                body.LateSimulate();
        }
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
    /// Runs this context's late visualization step for dynamic bodies.
    /// </summary>
    public void LateVisualize()
    {
        // TODO: we may not need this...
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out StiffBody body))
                body.LateVisualize();
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

        if (!collider1.TryGetCollisionPair(collider2.Id, out CollisionPair? pair))
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

        if (!TryRemovePairReferences(pair))
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

    private int InactiveFrameThreshold => _context.FrameRate * 8;
}
