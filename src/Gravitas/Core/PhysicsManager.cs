using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

public static class PhysicsManager
{
    #region Constants

    private const int DefaultColliderSize = 2048;
    private const int DefaultBodySize = DefaultColliderSize / 4;
    private const int DefaultColliderIdSize = DefaultColliderSize / 8;

    public const int ConvexMeshMaxTriangleCount = 255;

    #endregion

    #region Physics Settings

    // Air density (rho) at sea level (in kg/m^3)
    // This is a constant you can adjust for different altitudes or environments.
    public static Fixed64 FixedGravity { get; private set; } = (Fixed64)9.8f;

    public static Fixed64 AirDensity { get; private set; } = (Fixed64)1.225f;

    /// <summary>
    /// One pound is equal to 4.44822162 Newtons.
    /// To convert pounds to Newtons, multiply pound value by this
    /// To convert Newtons to pounds, divide Newton value by this
    /// </summary>
    public static readonly Fixed64 PoundToNewton = (Fixed64)4.44822162f;

    /// <summary>
    /// One kilogram is equal to 2.20462262 pounds.
    /// To convert kilograms to pounds, multiply kg value by this
    /// To convert pounds to kilograms, divide pound value by this
    /// </summary>
    public static readonly Fixed64 KilogramToPound = (Fixed64)2.20462262f;

    // Maximum speed should be comfortable for the players.
    // It should allow them to navigate the game world efficiently without feeling too slow or uncontrollable.
    // Note: A higher maximum speed may lead to more issues with physics simulations and collision detection
    public static readonly Fixed64 MinSpeed = (Fixed64)0.00001f;

    public static readonly Fixed64 MaxSpeed = (Fixed64)7f;

    public static readonly Fixed64 MaxFallSpeed = (Fixed64)9.8f;

    public static readonly Fixed64 FrictionTransitionSpeed = (Fixed64)0.2f;

    public static readonly Fixed64 DecelerationMultiplier = (Fixed64)10f;

    public static readonly Fixed64 DampingFactor = (Fixed64)0.95f;

    #endregion

    #region Simulation Settings

    private static SwiftBucket<StiffBody> _dynamicSimBodies = new(DefaultBodySize);
    private static LSCollider?[] _simColliders = new LSCollider?[DefaultColliderIdSize];

    public static bool SimulatePhysics = true;

    private static bool _settingsChanged = true;
    private static PhysicsSettings _defaultSettings = PhysicsSettings.DefaultSettings();
    private static PhysicsSettings? _settings;

    /// <summary>
    /// GridSettings for the GridManager's simulation. Make sure you set this property ONLY if you wish to change the settings.
    /// Changes will apply to the next session.
    /// </summary>
    public static PhysicsSettings Settings
    {
        get => _settings ?? _defaultSettings;
        set { _settings = value; _settingsChanged = true; }
    }

    public static int FrameRate => Settings.FrameRate;

    /// <summary>
    /// Number of frames that have passed. FrameCount/FrameRate = duration of game session in seconds.
    /// </summary>
    /// <value>The frame count.</value>
    public static int FrameCount { get; private set; }

    #endregion

    #region Culling

    /// <summary>
    /// The maximum distance-based culling score. Higher values allow more frames
    /// to pass between collision checks for distant objects.
    /// </summary>
    internal static int CullDistanceMax => FrameRate / 3;
    /// <summary>
    /// The maximum distance for fast culling. Objects closer than this distance
    /// will be checked more frequently.
    /// </summary>
    internal static readonly Fixed64 CullFastDistanceMax = Fixed64.One * 4 * (Fixed64.One * 4);
    /// <summary>
    /// The step value for velocity-based culling. The score is increased
    /// when the relative velocity between objects increases. Higher values make the culling more aggressive
    /// for objects with higher relative velocities.
    /// </summary>
    internal static readonly int CullVelocityStep = 2;
    /// <summary>
    /// The maximum velocity-based culling score. Higher values allow more frames
    /// to pass between collision checks for objects with higher relative velocities.
    /// </summary>
    internal static readonly int CullVelocityMax = 4;
    /// <summary>
    /// The step value for time-based culling. The score is increased
    /// when more frames have passed since the last collision. Higher values make the culling more aggressive
    /// for objects that haven't collided recently.
    /// </summary>
    internal static int CullTimeStep => FrameRate * 3;
    /// <summary>
    /// The maximum time-based culling score. Higher values allow more frames
    /// to pass between collision checks for objects that haven't collided recently.
    /// </summary>
    internal static int CullTimeMax => FrameRate / 5;

    #endregion

    #region Assignment Variables

    private static SwiftStack<CollisionPair> _cachedCollisionPairs = new();
    private static SwiftQueue<CollisionPair> _activeCollisionPairs = new();

    public static int PeakColliderCount = 0;
    private static SwiftStack<int> _cachedColliderIds = new(DefaultColliderIdSize);
    public static int AssimalatedBodyCount = 0;
    public static int AssimalatedColliderCount = 0;

    public static bool ResetAccumulation { get; private set; }

    public static bool Simulated { get; private set; }

    public static Fixed64 AccumulatedTime { get; private set; }

    public static Fixed64 ExpectedAccumulation { get; private set; }

    public static Fixed64 ElapsedTime;

    /// <summary>
    /// The unscaled time in seconds between the last frame and the current frame.
    /// </summary>
    public static Fixed64 DeltaTime => Fixed64.One / FrameRate;

    private static int InactiveFrameThreshold => FrameRate * 8;

    #endregion

    public static void Setup()
    {
        _defaultSettings = PhysicsSettings.DefaultSettings();
    }

    public static void Initialize()
    {
        if (_settingsChanged)
            _settings ??= _defaultSettings;

        ResetVars();
    }

    public static void LateInitialize()
    {
        int simCount = _simColliders?.Length ?? 0;
        if (_simColliders == null || simCount == 0)
            return;

        for (int i = 0; i < simCount; i++)
            _simColliders[i]?.LateInitialize();
    }

    public static void Simulate()
    {
        FrameCount++;

        if (!SimulatePhysics) return;

        Simulated = true;
    }

    public static void LateSimulate()
    {
        if (!SimulatePhysics) return;

        //Clear the buffer of collision pairs to turn off and pool
        int collisionCounter = _activeCollisionPairs?.Count ?? 0;
        if (_activeCollisionPairs == null || collisionCounter == 0)
            return;

        while (collisionCounter > 0)
        {
            CollisionPair instancePair = _activeCollisionPairs.Dequeue();

            //check if it's inactive, if not get it out of inactives and move on to the next guy.
            if (instancePair == null || !instancePair.Active)
            {
                collisionCounter--;
                continue;
            }

            int passedFrames = FrameCount - instancePair.LastCollidedFrame;
            if (passedFrames >= InactiveFrameThreshold)
                FullDeactivateCollisionPair(instancePair);
            else
            {
                if (instancePair.CullCounter <= 0) instancePair.NotifyCollidersOfContact();
                _activeCollisionPairs.Enqueue(instancePair);  // pair still active, requeue
            }


            collisionCounter--;
        }

        if (_dynamicSimBodies == null)
        {
            ResetAccumulation = true;
            return;
        }


        lock (_dynamicSimBodies)
        {
            for (int i = 0; i < _dynamicSimBodies.PeakCount; i++)
            {
                StiffBody body = _dynamicSimBodies[i];
                // TODO: physics already performed, now we calculate visual position;
                // after this, should we distribute collisions for specific bodies?
                // i.e. if body moved and is in continuous detection mode?
                // would need to get the grid node to retrieve attached partition node and mark it to check
                body?.LateSimulate();
            }
        }

        ResetAccumulation = true;
    }

    public static void Visualize()
    {
        if (ResetAccumulation)
            AccumulatedTime = Fixed64.Zero;

        // aka duration
        AccumulatedTime += DeltaTime;

        ExpectedAccumulation = AccumulatedTime / DeltaTime;
        if (_dynamicSimBodies == null)
        {
            ResetAccumulation = false;
            return;
        }

        lock (_dynamicSimBodies)
        {
            for (int i = 0; i < _dynamicSimBodies.PeakCount; i++)
            {
                if (_dynamicSimBodies.IsAllocated(i))
                    _dynamicSimBodies[i].OnVisualize();
            }
        }

        ResetAccumulation = false;
    }

    public static void LateVisualize()
    {
        if (_dynamicSimBodies == null)
            return;

        lock (_dynamicSimBodies)
        {
            for (int i = 0; i < _dynamicSimBodies.PeakCount; i++)
            {
                if (_dynamicSimBodies.IsAllocated(i))
                    _dynamicSimBodies[i].LateVisualize();
            }
        }
    }

    public static void Deactivate() { }

    private static void ResetVars()
    {
        if (_simColliders != null)
        {
            for (int i = 0; i < PeakColliderCount - 1; i++)
                _simColliders[i] = null;
        }


        _dynamicSimBodies?.Clear();
        _cachedColliderIds?.FastClear();

        PeakColliderCount = 0;
        AssimalatedBodyCount = 0;
        AssimalatedColliderCount = 0;

        _activeCollisionPairs?.FastClear();
    }

    internal static int AssimilateBody(StiffBody body, bool isDynamic)
    {
        // Important: If isDynamic is false, PhysicsManager won't check to update the item every frame.
        // When the object is changed, it must be updated manually.
        int dynamicId = -1;
        if (isDynamic)
        {
            dynamicId = _dynamicSimBodies.Add(body);
            AssimalatedBodyCount++;
        }

        return dynamicId;
    }

    internal static int AssimilateCollider(LSCollider collider)
    {
        int id = -1;
        lock (_cachedColliderIds)
        {
            if (_cachedColliderIds.Count > 0)
                id = _cachedColliderIds.Pop();
            else
            {
                PeakColliderCount++;
                id = PeakColliderCount;
                if (PeakColliderCount == _simColliders.Length)
                    //very very expensive
                    Array.Resize(ref _simColliders, _simColliders.Length * 2);
            }
        }

        if (id == -1)
        {
            GravitasLogger.Channel.Error($"Failed to assimilate collider: no available IDs");
            return -1;
        }

        _simColliders[id] = collider;

        AssimalatedColliderCount++;
        return id;
    }

    internal static void DessimilateBody(StiffBody body)
    {
        if (body.DynamicId < 0) return;

        _dynamicSimBodies.TryRemoveAt(body.DynamicId);
        AssimalatedBodyCount--;
    }

    internal static void DessimilateCollider(LSCollider collider)
    {
        int tid = collider.Id;

        if (_simColliders[tid] == null)
        {
            GravitasLogger.Channel.Warn($"Object with ID {collider.Id} cannot be dessimilated because it is not assimilated");
            return;
        }

        _simColliders[tid] = null;
        _cachedColliderIds.Push(tid);
        AssimalatedColliderCount--;
    }

    public static CollisionPair? GetCollisionPair(int Id1, int Id2)
    {
        if (Id1 >= _simColliders.Length || Id2 >= _simColliders.Length)
        {
            GravitasLogger.Channel.Error($"Attempted to get collision pair with invalid IDs: {Id1}, {Id2}");
            return null;
        }

        if (!TryGetColliderById(Id1, out LSCollider? collider1) || !TryGetColliderById(Id2, out LSCollider? collider2))
            return null;

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetColliderById(int id, out LSCollider? collider)
    {
        if (id < 0 || id >= _simColliders.Length)
        {
            collider = null;
            return false;
        }
        collider = _simColliders[id];
        bool result = collider != null;
        if (!result)
            GravitasLogger.Channel.Error($"Collider with ID {id} does not exist");
        return result;
    }

    public static bool RequireCollisionPair(LSCollider collider1, LSCollider collider2)
    {
        return collider1.IsActive && collider2.IsActive
            && collider1.Shape != ColliderType.None && collider2.Shape != ColliderType.None
            && (collider1.Body != null || collider2.Body != null)
            && !GetIgnoreLayerCollision(collider1.Layer, collider2.Layer)
            && !collider1.IsSibling(collider2);
    }

    /// <summary>
    /// Determines whether the collision between two specified layers should be ignored.
    /// Returns true if the collision between the layers is ignored (they won't collide), false otherwise.
    /// Layers must be within the range 0-31 (inclusive), otherwise an error will be logged.
    /// </summary>
    /// <param name="layer1">The first layer to check.</param>
    /// <param name="layer2">The second layer to check.</param>
    /// <returns>True if collision is ignored, false otherwise.</returns>
    public static bool GetIgnoreLayerCollision(int layer1, int layer2)
    {
        if (layer1 < 0 || layer1 > 31 || layer2 < 0 || layer2 > 31)
        {
            GravitasLogger.Channel.Error($"Layers must be between 0 and 31 inclusive.");
            return false;
        }

        return !Settings.CollisionMatrix[layer1, layer2];
    }

    private static CollisionPair CreatePair(LSCollider collider1, LSCollider collider2)
    {
        CollisionPair pair = _cachedCollisionPairs.Count > 0
            ? _cachedCollisionPairs.Pop()
            : new CollisionPair(collider1, collider2);
        pair.Initialize(collider1, collider2);
        return pair;
    }

    public static void PoolForDeactivation(CollisionPair pair)
    {
        lock (_activeCollisionPairs)
        {
            _activeCollisionPairs.Enqueue(pair);
        }
    }

    public static void FullDeactivateCollisionPair(CollisionPair pair)
    {
        if (!pair.Active) return;

        // If we fail to remove references, we still need to deactivate and
        // pool the pair to avoid memory leaks and other issues.
        if (!TryRemovePairReferences(pair))
            DeactivateAndPoolPair(pair);

    }

    public static void DeactivateAndPoolPair(CollisionPair pair)
    {
        if (!pair.Active) return;

        pair.Deactivate();
        if (Settings.PoolingEnabled)
            _cachedCollisionPairs.Push(pair);
    }

    public static bool TryRemovePairReferences(CollisionPair pair)
    {
        if (!pair.ColliderA.TryRemoveCollisionPair(pair.Id2))
            return false;
        if (!pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1))
            return false;
        return true;
    }
}
