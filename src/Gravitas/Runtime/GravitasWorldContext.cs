using FixedMathSharp;
using Gravitas.Raycasting;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>
/// Owns Gravitas runtime state for one explicit <see cref="GridWorld"/>.
/// </summary>
/// <remarks>
/// This is the context-first host API for multi-world Gravitas usage. Phase 1 owns
/// world lifetime, deterministic clock state, and lifecycle hooks; later phases move
/// physics registries, collision partitioning, query buffers, and coroutine state here.
/// </remarks>
public sealed class GravitasWorldContext : IDisposable
{
    private static readonly object _worldOwnershipLock = new();

    private static readonly SwiftDictionary<GridWorld, WeakReference<GravitasWorldContext>> _worldOwners = new();

    private readonly GravitasClock _clock = new();

    private readonly GravitasLifecycleHooks _hooks = new();

    private readonly bool _ownsWorld;

    private bool _disposed;

    private GravitasWorldContext(GridWorld world, bool ownsWorld)
    {
        World = world;
        _ownsWorld = ownsWorld;
        Settings = PhysicsSettings.DefaultSettings();
        Environment = PhysicsEnvironment.Default(Settings.FrameRate);
        Collisions = new GravitasCollisionService(this);
        Physics = new GravitasPhysicsService(this);
        Raycasts = new GravitasRaycastService(this);
        Circlecasts = new GravitasCirclecastService(this);
        Coroutines = new GravitasCoroutineService(this);
    }

    /// <summary>
    /// Gets the explicit GridForge world owned or referenced by this context.
    /// </summary>
    public GridWorld World { get; }

    /// <summary>
    /// Gets this context's world-local physics settings.
    /// </summary>
    public PhysicsSettings Settings { get; private set; }

    /// <summary>
    /// Gets this context's world-local physical environment values.
    /// </summary>
    public PhysicsEnvironment Environment { get; }

    /// <summary>
    /// Gets this context's world-local collision partitioning service.
    /// </summary>
    public GravitasCollisionService Collisions { get; }

    /// <summary>
    /// Gets this context's world-local physics registration and pair service.
    /// </summary>
    public GravitasPhysicsService Physics { get; }

    /// <summary>
    /// Gets this context's world-local raycast query service.
    /// </summary>
    public GravitasRaycastService Raycasts { get; }

    /// <summary>
    /// Gets this context's world-local circlecast query service.
    /// </summary>
    public GravitasCirclecastService Circlecasts { get; }

    /// <summary>
    /// Gets this context's world-local lockstep coroutine service.
    /// </summary>
    public GravitasCoroutineService Coroutines { get; }

    /// <summary>
    /// Gets whether this context has been disposed.
    /// </summary>
    public bool IsDisposed => _disposed;

    /// <summary>
    /// Gets the voxel size of this context's world.
    /// </summary>
    public Fixed64 VoxelSize
    {
        get
        {
            ThrowIfDisposed();
            return World.VoxelSize;
        }
    }

    /// <summary>
    /// Gets this context's fixed simulation frame rate.
    /// </summary>
    public int FrameRate
    {
        get
        {
            ThrowIfDisposed();
            return _clock.FrameRate;
        }
    }

    /// <summary>
    /// Gets this context's fixed simulation time step.
    /// </summary>
    public Fixed64 DeltaTime
    {
        get
        {
            ThrowIfDisposed();
            return _clock.DeltaTime;
        }
    }

    /// <summary>
    /// Gets the reciprocal of this context's fixed simulation time step.
    /// </summary>
    public Fixed64 InvDeltaTime
    {
        get
        {
            ThrowIfDisposed();
            return _clock.InvDeltaTime;
        }
    }

    /// <summary>
    /// Gets this context's simulated frame count.
    /// </summary>
    public int FrameCount
    {
        get
        {
            ThrowIfDisposed();
            return _clock.FrameCount;
        }
    }

    /// <summary>
    /// Gets this context's total simulated time.
    /// </summary>
    public Fixed64 TotalTime
    {
        get
        {
            ThrowIfDisposed();
            return _clock.TotalTime;
        }
    }

    /// <summary>
    /// Gets this context's accumulated visualization time.
    /// </summary>
    public Fixed64 AccumulatedTime
    {
        get
        {
            ThrowIfDisposed();
            return _clock.AccumulatedTime;
        }
    }

    /// <summary>
    /// Gets whether this context's visualization accumulation will reset on the next visualize call.
    /// </summary>
    public bool ResetAccumulation
    {
        get
        {
            ThrowIfDisposed();
            return _clock.ResetAccumulation;
        }
    }

    /// <summary>
    /// Gets this context's visualization accumulation expressed in simulation frames.
    /// </summary>
    public Fixed64 ExpectedAccumulation
    {
        get
        {
            ThrowIfDisposed();
            return _clock.ExpectedAccumulation;
        }
    }

    /// <summary>
    /// Attaches a context to a host-owned <see cref="GridWorld"/>.
    /// </summary>
    /// <param name="world">The active world to bind.</param>
    /// <param name="takeOwnership">True when disposing this context should dispose the supplied world.</param>
    /// <returns>A context bound to <paramref name="world"/>.</returns>
    public static GravitasWorldContext Attach(GridWorld world, bool takeOwnership = false)
    {
        SwiftThrowHelper.ThrowIfNull(world, nameof(world));
        SwiftThrowHelper.ThrowIfTrue(
            !world.IsActive,
            nameof(GravitasWorldContext),
            "GravitasWorldContext requires an active GridWorld.");

        return CreateRegistered(world, takeOwnership);
    }

    /// <summary>
    /// Creates a context with an owned <see cref="GridWorld"/>.
    /// </summary>
    /// <param name="voxelSize">Optional voxel size for the created world.</param>
    /// <param name="spatialGridCellSize">Spatial hash cell size for the created world.</param>
    /// <returns>A context that owns its created world.</returns>
    public static GravitasWorldContext CreateOwned(
        Fixed64? voxelSize = null,
        int spatialGridCellSize = GridWorld.DefaultSpatialGridCellSize)
    {
        return CreateRegistered(
            new GridWorld(voxelSize, spatialGridCellSize),
            ownsWorld: true);
    }

    /// <summary>
    /// Advances this context's deterministic simulation clock and ordered simulate hooks.
    /// </summary>
    public void Simulate()
    {
        ThrowIfDisposed();
        _clock.Simulate();
        Physics.Simulate();
        Coroutines.Simulate();
        _hooks.InvokeSimulate();
    }

    /// <summary>
    /// Runs this context's late-simulation step.
    /// </summary>
    public void LateSimulate()
    {
        ThrowIfDisposed();
        _clock.LateSimulate();
        Physics.LateSimulate();
        _hooks.InvokeLateSimulate();
    }

    /// <summary>
    /// Runs this context's visualization accumulation step.
    /// </summary>
    public void Visualize()
    {
        ThrowIfDisposed();
        _clock.Visualize();
        Physics.Visualize();
        _hooks.InvokeVisualize();
    }

    /// <summary>
    /// Runs this context's late-visualization step.
    /// </summary>
    public void LateVisualize()
    {
        ThrowIfDisposed();
        Physics.LateVisualize();
        _hooks.InvokeLateVisualize();
    }

    /// <summary>
    /// Resets this context's deterministic clock and context-local lifecycle hooks.
    /// </summary>
    public void Reset()
    {
        ThrowIfDisposed();
        _clock.Reset();
        Collisions.Reset();
        Physics.Reset();
        Raycasts.Reset();
        Circlecasts.Reset();
        Coroutines.Reset();
        _hooks.InvokeReset();
    }

    /// <summary>
    /// Updates this context's fixed simulation frame rate.
    /// </summary>
    /// <param name="frameRate">The new frame rate. Must be greater than zero.</param>
    public void SetFrameRate(int frameRate)
    {
        ThrowIfDisposed();
        Settings.SetFrameRate(frameRate);
        _clock.SetFrameRate(frameRate);
        _hooks.InvokeFrameRateChanged();
    }

    /// <summary>
    /// Applies context-local settings and synchronizes frame-derived clock state.
    /// </summary>
    /// <param name="settings">The settings instance to own.</param>
    public void ApplySettings(PhysicsSettings settings)
    {
        ThrowIfDisposed();
        SwiftThrowHelper.ThrowIfNull(settings, nameof(settings));

        Settings = settings;
        _clock.SetFrameRate(settings.FrameRate);
        _hooks.InvokeFrameRateChanged();
    }

    /// <summary>
    /// Calculates the frame index containing the specified fixed-point timestamp.
    /// </summary>
    /// <param name="timestamp">The timestamp to resolve.</param>
    /// <returns>The zero-based frame index for the timestamp.</returns>
    public int GetFrameFromTime(Fixed64 timestamp)
    {
        ThrowIfDisposed();
        return _clock.GetFrameFromTime(timestamp);
    }

    internal IDisposable RegisterOnSimulate(string owner, int order, Action callback)
    {
        ThrowIfDisposed();
        return _hooks.RegisterOnSimulate(owner, order, callback);
    }

    internal IDisposable RegisterOnLateSimulate(string owner, int order, Action callback)
    {
        ThrowIfDisposed();
        return _hooks.RegisterOnLateSimulate(owner, order, callback);
    }

    internal IDisposable RegisterOnVisualize(string owner, int order, Action callback)
    {
        ThrowIfDisposed();
        return _hooks.RegisterOnVisualize(owner, order, callback);
    }

    internal IDisposable RegisterOnLateVisualize(string owner, int order, Action callback)
    {
        ThrowIfDisposed();
        return _hooks.RegisterOnLateVisualize(owner, order, callback);
    }

    internal IDisposable RegisterOnReset(string owner, int order, Action callback)
    {
        ThrowIfDisposed();
        return _hooks.RegisterOnReset(owner, order, callback);
    }

    internal IDisposable RegisterOnFrameRateChanged(string owner, int order, Action callback)
    {
        ThrowIfDisposed();
        return _hooks.RegisterOnFrameRateChanged(owner, order, callback);
    }

    /// <inheritdoc/>
    public void Dispose()
    {
        lock (_worldOwnershipLock)
        {
            if (_disposed)
                return;

            _disposed = true;
            ReleaseWorldOwnership(this);

            if (_ownsWorld && World.IsActive)
                World.Dispose();
        }
    }

    private static GravitasWorldContext CreateRegistered(GridWorld world, bool ownsWorld)
    {
        lock (_worldOwnershipLock)
        {
            ThrowIfWorldOwned(world);
            GravitasWorldContext context = new(world, ownsWorld);
            _worldOwners[world] = new WeakReference<GravitasWorldContext>(context);
            return context;
        }
    }

    private static void ThrowIfWorldOwned(GridWorld world)
    {
        if (!_worldOwners.TryGetValue(world, out WeakReference<GravitasWorldContext> weakOwner))
            return;

        bool worldIsOwned =
            weakOwner.TryGetTarget(out GravitasWorldContext? owner)
            && !owner.IsDisposed
            && owner.World.IsActive;
        SwiftThrowHelper.ThrowIfTrue(
            worldIsOwned,
            nameof(GravitasWorldContext),
            "GridWorld is already attached to an active GravitasWorldContext.");

        _worldOwners.Remove(world);
    }

    private static void ReleaseWorldOwnership(GravitasWorldContext context)
    {
        if (!_worldOwners.TryGetValue(context.World, out WeakReference<GravitasWorldContext> weakOwner))
            return;

        if (!weakOwner.TryGetTarget(out GravitasWorldContext? owner)
            || ReferenceEquals(owner, context))
        {
            _worldOwners.Remove(context.World);
        }
    }

    private void ThrowIfDisposed()
    {
        SwiftThrowHelper.ThrowIfDisposed(_disposed, nameof(GravitasWorldContext));
        SwiftThrowHelper.ThrowIfTrue(
            !World.IsActive,
            nameof(GravitasWorldContext),
            "GravitasWorldContext is bound to an inactive GridWorld.");
    }
}
