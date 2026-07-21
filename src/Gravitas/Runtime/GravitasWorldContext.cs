//=======================================================================
// GravitasWorldContext.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Grids.Topology;
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

    private static readonly SwiftDictionary<GridWorld, GravitasWorldContext> _worldOwners = new();

    private readonly GravitasClock _clock = new();

    private readonly GravitasLifecycleHooks _hooks = new();

    private readonly bool _ownsWorld;

    private bool _disposed;

    private int _lateSimulateToken;

    private int _simulationPhaseDepth;

    private bool _fixedStepOpen;

    internal void EnterSimulationPhase() => _simulationPhaseDepth++;

    internal void ExitSimulationPhase() => _simulationPhaseDepth--;

    internal void ThrowIfFixedStepMutationNotAllowed()
    {
        SwiftThrowHelper.ThrowIfTrue(
            _fixedStepOpen || _simulationPhaseDepth > 0,
            nameof(GravitasWorldContext),
            "Authoritative body roles, static poses, and loaded state can change only outside the Simulate-to-LateSimulate fixed-step transaction.");
    }

    private GravitasWorldContext(GridWorld world, bool ownsWorld)
    {
        World = world;
        _ownsWorld = ownsWorld;
        Settings = PhysicsSettings.DefaultSettings();
        Environment = PhysicsEnvironment.Default(Settings.FrameRate);
        CollisionScratch = new CollisionSatScratch();
        Diagnostics = new GravitasDiagnosticSink(this);
        Constraints3D = new GravitasConstraint3DService(this);
        Constraints2D = new GravitasConstraint2DService(this);
        Collisions = new GravitasCollisionService(this);
        Collisions2D = new GravitasCollision2DService(this);
        Physics = new GravitasPhysicsService(this);
        Physics2D = new GravitasPhysics2DService(this);
        MixedCollisions = new GravitasMixedCollisionService(this);
        Query2D = new GravitasQuery2DService(this);
        Query3D = new GravitasQuery3DService(this);
        QueryMixed = new GravitasQueryMixedService(this);
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
    /// Gets this context's world-local pure 2D collision partitioning service.
    /// </summary>
    public GravitasCollision2DService Collisions2D { get; }

    /// <summary>
    /// Gets this context's world-local physics registration and pair service.
    /// </summary>
    public GravitasPhysicsService Physics { get; }

    /// <summary>
    /// Gets this context's world-local pure 2D physics service.
    /// </summary>
    public GravitasPhysics2DService Physics2D { get; }

    internal GravitasMixedCollisionService MixedCollisions { get; }

    /// <summary>
    /// Gets this context's world-local pure 2D query service.
    /// </summary>
    public GravitasQuery2DService Query2D { get; }

    /// <summary>
    /// Gets this context's world-local 3D query service.
    /// </summary>
    public GravitasQuery3DService Query3D { get; }

    /// <summary>
    /// Gets this context's explicit mixed 3D/2D query service.
    /// </summary>
    public GravitasQueryMixedService QueryMixed { get; }

    /// <summary>
    /// Gets this context's world-local lockstep coroutine service.
    /// </summary>
    public GravitasCoroutineService Coroutines { get; }

    /// <summary>
    /// Gets this context's deterministic diagnostic sink.
    /// </summary>
    public GravitasDiagnosticSink Diagnostics { get; }

    /// <summary>
    /// Gets this context's world-local 3D constraint and ragdoll service.
    /// </summary>
    public GravitasConstraint3DService Constraints3D { get; }

    /// <summary>
    /// Gets this context's world-local pure 2D constraint and ragdoll service.
    /// </summary>
    public GravitasConstraint2DService Constraints2D { get; }

    internal CollisionSatScratch CollisionScratch { get; }

    /// <summary>
    /// Gets whether this context has been disposed.
    /// </summary>
    public bool IsDisposed => _disposed;

    /// <summary>
    /// Gets a representative grid cell edge for this context's world.
    /// </summary>
    public Fixed64 VoxelSize
    {
        get
        {
            ThrowIfDisposed();
            return GridTopologyMetricUtility.GetRepresentativeCellEdge(World);
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

    internal bool ResetAccumulationThisVisualize
    {
        get
        {
            ThrowIfDisposed();
            return _clock.ResetAccumulationThisVisualize;
        }
    }

    internal int LateSimulateToken => _lateSimulateToken;

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
    /// Computes a deterministic fixed-width hash of this context's replay-relevant physics state.
    /// </summary>
    /// <param name="mode">Selects whether diagnostic solver/cache state is included in addition to authoritative state.</param>
    /// <returns>A deterministic hash suitable for lockstep replay conformance checks.</returns>
    /// <remarks>
    /// The hash includes context settings, environment values, body state, collider shape/filter state,
    /// retained pair/contact state, and continuation-affecting CCD handoff state. It excludes host object
    /// identity, delegates, diagnostics buffers, debug draw data, query scratch buffers, and visualization
    /// interpolation caches.
    /// </remarks>
    public ChronicleHash ComputeReplayHash(
        GravitasReplayHashMode mode = GravitasReplayHashMode.Authoritative)
    {
        ThrowIfDisposed();
        return GravitasReplayHashService.Compute(this, mode);
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
        return CreateRegistered(world, takeOwnership);
    }

    /// <summary>
    /// Creates a context with an owned <see cref="GridWorld"/>.
    /// </summary>
    /// <param name="spatialGridCellSize">Spatial hash cell size for the created world.</param>
    /// <returns>A context that owns its created world.</returns>
    public static GravitasWorldContext CreateOwned(
        int spatialGridCellSize = GridWorld.DefaultSpatialGridCellSize)
    {
        return CreateRegistered(
            new GridWorld(spatialGridCellSize),
            ownsWorld: true);
    }

    /// <summary>
    /// Advances this context's deterministic simulation clock and ordered simulate hooks.
    /// </summary>
    public void Simulate()
    {
        ThrowIfDisposed();
        _fixedStepOpen = true;
        EnterSimulationPhase();
        try
        {
            _clock.Simulate();
            PhysicsRuntimeMode runtimeMode = Settings.RuntimeMode;
            if (runtimeMode.Runs3D())
                Physics.Simulate();
            if (runtimeMode.Runs2D())
                Physics2D.Simulate();
            if (runtimeMode.RunsMixedContacts())
                MixedCollisions.Simulate();

            Coroutines.Simulate();
            _hooks.InvokeSimulate();
        }
        catch
        {
            _fixedStepOpen = false;
            throw;
        }
        finally
        {
            ExitSimulationPhase();
        }
    }

    /// <summary>
    /// Runs this context's late-simulation step.
    /// </summary>
    public void LateSimulate()
    {
        ThrowIfDisposed();
        _fixedStepOpen = true;
        EnterSimulationPhase();
        try
        {
            _clock.LateSimulate();
            PhysicsRuntimeMode runtimeMode = Settings.RuntimeMode;
            bool willRun3D = runtimeMode.Runs3D() && Physics.SimulatePhysics;
            bool willRun2D = runtimeMode.Runs2D() && Physics2D.SimulatePhysics;
            if (willRun3D || willRun2D)
                AdvanceLateSimulateToken();
            if (willRun3D)
                Physics.PrepareContinuousCollisionFrame();
            if (willRun2D)
                Physics2D.PrepareContinuousCollisionFrame();
            bool ran3D = willRun3D
                && Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: true);
            bool ran2D = willRun2D
                && Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: true);
            ProcessQueuedContinuousCollisionHandoffs(ran3D, ran2D);
            if (ran3D)
                Physics.CompleteLateSimulatePhysicsStep();
            if (ran2D)
                Physics2D.CompleteLateSimulatePhysicsStep();
            if (runtimeMode.RunsMixedContacts())
                MixedCollisions.LateSimulate();

            _hooks.InvokeLateSimulate();
        }
        finally
        {
            ExitSimulationPhase();
            _fixedStepOpen = false;
        }
    }

    private void ProcessQueuedContinuousCollisionHandoffs(bool runs3D, bool runs2D)
    {
        if (!runs3D && !runs2D)
            return;

        try
        {
            int iterationLimit = Settings.ContinuousCollisionMaxToiIterations;
            int remainingIterations = iterationLimit;
            for (int iteration = 0; iteration < iterationLimit && remainingIterations > 0; iteration++)
            {
                int processedIterations = 0;
                if (runs3D)
                {
                    int usedIterations = Physics.ProcessQueuedContinuousCollisionHandoffs(remainingIterations);
                    processedIterations += usedIterations;
                    remainingIterations -= usedIterations;
                }

                if (remainingIterations > 0 && runs2D)
                {
                    int usedIterations = Physics2D.ProcessQueuedContinuousCollisionHandoffs(remainingIterations);
                    processedIterations += usedIterations;
                    remainingIterations -= usedIterations;
                }

                if (processedIterations == 0)
                    return;
            }

            if (runs3D)
                Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 0);
            if (runs2D)
                Physics2D.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 0);
        }
        catch
        {
            if (runs3D)
                Physics.AbortContinuousCollisionHandoffFrame();
            if (runs2D)
                Physics2D.AbortContinuousCollisionHandoffFrame();
            throw;
        }
    }

    internal void AdvanceLateSimulateToken() => _lateSimulateToken++;

    /// <summary>
    /// Runs this context's visualization accumulation step.
    /// </summary>
    public void Visualize()
    {
        ThrowIfDisposed();
        _clock.Visualize();
        PhysicsRuntimeMode runtimeMode = Settings.RuntimeMode;
        if (runtimeMode.Runs3D())
            Physics.Visualize();
        if (runtimeMode.Runs2D())
            Physics2D.Visualize();
        if (runtimeMode.RunsMixedContacts())
            MixedCollisions.Visualize();

        _hooks.InvokeVisualize();
    }

    /// <summary>
    /// Runs this context's host-owned late-visualization hook phase.
    /// </summary>
    public void LateVisualize()
    {
        ThrowIfDisposed();
        _hooks.InvokeLateVisualize();
    }

    /// <summary>
    /// Resets this context's deterministic clock and context-local lifecycle hooks.
    /// </summary>
    public void Reset()
    {
        ThrowIfDisposed();
        _lateSimulateToken = 0;
        _clock.Reset();
        Constraints3D.Reset();
        Constraints2D.Reset();
        Collisions.Reset();
        Collisions2D.Reset();
        Physics.Reset();
        Physics2D.Reset();
        MixedCollisions.Reset();
        Query2D.Reset();
        Query3D.Reset();
        QueryMixed.Reset();
        Coroutines.Reset();
        Diagnostics.Reset();
        _hooks.InvokeReset();
    }

    /// <summary>
    /// Updates this context's fixed simulation frame rate.
    /// </summary>
    /// <param name="frameRate">The new frame rate. Must be within the supported physics settings range.</param>
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

            Constraints3D.Reset();
            Constraints2D.Reset();
            _disposed = true;
            try
            {
                Coroutines.Deactivate();
            }
            finally
            {
                try
                {
                    if (_ownsWorld && World.IsActive)
                        World.Dispose();
                }
                finally
                {
                    ReleaseWorldOwnership(this);
                }
            }
        }
    }

    private static GravitasWorldContext CreateRegistered(GridWorld world, bool ownsWorld)
    {
        lock (_worldOwnershipLock)
        {
            SwiftThrowHelper.ThrowIfTrue(
                !world.IsActive,
                nameof(GravitasWorldContext),
                "GravitasWorldContext requires an active GridWorld.");
            ThrowIfWorldOwned(world);
            GravitasWorldContext context = new(world, ownsWorld);
            _worldOwners[world] = context;
            return context;
        }
    }

    private static void ThrowIfWorldOwned(GridWorld world)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _worldOwners.ContainsKey(world),
            nameof(GravitasWorldContext),
            "GridWorld is already attached to an active GravitasWorldContext.");
    }

    private static void ReleaseWorldOwnership(GravitasWorldContext context)
    {
        _worldOwners.Remove(context.World);
    }

    internal void ThrowIfDisposed()
    {
        SwiftThrowHelper.ThrowIfDisposed(_disposed, nameof(GravitasWorldContext));
        SwiftThrowHelper.ThrowIfTrue(
            !World.IsActive,
            nameof(GravitasWorldContext),
            "GravitasWorldContext is bound to an inactive GridWorld.");
    }
}
