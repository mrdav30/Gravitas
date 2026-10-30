using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using GridForge.Grids;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class WorldContextBenchmarks
{
    private GravitasWorldContext _emptyContext;

    [GlobalSetup(Target = nameof(RunEmptySimulationFrame))]
    public void SetupEmptySimulationContext()
    {
        _emptyContext = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools: true);
    }

    [GlobalCleanup(Target = nameof(RunEmptySimulationFrame))]
    public void CleanupEmptySimulationContext()
    {
        _emptyContext.Dispose();
        _emptyContext = null;
    }

    [Benchmark]
    public int CreateAndDisposeOwnedContext()
    {
        using GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        return context.FrameRate;
    }

    [Benchmark]
    public int CreateGridAndDisposeOwnedContext()
    {
        using GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(16);
        return context.World.ActiveGrids.Count;
    }

    [Benchmark]
    public int AttachHostOwnedWorld()
    {
        var world = new GridWorld();
        try
        {
            using GravitasWorldContext context = GravitasWorldContext.Attach(world);
            return context.FrameRate;
        }
        finally
        {
            world.Dispose();
        }
    }

    [Benchmark]
    public long RunEmptySimulationFrame()
    {
        _emptyContext.Simulate();
        _emptyContext.LateSimulate();
        _emptyContext.Visualize();
        _emptyContext.LateVisualize();
        return _emptyContext.FrameCount;
    }
}

/// <summary>Reset after registration, contacts, CCD candidate preparation and replay inspection.</summary>
[MemoryDiagnoser]
public class WorldContextResetBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody[] _bodies3D;
    private SolidBody2D[] _bodies2D;
    private BenchmarkMatterAgent _floorAgent3D;
    private BenchmarkMatterAgent _floorAgent2D;
    private readonly LSCuboidCollider _floor3D = new() { Size = new(32, 2, 32), Material = PhysicsMaterial.Frictionless };
    private readonly LSAABBoxCollider2D _floor2D = new(new Vector2d(32, 2)) { Material = PhysicsMaterial.Frictionless };

    [Params(PhysicsRuntimeMode.ThreeD, PhysicsRuntimeMode.TwoD, PhysicsRuntimeMode.Both, PhysicsRuntimeMode.Mixed)]
    public PhysicsRuntimeMode Mode { get; set; }

    [Params(16, 128)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(20);
        _context.Settings.RuntimeMode = Mode;
        _context.SetFrameRate(16);
        _context.Environment.Gravity = Fixed64.Zero;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        _floorAgent3D = new BenchmarkMatterAgent(_context, new Vector3d(0, -1, 0));
        _floorAgent2D = new BenchmarkMatterAgent(_context, new Vector3d(0, 0, -16));
        _bodies3D = Mode == PhysicsRuntimeMode.TwoD ? Array.Empty<SolidBody>() : new SolidBody[BodyCount];
        _bodies2D = Mode == PhysicsRuntimeMode.ThreeD ? Array.Empty<SolidBody2D>() : new SolidBody2D[BodyCount];
        for (int i = 0; i < _bodies3D.Length; i++)
            _bodies3D[i] = new SolidBody(new BenchmarkMatterAgent(_context, Vector3d.Zero), new LSSphereCollider())
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation, SleepEnabled = false };
        for (int i = 0; i < _bodies2D.Length; i++)
            _bodies2D[i] = new SolidBody2D(new BenchmarkMatterAgent(_context, Vector3d.Zero), new LSCircleCollider2D(Fixed64.Half))
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
    }

    [IterationSetup]
    public void PrepareReset()
    {
        foreach (SolidBody body in _bodies3D) body.Deactivate();
        foreach (SolidBody2D body in _bodies2D) body.Deactivate();
        _floor3D.Deactivate();
        _floor2D.Deactivate();
        if (_bodies3D.Length != 0)
            _floor3D.InitializeWithNoBody(_floorAgent3D);
        if (_bodies2D.Length != 0)
            _floor2D.InitializeWithNoBody(_floorAgent2D);
        for (int i = 0; i < _bodies3D.Length; i++)
        {
            _bodies3D[i].Initialize(new Vector3d((Fixed64)(i % 8 * 2 - 7), Fixed64.Quarter,
                (Fixed64)(i / 8 * 2 - 15)), FixedQuaternion.Identity);
            _bodies3D[i].UseManualGrounding();
        }
        for (int i = 0; i < _bodies2D.Length; i++)
            _bodies2D[i].Initialize(new Vector2d((Fixed64)(i % 8 * 2 - 7),
                (Fixed64)(i / 8 * 2 - 15) + Fixed64.Quarter));
        _context.Simulate(); _context.LateSimulate();
        _context.ComputeReplayHash();
    }

    [Benchmark]
    public int ResetPopulatedContext()
    {
        _context.Reset();
        return _context.Physics.ColliderCount + _context.Physics2D.ColliderCount;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
