using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Diagnostics;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class DiagnosticsBenchmarks
{
    private GravitasWorldContext _disabledContext;
    private GravitasWorldContext _enabledEventContext;
    private GravitasWorldContext _enabledDrawContext;
    private SwiftList<StiffBody> _disabledBodies;
    private SwiftList<StiffBody> _enabledEventBodies;
    private SwiftList<StiffBody> _enabledDrawBodies;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount);

        _disabledContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        _disabledBodies = new SwiftList<StiffBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_disabledContext, ColliderCount, _disabledBodies);

        _enabledEventContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _enabledEventBodies = new SwiftList<StiffBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_enabledEventContext, ColliderCount, _enabledEventBodies);
        _enabledEventContext.Diagnostics.Enable(eventCapacity: ColliderCount * 2, drawCommandCapacity: 0);

        _enabledDrawContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _enabledDrawBodies = new SwiftList<StiffBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_enabledDrawContext, ColliderCount, _enabledDrawBodies);
        _enabledDrawContext.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: ColliderCount);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _disabledContext.Dispose();
        _enabledEventContext.Dispose();
        _enabledDrawContext.Dispose();

        _disabledContext = null;
        _enabledEventContext = null;
        _enabledDrawContext = null;
        _disabledBodies = null;
        _enabledEventBodies = null;
        _enabledDrawBodies = null;
    }

    [Benchmark(Baseline = true)]
    public int ForceAndTorqueDiagnosticsDisabled()
    {
        ApplyForceAndTorque(_disabledBodies);
        return _disabledContext.Diagnostics.EventCount;
    }

    [Benchmark]
    public int ForceAndTorqueDiagnosticsEnabled()
    {
        _enabledEventContext.Diagnostics.Clear();
        ApplyForceAndTorque(_enabledEventBodies);
        return _enabledEventContext.Diagnostics.EventCount;
    }

    [Benchmark]
    public int ColliderCaptureDiagnosticsDisabled()
    {
        CaptureColliders(_disabledContext, _disabledBodies);
        return _disabledContext.Diagnostics.DrawCommandCount;
    }

    [Benchmark]
    public int ColliderCaptureDiagnosticsEnabled()
    {
        _enabledDrawContext.Diagnostics.Clear();
        CaptureColliders(_enabledDrawContext, _enabledDrawBodies);
        return _enabledDrawContext.Diagnostics.DrawCommandCount;
    }

    private static void ApplyForceAndTorque(SwiftList<StiffBody> bodies)
    {
        var force = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        var torque = new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero);

        for (int i = 0; i < bodies.Count; i++)
        {
            StiffBody body = bodies[i];
            body.AddForce(force);
            body.AddTorque(torque);
        }
    }

    private static void CaptureColliders(GravitasWorldContext context, SwiftList<StiffBody> bodies)
    {
        for (int i = 0; i < bodies.Count; i++)
            context.Diagnostics.CaptureCollider(bodies[i].Collider, GravitasDiagnosticColor.Cyan);
    }
}
