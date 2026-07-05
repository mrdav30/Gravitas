using BenchmarkDotNet.Attributes;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class SimulationAllocationBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _lateSimulateContext;
    private GravitasWorldContext _groundingContext;
    private GravitasWorldContext _sweptGroundingContext;
    private GravitasWorldContext _distributionContext;
    private GravitasWorldContext _activePairContext;
    private SwiftList<SolidBody> _groundedBodies;
    private SwiftList<SolidBody> _sweptGroundedBodies;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount);

        _lateSimulateContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_lateSimulateContext, ColliderCount);

        _groundingContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _groundingContext.Settings.GroundCheckLayerMask = IncludeLayerZero;
        _groundedBodies = new SwiftList<SolidBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_groundingContext, ColliderCount, _groundedBodies);
        SetGroundProbeMode(_groundedBodies, GroundProbeMode.Ray);

        _sweptGroundingContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _sweptGroundingContext.Settings.GroundCheckLayerMask = IncludeLayerZero;
        _sweptGroundedBodies = new SwiftList<SolidBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_sweptGroundingContext, ColliderCount, _sweptGroundedBodies);
        SetGroundProbeMode(_sweptGroundedBodies, GroundProbeMode.SweptSphere);

        _distributionContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_distributionContext, ColliderCount);

        _activePairContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        BenchmarkPhysicsScene.CreateOverlappingDynamicSpherePairs(_activePairContext, ColliderCount / 2);
        _activePairContext.Simulate();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _lateSimulateContext.Dispose();
        _groundingContext.Dispose();
        _sweptGroundingContext.Dispose();
        _distributionContext.Dispose();
        _activePairContext.Dispose();

        _lateSimulateContext = null;
        _groundingContext = null;
        _sweptGroundingContext = null;
        _distributionContext = null;
        _activePairContext = null;
        _groundedBodies = null;
        _sweptGroundedBodies = null;
    }

    [Benchmark]
    public int SolidBodyLateSimulateOnly()
    {
        _lateSimulateContext.Physics.LateSimulate();
        return _lateSimulateContext.Physics.BodyCount;
    }

    [Benchmark]
    public int GroundingRaycastProbeOnly()
    {
        int groundedCount = 0;
        for (int i = 0; i < _groundedBodies.Count; i++)
        {
            SolidBody body = _groundedBodies[i];
            body.CheckGround();
            if (body.IsGrounded)
                groundedCount++;
        }

        return groundedCount;
    }

    [Benchmark]
    public int GroundingSweptSphereProbeOnly()
    {
        int groundedCount = 0;
        for (int i = 0; i < _sweptGroundedBodies.Count; i++)
        {
            SolidBody body = _sweptGroundedBodies[i];
            body.CheckGround();
            if (body.IsGrounded)
                groundedCount++;
        }

        return groundedCount;
    }

    [Benchmark]
    public uint CollisionPartitionDistributionOnly()
    {
        _distributionContext.Collisions.CheckAndDistributeCollisions();
        return _distributionContext.Collisions.Version;
    }

    [Benchmark]
    public int ActivePairProcessingLateSimulate()
    {
        _activePairContext.Physics.LateSimulate();
        return _activePairContext.Physics.BodyCount;
    }

    private static void SetGroundProbeMode(SwiftList<SolidBody> bodies, GroundProbeMode mode)
    {
        for (int i = 0; i < bodies.Count; i++)
            bodies[i].GroundProbeMode = mode;
    }
}
