using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Raycasting;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class SimulationAllocationBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _lateSimulateContext;
    private GravitasWorldContext _groundingContext;
    private GravitasWorldContext _distributionContext;
    private GravitasWorldContext _activePairContext;
    private SwiftList<StiffBody> _groundedBodies;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount);

        _lateSimulateContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_lateSimulateContext, ColliderCount);

        _groundingContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _groundedBodies = new SwiftList<StiffBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_groundingContext, ColliderCount, _groundedBodies);

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
        _distributionContext.Dispose();
        _activePairContext.Dispose();

        _lateSimulateContext = null;
        _groundingContext = null;
        _distributionContext = null;
        _activePairContext = null;
        _groundedBodies = null;
    }

    [Benchmark]
    public int StiffBodyLateSimulateOnly()
    {
        _lateSimulateContext.Physics.LateSimulate();
        return _lateSimulateContext.Physics.AssimilatedBodyCount;
    }

    [Benchmark]
    public int GroundingCircleCastOnly()
    {
        int groundedCount = 0;
        for (int i = 0; i < _groundedBodies.Count; i++)
        {
            StiffBody body = _groundedBodies[i];
            Vector3d origin = body.Position3d;
            origin.y += body.GroundOriginOffset;

            if (_groundingContext.Circlecasts.CircleCast(
                    origin,
                    body.GroundCheckSphereRadius,
                    Vector3d.Down,
                    out LSRaycastHit _,
                    body.GroundedDistanceRay,
                    IncludeLayerZero))
            {
                groundedCount++;
            }
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
        return _activePairContext.Physics.AssimilatedBodyCount;
    }
}
