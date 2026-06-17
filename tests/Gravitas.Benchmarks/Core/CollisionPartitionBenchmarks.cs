using BenchmarkDotNet.Attributes;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionPartitionBenchmarks
{
    private GravitasWorldContext _simulateContext;
    private GravitasWorldContext _resetContext;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup(Target = nameof(SimulatePartitionedDynamicSpheres))]
    public void SetupSimulationContext()
    {
        _simulateContext = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount),
            clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_simulateContext, ColliderCount);
    }

    [GlobalSetup(Target = nameof(ResetAndReRegisterDynamicSpheres))]
    public void SetupResetContext()
    {
        _resetContext = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount),
            clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_resetContext, ColliderCount);
    }

    [GlobalCleanup(Target = nameof(SimulatePartitionedDynamicSpheres))]
    public void CleanupSimulationContext()
    {
        _simulateContext.Dispose();
        _simulateContext = null;
    }

    [GlobalCleanup(Target = nameof(ResetAndReRegisterDynamicSpheres))]
    public void CleanupResetContext()
    {
        _resetContext.Dispose();
        _resetContext = null;
    }

    [Benchmark]
    public int CreateAndRegisterDynamicSpheres()
    {
        using GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount));
        return BenchmarkPhysicsScene.CreateDynamicSphereGrid(context, ColliderCount);
    }

    [Benchmark]
    public int CreateAndPartitionStaticSpheres()
    {
        using GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount));
        return BenchmarkPhysicsScene.CreateStaticSphereGrid(context, ColliderCount);
    }

    [Benchmark]
    public int SimulatePartitionedDynamicSpheres()
    {
        _simulateContext.Simulate();
        _simulateContext.LateSimulate();
        return _simulateContext.FrameCount;
    }

    [Benchmark]
    public int ResetAndReRegisterDynamicSpheres()
    {
        _resetContext.Reset();
        return BenchmarkPhysicsScene.CreateDynamicSphereGrid(_resetContext, ColliderCount);
    }
}
