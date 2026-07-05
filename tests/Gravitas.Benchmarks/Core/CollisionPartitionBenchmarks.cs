using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionPartitionBenchmarks
{
    private const int BodylessColliderCount = 4;

    private GravitasWorldContext _simulateContext;
    private GravitasWorldContext _resetContext;
    private GravitasWorldContext _mostlyDynamicWithBodylessContext;

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

    [GlobalSetup(Target = nameof(LateSimulateMostlyDynamicWithBodylessStaticSpheres))]
    public void SetupMostlyDynamicWithBodylessContext()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount) + 16;
        _mostlyDynamicWithBodylessContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_mostlyDynamicWithBodylessContext, ColliderCount);

        for (int i = 0; i < BodylessColliderCount; i++)
        {
            BenchmarkPhysicsScene.CreateStaticCollider(
                _mostlyDynamicWithBodylessContext,
                new LSSphereCollider(),
                new Vector3d((Fixed64)(i * 2), Fixed64.Zero, (Fixed64)(gridExtent - 2)));
        }
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

    [GlobalCleanup(Target = nameof(LateSimulateMostlyDynamicWithBodylessStaticSpheres))]
    public void CleanupMostlyDynamicWithBodylessContext()
    {
        _mostlyDynamicWithBodylessContext.Dispose();
        _mostlyDynamicWithBodylessContext = null;
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
    public int LateSimulateMostlyDynamicWithBodylessStaticSpheres()
    {
        _mostlyDynamicWithBodylessContext.Simulate();
        _mostlyDynamicWithBodylessContext.LateSimulate();
        return _mostlyDynamicWithBodylessContext.Physics.AssimilatedColliderCount;
    }

    [Benchmark]
    public int ResetAndReRegisterDynamicSpheres()
    {
        _resetContext.Reset();
        return BenchmarkPhysicsScene.CreateDynamicSphereGrid(_resetContext, ColliderCount);
    }
}
