using BenchmarkDotNet.Attributes;
using GridForge.Grids;

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
    public int RunEmptySimulationFrame()
    {
        _emptyContext.Simulate();
        _emptyContext.LateSimulate();
        _emptyContext.Visualize();
        _emptyContext.LateVisualize();
        return _emptyContext.FrameCount;
    }
}
