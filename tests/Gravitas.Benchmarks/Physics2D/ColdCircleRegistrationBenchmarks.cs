using BenchmarkDotNet.Attributes;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
[InvocationCount(1, 1)]
public class ColdCircleRegistrationBenchmarks
{
    private CirclePartitionBenchmarkScene _scene;

    [Params(64, 256, 1024)]
    public int ColliderCount { get; set; }

    [Params(1, 16)]
    public int CellSize { get; set; }

    [IterationSetup]
    public void Setup() => _scene = new CirclePartitionBenchmarkScene(ColliderCount, CellSize);

    [Benchmark]
    public int RegisterCirclePairs()
    {
        _scene.RegisterBodies();
        return _scene.Context.Physics2D.ColliderCount;
    }

    [IterationCleanup]
    public void Cleanup()
    {
        try
        {
            _scene.ValidateGeometry();
        }
        finally
        {
            _scene.Dispose();
        }
    }
}
