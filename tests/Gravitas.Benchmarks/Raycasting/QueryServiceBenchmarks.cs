using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Raycasting;
using Gravitas.Support;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class QueryServiceBenchmarks
{
    private static readonly SingleLayer IncludeLayerZero = new(0);

    private GravitasWorldContext _context;
    private GravitasWorldContext _overlappingContext;
    private Vector3d _rayStart;
    private Vector3d _rayEnd;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extent = BenchmarkPhysicsScene.GridExtentForLine(ColliderCount);

        _context = BenchmarkPhysicsScene.CreateContext(extent, clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_context, ColliderCount);

        _overlappingContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_overlappingContext, ColliderCount);

        _rayStart = new Vector3d((Fixed64)(-2), -Fixed64.Fraction(1, 4), Fixed64.Zero);
        _rayEnd = new Vector3d((Fixed64)(ColliderCount * 2), Fixed64.Fraction(1, 4), Fixed64.Zero);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _overlappingContext.Dispose();
        _context = null;
        _overlappingContext = null;
    }

    [Benchmark]
    public int RaycastAllAcrossPopulatedContext() =>
        CountRaycastHits(_context);

    [Benchmark]
    public int CircleCastAllAcrossPopulatedContext()
    {
        int count = 0;
        foreach (LSRaycastHit _ in _context.Circlecasts.CircleCastAll(Vector3d.Zero, (Fixed64)4, IncludeLayerZero))
            count++;

        return count;
    }

    [Benchmark]
    public int RaycastAcrossTwoOverlappingContexts() =>
        CountRaycastHits(_context) + CountRaycastHits(_overlappingContext);

    private int CountRaycastHits(GravitasWorldContext context)
    {
        int count = 0;
        foreach (LSRaycastHit _ in context.Raycasts.RaycastAll(_rayStart, _rayEnd, IncludeLayerZero))
            count++;

        return count;
    }
}
