using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class QueryServiceBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _context;
    private GravitasWorldContext _overlappingContext;
    private Vector3d _rayStart;
    private Vector3d _rayEnd;
    private SwiftList<Physics3DHit> _raycastHits;
    private SwiftList<Physics3DHit> _overlappingRaycastHits;
    private SwiftList<Physics3DHit> _circlecastHits;
    private SwiftList<Physics3DHit> _sweepSphereHits;

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

        _rayStart = new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        _rayEnd = new Vector3d((Fixed64)(ColliderCount * 2), Fixed64.Zero, Fixed64.Zero);
        _raycastHits = new SwiftList<Physics3DHit>(ColliderCount);
        _overlappingRaycastHits = new SwiftList<Physics3DHit>(ColliderCount);
        _circlecastHits = new SwiftList<Physics3DHit>(ColliderCount);
        _sweepSphereHits = new SwiftList<Physics3DHit>(ColliderCount);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _overlappingContext.Dispose();
        _context = null;
        _overlappingContext = null;
        _raycastHits = null;
        _overlappingRaycastHits = null;
        _circlecastHits = null;
        _sweepSphereHits = null;
    }

    [Benchmark]
    public int RaycastAllAcrossPopulatedContext() =>
        CountRaycastHits(_context, _raycastHits);

    [Benchmark]
    public int OverlapCircleAllAcrossPopulatedContext() =>
        _context.Query3D.OverlapCircleAll(Vector3d.Zero, (Fixed64)4, IncludeLayerZero, _circlecastHits);

    [Benchmark]
    public bool DirectionalOverlapCircleAcrossPopulatedContext() =>
        _context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)4,
            Vector3d.Right,
            out _,
            (Fixed64)(ColliderCount * 2),
            IncludeLayerZero);

    [Benchmark]
    public int RaycastAcrossTwoOverlappingContexts() =>
        CountRaycastHits(_context, _raycastHits)
        + CountRaycastHits(_overlappingContext, _overlappingRaycastHits);

    [Benchmark]
    public int SweepSphereAllAcrossPopulatedContext() =>
        _context.Query3D.SweepSphereAll(_rayStart, _rayEnd, Fixed64.Half, IncludeLayerZero, _sweepSphereHits);

    private int CountRaycastHits(GravitasWorldContext context, SwiftList<Physics3DHit> results)
    {
        return context.Query3D.RaycastAll(_rayStart, _rayEnd, IncludeLayerZero, results);
    }
}
