using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedMeshTriangleScalingBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _denseContext;
    private GravitasWorldContext _falsePositiveContext;
    private SwiftList<PhysicsMixedHit> _hits;
    private Vector2d _start;
    private Vector2d _end;

    [Params(8, 16, 32)]
    public int Subdivision { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int triangleCount = Subdivision * Subdivision * 2;
        _denseContext = BenchmarkPhysicsScene.CreateMixedContext(8, 4, clearAllPools: true);
        _falsePositiveContext = BenchmarkPhysicsScene.CreateMixedContext(8, 4);
        BenchmarkPhysicsScene.CreateStaticCollider(
            _denseContext,
            BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(Subdivision),
            Vector3d.Zero);
        BenchmarkPhysicsScene.CreateStaticCollider(
            _falsePositiveContext,
            BenchmarkPhysicsScene.CreateRepeatedSlabClippedProxyOnlyTriangleMesh(triangleCount),
            Vector3d.Zero);
        _denseContext.Simulate();
        _falsePositiveContext.Simulate();

        _hits = new SwiftList<PhysicsMixedHit>(1);
        _start = new Vector2d((Fixed64)(-3), Fixed64.Zero);
        _end = new Vector2d((Fixed64)3, Fixed64.Zero);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _denseContext?.Dispose();
        _falsePositiveContext?.Dispose();
        _denseContext = null;
        _falsePositiveContext = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseTriangleMeshTarget()
    {
        return SweepCircleAgainst3D(_denseContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseTriangleMeshTarget_ColliderCandidateCount()
    {
        _ = SweepCircleAgainst3D(_denseContext);
        return _denseContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseTriangleMeshTarget_TriangleCandidateCount()
    {
        _ = SweepCircleAgainst3D(_denseContext);
        return _denseContext.QueryMixed.LastMeshTriangleCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveTriangleMeshTarget()
    {
        return SweepCircleAgainst3D(_falsePositiveContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveTriangleMeshTarget_ColliderCandidateCount()
    {
        _ = SweepCircleAgainst3D(_falsePositiveContext);
        return _falsePositiveContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveTriangleMeshTarget_TriangleCandidateCount()
    {
        _ = SweepCircleAgainst3D(_falsePositiveContext);
        return _falsePositiveContext.QueryMixed.LastMeshTriangleCandidateCount;
    }

    private int SweepCircleAgainst3D(GravitasWorldContext context)
    {
        return context.QueryMixed.SweepCircleAgainst3DAll(
            _start,
            _end,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }
}
