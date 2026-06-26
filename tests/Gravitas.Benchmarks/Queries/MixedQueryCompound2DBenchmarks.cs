using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedQueryCompound2DBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _denseAabbContext;
    private GravitasWorldContext _falsePositiveAabbContext;
    private GravitasWorldContext _densePolygonContext;
    private GravitasWorldContext _falsePositivePolygonContext;
    private GravitasWorldContext _denseCompoundContext;
    private GravitasWorldContext _falsePositiveCompoundContext;
    private SwiftList<PhysicsMixedHit> _hits;
    private Vector3d _denseStart;
    private Vector3d _denseEnd;
    private Vector3d _falsePositiveStart;
    private Vector3d _falsePositiveEnd;

    [Params(64, 1024)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extentX = 16 + ColliderCount;
        _denseAabbContext = BenchmarkPhysicsScene.CreateMixedContext(extentX, 16);
        _falsePositiveAabbContext = BenchmarkPhysicsScene.CreateMixedContext(extentX, 16);
        _densePolygonContext = BenchmarkPhysicsScene.CreateMixedContext(extentX, 16);
        _falsePositivePolygonContext = BenchmarkPhysicsScene.CreateMixedContext(extentX, 16);
        _denseCompoundContext = BenchmarkPhysicsScene.CreateMixedContext(extentX, 16);
        _falsePositiveCompoundContext = BenchmarkPhysicsScene.CreateMixedContext(extentX, 16);
        _hits = new SwiftList<PhysicsMixedHit>(ColliderCount);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector3d position = new((Fixed64)i, Fixed64.Zero, Fixed64.Zero);
            _ = CreateAabb2D(_denseAabbContext, position);
            _ = CreateAabb2D(_falsePositiveAabbContext, position);
            _ = CreatePolygon2D(_densePolygonContext, position);
            _ = CreatePolygon2D(_falsePositivePolygonContext, position);
            _ = CreateCompound2D(_denseCompoundContext, position);
            _ = CreateCompound2D(_falsePositiveCompoundContext, position);
        }

        _denseAabbContext.Simulate();
        _falsePositiveAabbContext.Simulate();
        _densePolygonContext.Simulate();
        _falsePositivePolygonContext.Simulate();
        _denseCompoundContext.Simulate();
        _falsePositiveCompoundContext.Simulate();

        _denseStart = new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        _denseEnd = new Vector3d((Fixed64)ColliderCount, Fixed64.Zero, Fixed64.Zero);
        _falsePositiveStart = new Vector3d(
            (Fixed64)(-2),
            Fixed64.FromFraction(9, 10),
            Fixed64.FromFraction(7, 5));
        _falsePositiveEnd = new Vector3d(
            (Fixed64)ColliderCount,
            Fixed64.FromFraction(9, 10),
            Fixed64.FromFraction(7, 5));
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _denseAabbContext?.Dispose();
        _falsePositiveAabbContext?.Dispose();
        _densePolygonContext?.Dispose();
        _falsePositivePolygonContext?.Dispose();
        _denseCompoundContext?.Dispose();
        _falsePositiveCompoundContext?.Dispose();
        _denseAabbContext = null;
        _falsePositiveAabbContext = null;
        _densePolygonContext = null;
        _falsePositivePolygonContext = null;
        _denseCompoundContext = null;
        _falsePositiveCompoundContext = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_DenseAabbTargets()
    {
        return SweepSphereAgainst2D(_denseAabbContext, _denseStart, _denseEnd);
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_DenseAabbTargets_CandidateCount()
    {
        _ = SweepSphereAgainst2D(_denseAabbContext, _denseStart, _denseEnd);
        return _denseAabbContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_FalsePositiveAabbTargets()
    {
        return SweepSphereAgainst2D(_falsePositiveAabbContext, _falsePositiveStart, _falsePositiveEnd);
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_FalsePositiveAabbTargets_CandidateCount()
    {
        _ = SweepSphereAgainst2D(_falsePositiveAabbContext, _falsePositiveStart, _falsePositiveEnd);
        return _falsePositiveAabbContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_DensePolygonTargets()
    {
        return SweepSphereAgainst2D(_densePolygonContext, _denseStart, _denseEnd);
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_DensePolygonTargets_CandidateCount()
    {
        _ = SweepSphereAgainst2D(_densePolygonContext, _denseStart, _denseEnd);
        return _densePolygonContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_FalsePositivePolygonTargets()
    {
        return SweepSphereAgainst2D(_falsePositivePolygonContext, _falsePositiveStart, _falsePositiveEnd);
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_FalsePositivePolygonTargets_CandidateCount()
    {
        _ = SweepSphereAgainst2D(_falsePositivePolygonContext, _falsePositiveStart, _falsePositiveEnd);
        return _falsePositivePolygonContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_DenseCompoundTargets()
    {
        return SweepSphereAgainst2D(_denseCompoundContext, _denseStart, _denseEnd);
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_DenseCompoundTargets_CandidateCount()
    {
        _ = SweepSphereAgainst2D(_denseCompoundContext, _denseStart, _denseEnd);
        return _denseCompoundContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_FalsePositiveCompoundTargets()
    {
        return SweepSphereAgainst2D(_falsePositiveCompoundContext, _falsePositiveStart, _falsePositiveEnd);
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_FalsePositiveCompoundTargets_CandidateCount()
    {
        _ = SweepSphereAgainst2D(_falsePositiveCompoundContext, _falsePositiveStart, _falsePositiveEnd);
        return _falsePositiveCompoundContext.QueryMixed.LastQueryCandidateCount;
    }

    private int SweepSphereAgainst2D(GravitasWorldContext context, Vector3d start, Vector3d end)
    {
        return context.QueryMixed.SweepSphereAgainst2DAll(
            start,
            end,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    private static SolidBody2D CreateAabb2D(GravitasWorldContext context, Vector3d position)
    {
        return CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), position);
    }

    private static SolidBody2D CreatePolygon2D(GravitasWorldContext context, Vector3d position)
    {
        return CreateBody2D(context, new LSPolygonCollider2D(CreateDiamondVertices()), position);
    }

    private static SolidBody2D CreateCompound2D(GravitasWorldContext context, Vector3d position)
    {
        return CreateBody2D(
            context,
            new LSCompoundCollider2D(
                CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(-Fixed64.Half, Fixed64.Zero)),
                CompoundColliderPart2D.ConvexPolygon(
                    CreateDiamondVertices(),
                    new Vector2d(Fixed64.Half, Fixed64.Zero))),
            position);
    }

    private static SolidBody2D CreateBody2D(GravitasWorldContext context, LSCollider2D collider, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(new Vector2d(position.X, position.Z));
        return body;
    }

    private static Vector2d[] CreateDiamondVertices() =>
        new[]
        {
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero)
        };
}
