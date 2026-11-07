using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedMeshSurfaceInitialQueryBenchmarks
{
    private GravitasWorldContext _context;
    private Vector2d _start;
    private readonly ContactManifold _manifold = new();

    [Params("Interior.2", "Rim.2", "Tilted.2", "Interior.16", "Interior.32")]
    public string Fixture { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        string[] parts = Fixture.Split('.');
        _context = BenchmarkPhysicsScene.CreateMixedContext(16, 16, clearAllPools: true);
        LSMeshCollider mesh = MixedMeshSurfaceContactBenchmarks.CreateMesh(int.Parse(parts[1]), parts[0] == "Tilted");
        BenchmarkPhysicsScene.CreateStaticCollider(_context, mesh, Vector3d.Zero);
        _start = MixedMeshSurfaceContactBenchmarks.GetCenter(parts[0]);
        // Validate through the same finite source geometry before timing public
        // query admission, so a cheap missed fixture cannot appear as a speedup.
        var source = new LSCircleCollider2D(Fixed64.Half) { MixedHalfThicknessOverride = Fixed64.Half };
        source.InitializeWithNoBody(new BenchmarkMatterAgent(_context,
            new Vector3d(_start.X, -Fixed64.Quarter, _start.Y)));
        _context.Simulate();
        if (!CollisionDetectionMixed.TryBuildManifold(mesh, source, _manifold) || !InitialCircleQuery())
            throw new InvalidOperationException("The query fixture must have a finite initial overlap.");
        for (int i = 0; i < 16; i++) _ = InitialCircleQuery();
    }

    [Benchmark]
    public bool InitialCircleQuery() => _context.QueryMixed.SweepCircleAgainst3D(
        _start, _start + Vector2d.Left, Fixed64.Half, -Fixed64.Quarter, Fixed64.Half,
        PhysicsLayerMask.FromLayer(0), out _);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
