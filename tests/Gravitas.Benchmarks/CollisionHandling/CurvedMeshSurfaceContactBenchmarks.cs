using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CurvedMeshSurfaceContactBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;

    [Params(false, true)]
    public bool Capsule { get; set; }

    [Params(false, true)]
    public bool ExposedEdge { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave);
        mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        LSCollider curved = Capsule ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSSphereCollider { Radius = Fixed64.Half };
        Vector3d center = ExposedEdge ? new((Fixed64)2 + Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero)
            : new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10));
        curved.InitializeWithNoBody(new BenchmarkMatterAgent(_context, center));
        _pair = new CollisionPair(mesh, curved);
        if (!CollisionDetection.DoCollisionCheck(_pair))
            throw new System.InvalidOperationException("The curved surface fixture must overlap.");
    }

    [Benchmark]
    public bool Contact() => CollisionDetection.DoCollisionCheck(_pair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
