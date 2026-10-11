using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ConvexMeshSphereContactBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;

    [Params(false, true)]
    public bool Capsule { get; set; }

    [Params("Contained", "Face", "Edge", "Separated")]
    public string Pose { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        var mesh = BenchmarkPhysicsScene.CreateDynamicConvexCube(_context, Vector3d.Zero);
        Fixed64 radius = Fixed64.Quarter;
        LSCollider curved = Capsule ? new LSCapsuleCollider { Radius = radius, Size = Vector3d.One * (radius * Fixed64.Two) }
            : new LSSphereCollider { Radius = radius };
        Vector3d center = Pose == "Contained" ? Vector3d.Right * Fixed64.Quarter
            : Pose == "Face" ? Vector3d.Right * Fixed64.FromFraction(5, 8)
            : Pose == "Edge" ? new(Fixed64.FromFraction(5, 8), Fixed64.FromFraction(5, 8), Fixed64.Zero)
            : Vector3d.Right;
        curved.InitializeWithNoBody(new BenchmarkMatterAgent(_context, center));
        _pair = new CollisionPair(mesh, curved);
        bool hit = CollisionDetection.DoCollisionCheck(_pair);
        if (hit != (Pose != "Separated")
            || (hit && (_pair.Manifold.Count != 1 || _pair.Manifold.PrimaryContact.Depth <= Fixed64.Zero))
            || (Pose == "Contained" && _pair.Manifold.PrimaryContact.Depth != Fixed64.Half)
            || (Pose == "Face" && _pair.Manifold.PrimaryContact.Depth != Fixed64.FromFraction(1, 8)))
            throw new InvalidOperationException("Convex volume fixtures must retain their whole-shape contacts.");
    }

    [Benchmark]
    public bool Contact() => CollisionDetection.DoCollisionCheck(_pair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
