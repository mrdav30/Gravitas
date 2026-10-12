using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using System;
using System.Collections.Generic;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedMeshSurfaceContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSMeshCollider _mesh;
    private LSCollider2D _slab;
    private readonly ContactManifold _manifold = new();

    // Dense topology probes use the curved families whose exact finite sections
    // are most expensive. Ordinary geometry retains all four embedded shapes.
    public static IEnumerable<string> Fixtures => new[]
    {
        "Circle.Interior.2", "Stadium.Interior.2", "Box.Interior.2", "Polygon.Interior.2",
        "Circle.Rim.2", "Stadium.Rim.2", "Box.Rim.2", "Polygon.Rim.2",
        "Circle.Tilted.2", "Stadium.Tilted.2", "Box.Tilted.2", "Polygon.Tilted.2",
        "Circle.Interior.16", "Circle.Interior.32", "Stadium.Interior.16", "Stadium.Interior.32"
    };

    [ParamsSource(nameof(Fixtures))]
    public string Fixture { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        string[] parts = Fixture.Split('.');
        _context = BenchmarkPhysicsScene.CreateMixedContext(16, 16, clearAllPools: true);
        _mesh = CreateMesh(int.Parse(parts[2]), parts[1] == "Tilted");
        BenchmarkPhysicsScene.CreateStaticCollider(_context, _mesh, Vector3d.Zero);
        _slab = parts[0] switch
        {
            "Circle" => new LSCircleCollider2D(Fixed64.Half),
            "Stadium" => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.FromFraction(5, 4)),
            "Box" => new LSAABBoxCollider2D(Vector2d.One),
            _ => new LSPolygonCollider2D(new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half), new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half))
        };
        _slab.MixedHalfThicknessOverride = Fixed64.Half;
        Vector2d center = GetCenter(parts[1]);
        _slab.InitializeWithNoBody(new BenchmarkMatterAgent(_context,
            new Vector3d(center.X, -Fixed64.Quarter, center.Y)));
        if (!BuildManifold()) throw new InvalidOperationException("The finite mixed surface fixture must overlap.");
        if (parts[1] == "Interior" && (_manifold.GroupCount != 1 || _manifold.PrimaryContact.Normal != Vector3d.Down))
            throw new InvalidOperationException("The filled interior must retain one physical face region.");
        if (parts[1] == "Rim")
        {
            bool hasRim = false;
            foreach (ManifoldContact row in _manifold)
                hasRim |= row.Normal == Vector3d.Right && row.Depth == Fixed64.FromFraction(1, 8);
            if (!hasRim) throw new InvalidOperationException("The real outer rim must remain an admitted constraint.");
        }
        for (int i = 0; i < 16; i++) { _ = BuildManifold(); _ = ContactSummary(); }
    }

    [Benchmark]
    public bool ContactSummary() => CollisionDetectionMixed.TryCollide(_mesh, _slab, out _);

    [Benchmark]
    public bool BuildManifold() => CollisionDetectionMixed.TryBuildManifold(_mesh, _slab, _manifold);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    internal static Vector2d GetCenter(string fixture) => fixture switch
    {
        "Rim" => new Vector2d(Fixed64.FromFraction(19, 8), Fixed64.Zero),
        "Tilted" => Vector2d.Zero,
        _ => new Vector2d(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10))
    };

    internal static LSMeshCollider CreateMesh(int subdivision, bool tilted)
    {
        LSMeshCollider template = BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(subdivision, MeshColliderMode.Concave);
        Vector3d[] vertices = template.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++)
        {
            Fixed64 x = vertices[i].Y * (Fixed64)4;
            vertices[i] = new Vector3d(x, tilted ? -x : Fixed64.Zero, vertices[i].Z * (Fixed64)4);
        }
        return new LSMeshCollider(vertices, template.Mesh.Triangles.ToArray(),
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}

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
