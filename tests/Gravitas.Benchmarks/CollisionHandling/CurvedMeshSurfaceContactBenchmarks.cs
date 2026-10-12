using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Collections.Generic;

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

    [Params(false, true)]
    public bool MixedWinding { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) },
            MixedWinding ? new[] { 0, 2, 1, 3, 2, 1 } : new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave);
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

[MemoryDiagnoser]
public class CurvedMeshSurfaceRegionBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;

    [Params(1, 4, 8)]
    public int Regions { get; set; }

    [Params(false, true)]
    public bool Capsule { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        var vertices = new List<Vector3d>();
        var triangles = new List<int>();
        // A distant backbone connects authored tabs outside the shape section.
        // Each intersected tab therefore requires its own finite face group.
        for (int row = 0; row < 2; row++)
        for (int column = 0; column < 2 * Regions; column++)
            vertices.Add(new(Fixed64.FromFraction(2 * column - 2 * Regions + 1, 4 * Regions),
                Fixed64.Zero, (Fixed64)2 + row * Fixed64.Quarter));
        for (int column = 0; column < 2 * Regions - 1; column++)
            AddQuad(column, column + 1, column + 2 * Regions, column + 2 * Regions + 1);
        for (int tab = 0; tab < Regions; tab++)
        {
            int first = vertices.Count;
            vertices.Add(new(vertices[2 * tab].X, Fixed64.Zero, Fixed64.FromFraction(3, 8)));
            vertices.Add(new(vertices[2 * tab + 1].X, Fixed64.Zero, Fixed64.FromFraction(3, 8)));
            AddQuad(first, first + 1, 2 * tab, 2 * tab + 1);
        }
        var mesh = new LSMeshCollider(vertices.ToArray(), triangles.ToArray(), MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        int owner = mesh.Mesh.GetManifoldSurfaceOwner(0);
        for (int triangle = 1; triangle < mesh.Mesh.TriangleCount; triangle++)
            if (mesh.Mesh.GetManifoldSurfaceOwner(triangle) != owner)
                throw new System.InvalidOperationException("The authored comb must retain one connected surface owner.");
        LSCollider shape = Capsule ? new LSCapsuleCollider { Radius = Fixed64.One, Size = new(2, 3, 2) }
            : new LSSphereCollider { Radius = Fixed64.One };
        shape.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Up * Fixed64.Quarter));
        _pair = new CollisionPair(mesh, shape);
        if (!Contact()) throw new System.InvalidOperationException("Every tab must overlap.");
        int groups = 0;
        for (int group = 0; group < _pair.Manifold.GroupCount; group++)
            if (_pair.Manifold.GetGroup(group).Key.Region >= 0) groups++;
        if (groups != Regions) throw new System.InvalidOperationException("Every admitted tab must retain a face group.");

        void AddQuad(int a, int b, int c, int d) => triangles.AddRange(new[] { a, c, b, b, c, d });
    }

    [Benchmark]
    public bool Contact() => CollisionDetection.DoCollisionCheck(_pair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
