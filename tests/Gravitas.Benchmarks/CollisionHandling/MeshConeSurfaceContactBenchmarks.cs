using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshConeSurfaceContactBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;

    [Params("QuadInterior", "QuadEdge", "SmallInterior", "SmallEdge", "SingleTriangle", "SubdividedInterior", "SubdividedTilted")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        bool tilted = Geometry == "SubdividedTilted";
        int subdivision = Geometry.StartsWith("Subdivided", StringComparison.Ordinal) ? 8 : 1;
        Fixed64 extent = Geometry == "SmallInterior" ? Fixed64.FromFraction(1, 5)
            : Geometry == "SmallEdge" ? Fixed64.FromFraction(3, 20) : Fixed64.Two;
        int width = subdivision + 1;
        var vertices = new Vector3d[width * width];
        var triangles = new int[subdivision * subdivision * 6];
        for (int z = 0; z < width; z++)
            for (int x = 0; x < width; x++)
                vertices[z * width + x] = new Vector3d(
                    extent * Fixed64.FromFraction(2 * x, subdivision) - extent,
                    Fixed64.Zero,
                    extent * Fixed64.FromFraction(2 * z, subdivision) - extent);

        int offset = 0;
        for (int z = 0; z < subdivision; z++)
            for (int x = 0; x < subdivision; x++)
            {
                int first = z * width + x;
                triangles[offset++] = first;
                triangles[offset++] = first + width;
                triangles[offset++] = first + 1;
                triangles[offset++] = first + 1;
                triangles[offset++] = first + width;
                triangles[offset++] = first + width + 1;
            }

        if (Geometry == "SingleTriangle")
        {
            Array.Resize(ref vertices, 3);
            triangles = new[] { 0, 2, 1 };
        }
        var mesh = new LSMeshCollider(vertices, triangles,
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        var cone = new LSConeCollider();
        var coneAgent = new BenchmarkMatterAgent(_context,
            new Vector3d(tilted ? -Fixed64.FromFraction(28, 25)
                : Geometry == "QuadEdge" ? Fixed64.Two : Fixed64.Zero,
                -Fixed64.Quarter, Fixed64.Zero));
        if (tilted)
            coneAgent.Transform.LocalRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
                Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        cone.InitializeWithNoBody(coneAgent);
        _pair = new CollisionPair(mesh, cone);
        if (!CollisionDetection.DoCollisionCheck(_pair))
            throw new InvalidOperationException("Surface contact benchmark must contain a real intersection.");
        ManifoldContact contact = _pair.Manifold.PrimaryContact;
        if (Geometry is "QuadEdge" or "SmallEdge" or "SingleTriangle")
        {
            if (contact.Depth <= Fixed64.Zero || contact.Depth >= Fixed64.Quarter || contact.Normal == Vector3d.Down)
                throw new InvalidOperationException("Exposed edge must retain its shorter curved-feature exit.");
        }
        else if (contact.Normal != Vector3d.Down || FixedMath.Abs(contact.Depth
            - (tilted ? Fixed64.FromFraction(9, 100) : Fixed64.Quarter)) > Fixed64.FromRaw(4))
            throw new InvalidOperationException("Interior contact must describe the complete surface exit.");
    }

    [Benchmark]
    public bool ConeSurface() => CollisionDetection.DoCollisionCheck(_pair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
