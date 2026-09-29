using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshConeContactBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;

    [Params("BaseFace", "SideFace", "ApexFace", "SideIntrusion", "ObliqueRim", "InteriorRim", "RimTouch", "RimGap", "UnrepresentableCenter")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        Vector3d offset = Vector3d.Zero;
        Vector3d coneCenter = Vector3d.Zero;
        Fixed64 radius = Fixed64.One;
        Fixed64 height = Fixed64.Two;
        Vector3d[] vertices;
        switch (Geometry)
        {
            case "BaseFace":
            case "ApexFace":
                Fixed64 y = Geometry == "BaseFace" ? Fixed64.FromFraction(-3, 4) : Fixed64.FromFraction(7, 8);
                vertices = new[]
                {
                    new Vector3d((Fixed64)(-16), y, (Fixed64)(-16)),
                    new Vector3d((Fixed64)16, y, (Fixed64)(-16)),
                    new Vector3d(Fixed64.Zero, y, (Fixed64)16)
                };
                break;
            case "SideFace":
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(3, 4), (Fixed64)(-16), (Fixed64)(-16)),
                    new Vector3d(Fixed64.FromFraction(3, 4), (Fixed64)16, (Fixed64)(-16)),
                    new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, (Fixed64)16)
                };
                break;
            case "SideIntrusion":
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, Fixed64.Zero),
                    new Vector3d(Fixed64.FromFraction(4, 5), (Fixed64)(-2), Fixed64.Zero),
                    new Vector3d(2, -2, 0)
                };
                break;
            case "ObliqueRim":
            case "RimTouch":
            case "RimGap":
                radius = (Fixed64)5;
                height = (Fixed64)10;
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)(-4), Fixed64.FromFraction(5, 4)),
                    new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(-11, 2), Fixed64.FromFraction(-5, 4)),
                    new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(-35, 4), Fixed64.Zero)
                };
                if (Geometry == "RimTouch")
                    offset = new Vector3d(Fixed64.FromFraction(3, 16), Fixed64.FromFraction(-1, 4), Fixed64.Zero);
                else if (Geometry == "RimGap")
                    offset = new Vector3d(Fixed64.FromFraction(3, 8), -Fixed64.Half, Fixed64.Zero);
                break;
            case "InteriorRim":
                radius = (Fixed64)5;
                height = (Fixed64)10;
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(-309, 64), Fixed64.FromFraction(231, 64)),
                    new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(-325, 64), Fixed64.FromFraction(279, 64)),
                    new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(-365, 64), Fixed64.FromFraction(271, 64))
                };
                break;
            case "UnrepresentableCenter":
                radius = Fixed64.MaxValue;
                height = Fixed64.One;
                vertices = new[] { new Vector3d(-3, -1, 0), new Vector3d(3, -1, 0), new Vector3d(0, 1, 0) };
                offset = new Vector3d(3, 0, 0);
                coneCenter = new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero, Fixed64.Zero);
                break;
            default:
                throw new InvalidOperationException("Unknown contact geometry.");
        }

        var mesh = new LSMeshCollider(vertices, new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, offset));
        var cone = new LSConeCollider { Radius = radius, Size = new Vector3d(Fixed64.One, height, Fixed64.One) };
        cone.InitializeWithNoBody(new BenchmarkMatterAgent(_context, coneCenter));
        _pair = new CollisionPair(mesh, cone);

        bool hit = CollisionDetection.DoCollisionCheck(_pair);
        if (hit != (Geometry != "RimGap"))
            throw new InvalidOperationException("Cone contact classification failed the benchmark preflight.");
        if (hit && Geometry != "UnrepresentableCenter")
        {
            Fixed64 depth = Geometry switch
            {
                "RimTouch" => Fixed64.Zero,
                "ObliqueRim" => Fixed64.FromFraction(5, 16),
                "InteriorRim" => Fixed64.FromFraction(13, 256),
                "ApexFace" => Fixed64.FromFraction(1, 8),
                "SideIntrusion" => Fixed64.One - Fixed64.FromFraction(4, 5),
                _ => Fixed64.FromFraction(1, 4)
            };
            if (_pair.Manifold.PrimaryContact.Depth != depth)
                throw new InvalidOperationException("Cone contact depth failed the benchmark preflight.");
        }
    }

    [Benchmark]
    public bool ConeTriangle() => CollisionDetection.DoCollisionCheck(_pair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
