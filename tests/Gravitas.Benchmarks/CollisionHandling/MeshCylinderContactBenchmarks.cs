using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshCylinderContactBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;
    private LSMeshCollider _mesh;
    private LSCircleCollider2D _circle;

    [Params("CapFace", "FilledPatch", "SideFace", "CapIntrusion", "InteriorRim", "ObliqueRim", "RimTouch", "RimGap")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Vector3d offset = Vector3d.Zero;
        Vector3d[] vertices;
        switch (Geometry)
        {
            case "FilledPatch":
                vertices = new[]
                {
                    new Vector3d(-1, 0, -1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, -Fixed64.One),
                    new Vector3d(-1, 0, 1), new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.One)
                };
                break;
            case "CapFace":
                vertices = new[]
                {
                    new Vector3d((Fixed64)(-16), Fixed64.FromFraction(19, 4), (Fixed64)(-16)),
                    new Vector3d((Fixed64)16, Fixed64.FromFraction(19, 4), (Fixed64)(-16)),
                    new Vector3d(Fixed64.Zero, Fixed64.FromFraction(19, 4), (Fixed64)16)
                };
                break;
            case "SideFace":
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(19, 4), (Fixed64)(-16), (Fixed64)(-16)),
                    new Vector3d(Fixed64.FromFraction(19, 4), (Fixed64)16, (Fixed64)(-16)),
                    new Vector3d(Fixed64.FromFraction(19, 4), Fixed64.Zero, (Fixed64)16)
                };
                break;
            case "CapIntrusion":
                vertices = new[] { new Vector3d(0, 7, -5), new Vector3d(10, 1, -5), new Vector3d(5, 4, 5) };
                break;
            case "InteriorRim":
                // The exact stationary rim root wins here, unlike the analytic
                // winners in the original intrusion/oblique/touch fixtures.
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(309, 64), Fixed64.FromFraction(231, 64)),
                    new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(325, 64), Fixed64.FromFraction(279, 64)),
                    new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(365, 64), Fixed64.FromFraction(271, 64))
                };
                break;
            case "ObliqueRim":
            case "RimTouch":
            case "RimGap":
                vertices = new[]
                {
                    new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)4, Fixed64.FromFraction(5, 4)),
                    new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(11, 2), Fixed64.FromFraction(-5, 4)),
                    new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(35, 4), Fixed64.Zero)
                };
                if (Geometry == "RimTouch")
                    offset = new Vector3d(Fixed64.FromFraction(3, 16), Fixed64.FromFraction(1, 4), Fixed64.Zero);
                else if (Geometry == "RimGap")
                    offset = new Vector3d(Fixed64.FromFraction(3, 8), Fixed64.Half, Fixed64.Zero);
                break;
            default:
                throw new InvalidOperationException("Unknown contact geometry.");
        }

        bool patch = Geometry == "FilledPatch";
        _mesh = new LSMeshCollider(vertices, patch ? new[] { 0, 2, 1, 1, 2, 3 } : new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        _mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, offset));
        Fixed64 radius = patch ? Fixed64.Quarter : (Fixed64)5;
        Fixed64 height = patch ? Fixed64.One : (Fixed64)10;
        Vector3d center = patch ? Vector3d.Up * Fixed64.FromFraction(1, 10) : Vector3d.Zero;
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(radius, height));
        cylinder.InitializeWithNoBody(new BenchmarkMatterAgent(_context, center));
        _pair = new CollisionPair(_mesh, cylinder);
        _circle = new LSCircleCollider2D(radius) { MixedHalfThicknessOverride = height * Fixed64.Half };
        _circle.InitializeWithNoBody(new BenchmarkMatterAgent(_context, center));

        bool expected = Geometry != "RimGap";
        bool cylinderHit = CollisionDetection.DoCollisionCheck(_pair);
        bool circleHit = CollisionDetectionMixed.TryCollide(_mesh, _circle, out MixedContact mixed);
        if (cylinderHit != expected || circleHit != expected)
            throw new InvalidOperationException("Contact classification failed the benchmark preflight.");
        if (expected && Geometry != "CapIntrusion")
        {
            Fixed64 depth = patch ? Fixed64.Half - center.Y : Geometry == "RimTouch" ? Fixed64.Zero
                : Geometry == "InteriorRim" ? Fixed64.FromFraction(13, 256)
                : Geometry == "ObliqueRim" ? Fixed64.FromFraction(5, 16) : Fixed64.FromFraction(1, 4);
            if (_pair.Manifold.PrimaryContact.Depth != depth || mixed.Depth != depth)
                throw new InvalidOperationException("Contact depth failed the benchmark preflight.");
            if (patch && (_pair.Manifold.PrimaryContact.Normal != Vector3d.Up || mixed.Normal3DTo2D != Vector3d.Up))
                throw new InvalidOperationException("A covered diagonal cannot replace the filled patch's vertical exit.");
        }
    }

    [Benchmark]
    public bool CylinderTriangle() => CollisionDetection.DoCollisionCheck(_pair);

    [Benchmark]
    public bool CircleSlabTriangle() => CollisionDetectionMixed.TryCollide(_mesh, _circle, out _);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
