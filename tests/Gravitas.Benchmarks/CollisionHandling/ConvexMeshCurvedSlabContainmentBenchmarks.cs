using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ConvexMeshCurvedSlabContainmentBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody2D _body;
    private Vector2d _start;

    [Params("CircleContained", "ZeroCoreContained", "CapsuleContained", "YawedCapsuleContained",
        "AffineCapsuleContained", "AffineYawedCapsuleContained", "CircleExterior", "CapsuleExterior", "CapsuleMiss")]
    public string Geometry { get; set; } = "CircleContained";

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(8);
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        _context.SetFrameRate(16);
        _context.Environment.Gravity = _context.Environment.AirDensity =
            _context.Environment.DampingFactor = _context.Environment.MinSpeed = Fixed64.Zero;
        bool affine = Geometry.StartsWith("Affine");
        var vertices = new[]
        {
            new Vector3d(-1,-1,-1), new Vector3d(1,-1,-1), new Vector3d(-1,1,-1), new Vector3d(1,1,-1),
            new Vector3d(-1,-1,1), new Vector3d(1,-1,1), new Vector3d(-1,1,1), new Vector3d(1,1,1)
        };
        if (affine)
            for (int index = 0; index < vertices.Length; index++) vertices[index] *= Fixed64.One + Fixed64.MinIncrement;
        var mesh = new LSMeshCollider(vertices, new[] { 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5 }, MeshColliderMode.Convex)
        { Material = PhysicsMaterial.Frictionless };
        var meshAgent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        if (affine) meshAgent.Transform.LocalScale = Vector3d.One * ((Fixed64)1.5m + Fixed64.MinIncrement);
        mesh.InitializeWithNoBody(meshAgent);
        if (affine && !mesh.Mesh.GeometryVertices.SequenceEqual(mesh.Mesh.LocalVertices))
            throw new InvalidOperationException("Affine fixtures must retain their exact authored geometry chart.");
        _start = Geometry.EndsWith("Exterior") ? Vector2d.Right * (Fixed64)1.125m
            : Geometry.EndsWith("Miss") ? new Vector2d((Fixed64)1.3m, (Fixed64)1.6m)
            : Vector2d.Right * Fixed64.Half;
        LSCollider2D slab = Geometry.StartsWith("Circle") ? new LSCircleCollider2D(Fixed64.Quarter)
            : new LSCapsuleCollider2D(Fixed64.Quarter, Geometry == "ZeroCoreContained" ? Fixed64.Half : Fixed64.One);
        slab.MixedHalfThicknessOverride = Fixed64.Quarter;
        slab.Material = PhysicsMaterial.Frictionless;
        _body = new SolidBody2D(new BenchmarkMatterAgent(_context, Vector3d.Zero), slab)
        { Mass = Fixed64.One, SleepEnabled = false, FreezeAxes = BodyFreezeAxes2D.Rotation };
        _body.Initialize(_start, Geometry.Contains("Yawed") ? (Fixed64)0.4m : Fixed64.Zero);
        _body.UseManualGrounding();
        for (int i = 0; i < 16; i++) FullStep();
    }

    [Benchmark]
    public long FullStep()
    {
        _body.SetPosition(_start);
        _context.Simulate();
        _context.LateSimulate();
        return _body.Position.X.m_rawValue ^ _body.Position.Y.m_rawValue;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
