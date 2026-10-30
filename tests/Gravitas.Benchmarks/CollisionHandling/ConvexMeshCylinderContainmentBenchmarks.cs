using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;

namespace Gravitas.Benchmarks;

/// <summary>A complete host step with one cylinder/sphere and one closed convex mesh.</summary>
[MemoryDiagnoser]
public class ConvexMeshCylinderContainmentBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;
    private Vector3d _start;

    [Params("CylinderContained", "CylinderExterior", "CylinderMiss", "SphereContained")]
    public string Geometry { get; set; } = "CylinderContained";

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(8);
        _context.SetFrameRate(16);
        _context.Environment.Gravity = _context.Environment.AirDensity =
            _context.Environment.DampingFactor = _context.Environment.MinSpeed = Fixed64.Zero;
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(-1,-1,-1), new Vector3d(1,-1,-1), new Vector3d(-1,1,-1), new Vector3d(1,1,-1),
            new Vector3d(-1,-1,1), new Vector3d(1,-1,1), new Vector3d(-1,1,1), new Vector3d(1,1,1)
        }, new[] { 0,2,3, 0,3,1, 4,5,7, 4,7,6, 0,1,5, 0,5,4, 2,6,7, 2,7,3, 0,4,6, 0,6,2, 1,3,7, 1,7,5 }, MeshColliderMode.Convex)
        { Material = PhysicsMaterial.Frictionless };
        mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        _start = Geometry == "CylinderExterior" ? Vector3d.Right * (Fixed64)1.125m
            : Geometry == "CylinderMiss" ? new Vector3d((Fixed64)1.2m, Fixed64.Zero, (Fixed64)1.2m) : Vector3d.Zero;
        LSCollider collider = Geometry == "SphereContained" ? new LSSphereCollider { Radius = Fixed64.Quarter }
            : new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One * Fixed64.Half };
        collider.Material = PhysicsMaterial.Frictionless;
        _body = new SolidBody(new BenchmarkMatterAgent(_context, _start), collider)
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation, SleepEnabled = false };
        _body.Initialize(_start, FixedQuaternion.Identity);
        _body.UseManualGrounding();
        for (int i = 0; i < 16; i++) FullStep();
    }

    [Benchmark]
    public long FullStep()
    {
        _body.SetPosition(_start);
        _context.Simulate();
        _context.LateSimulate();
        Vector3d position = _body.Position3d;
        return position.X.m_rawValue ^ position.Y.m_rawValue ^ position.Z.m_rawValue;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
