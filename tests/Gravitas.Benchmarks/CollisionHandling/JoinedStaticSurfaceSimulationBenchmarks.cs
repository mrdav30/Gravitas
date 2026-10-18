using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using GridForge.Configuration;
using System;

namespace Gravitas.Benchmarks;

/// <summary>One moving body crossing a joined or equivalent continuous frictionless floor.</summary>
[MemoryDiagnoser]
public class JoinedStaticSurfaceSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;
    private LSMeshCollider[] _members;
    private StaticSurface _surface;

    [Params(false, true)]
    public bool Joined { get; set; }

    [Params("Sphere", "Capsule", "Cone", "Cuboid", "Cylinder")]
    public string Shape { get; set; } = "Sphere";

    [Params("Small", "Partial", "Dense", "Members")]
    public string Workload { get; set; } = "Small";

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.World.TryAddGrid(new GridConfiguration(new Vector3d(-16, -16, -16), new Vector3d(16, 16, 16)), out _);
        _context.SetFrameRate(16);
        _context.Environment.Gravity = Fixed64.One;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        int count = Joined ? Workload == "Members" ? 16 : 2 : 1;
        _members = new LSMeshCollider[count];
        for (int i = 0; i < count; i++)
            _members[i] = Floor(-8 + 16 * i / count, -8 + 16 * (i + 1) / count,
                Workload == "Partial" && i == 1 ? 2 : 4, Workload == "Dense" ? 32 : 1);
        if (Joined) _surface = _context.Physics.CreateStaticSurface(_members);
        LSCollider collider = Shape switch
        {
            "Capsule" => new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) },
            "Cone" => new LSConeCollider(),
            "Cuboid" => new LSCuboidCollider(),
            "Cylinder" => new LSCylinderCollider(),
            _ => new LSSphereCollider { Radius = Fixed64.Half }
        };
        collider.Material = PhysicsMaterial.Frictionless;
        var start = new Vector3d((Fixed64)(-2), Shape == "Capsule" ? Fixed64.One : Fixed64.Half, Fixed64.Zero);
        _body = new SolidBody(new BenchmarkMatterAgent(_context, start), collider)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation, SleepEnabled = false };
        _body.Initialize(start, FixedQuaternion.Identity, BodyMotionType.Dynamic);
        _body.UseManualGrounding();
        for (int step = 0; step < 32; step++) { _context.Simulate(); _context.LateSimulate(); }
        _body.AddLinearImpulse(Vector3d.Right);
        for (int batch = 0; batch < 4; batch++) FullStep();
    }

    [Benchmark(OperationsPerInvoke = 64)]
    public void FullStep()
    {
        for (int frame = 0; frame < 64; frame++)
        {
            if (_body.Position3d.X >= Fixed64.Two && _body.LinearVelocity.X > Fixed64.Zero)
                _body.AddLinearImpulse(-Vector3d.Right * Fixed64.Two);
            else if (_body.Position3d.X <= -Fixed64.Two && _body.LinearVelocity.X < Fixed64.Zero)
                _body.AddLinearImpulse(Vector3d.Right * Fixed64.Two);
            _context.Simulate();
            _context.LateSimulate();
            if (FixedMath.Abs(_body.LinearVelocity.X) != Fixed64.One)
                throw new InvalidOperationException("A frictionless shared edge must preserve tangential velocity.");
        }
    }

    // Cold declaration cost includes ownership, exact boundary preparation and
    // invalidation; it is deliberately separate from warmed simulation timing.
    [Benchmark]
    public void PrepareDeclaration()
    {
        _surface?.Dispose();
        _surface = _context.Physics.CreateStaticSurface(_members);
    }

    private LSMeshCollider Floor(int left, int right, int halfWidth, int divisions)
    {
        var vertices = new Vector3d[2 * (divisions + 1)];
        var indices = new int[6 * divisions];
        for (int i = 0; i <= divisions; i++)
        {
            Fixed64 x = left + (Fixed64)(right - left) * i / divisions;
            vertices[2 * i] = new(x, Fixed64.Zero, (Fixed64)(-halfWidth));
            vertices[2 * i + 1] = new(x, Fixed64.Zero, (Fixed64)halfWidth);
            if (i == divisions) continue;
            int v = 2 * i, t = 6 * i;
            indices[t] = v; indices[t + 1] = v + 1; indices[t + 2] = v + 2;
            indices[t + 3] = v + 2; indices[t + 4] = v + 1; indices[t + 5] = v + 3;
        }
        var collider = new LSMeshCollider(vertices, indices,
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        collider.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        return collider;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
