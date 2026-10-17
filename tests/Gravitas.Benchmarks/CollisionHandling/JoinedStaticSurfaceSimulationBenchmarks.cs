using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using GridForge.Configuration;
using System;

namespace Gravitas.Benchmarks;

/// <summary>One moving rounded body crossing a joined or equivalent continuous frictionless floor.</summary>
[MemoryDiagnoser]
public class JoinedStaticSurfaceSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;

    [Params(false, true)]
    public bool Joined { get; set; }

    [Params(false, true)]
    public bool Capsule { get; set; }

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
        if (Joined) _context.Physics.CreateStaticSurface(new[] { Floor(-8, 0), Floor(0, 8) });
        else Floor(-8, 8);
        LSCollider collider = Capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSSphereCollider { Radius = Fixed64.Half };
        collider.Material = PhysicsMaterial.Frictionless;
        var start = new Vector3d((Fixed64)(-2), Capsule ? Fixed64.One : Fixed64.Half, Fixed64.Zero);
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

    private LSMeshCollider Floor(int left, int right)
    {
        var collider = new LSMeshCollider(new[] { new Vector3d(left, 0, -4), new(right, 0, -4),
            new(left, 0, 4), new(right, 0, 4) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        collider.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        return collider;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
