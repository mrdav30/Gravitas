using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using GridForge.Configuration;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Planar bodies crossing a vertical continuous or joined 3D wall through mixed response.</summary>
[MemoryDiagnoser]
public class JoinedStaticSurfaceMixedSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody2D _body;
    private Fixed64 _direction;

    [Params("Circle", "Capsule", "Polygon")]
    public string Shape { get; set; } = "Circle";

    [Params("Continuous", "Joined2", "Joined16", "Joined64")]
    public string Layout { get; set; } = "Continuous";

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        _context.SetFrameRate(16);
        _context.World.TryAddGrid(new GridConfiguration(new(-16, -16, -16), new(16, 16, 16)), out _);
        _context.Environment.Gravity = Fixed64.Zero;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        int count = Layout switch
        {
            "Continuous" => 1,
            "Joined2" => 2,
            "Joined16" => 16,
            "Joined64" => 64,
            _ => throw new ArgumentException("Unknown wall layout.", nameof(Layout))
        };
        var members = new LSMeshCollider[count];
        Fixed64 width = (Fixed64)16 / count;
        for (int i = 0; i < count; i++)
        {
            Fixed64 left = -8 + width * i, right = left + width;
            var member = new LSMeshCollider(new[]
            {
                new Vector3d(left, -Fixed64.Two, Fixed64.Zero), new(left, Fixed64.Two, Fixed64.Zero),
                new(right, -Fixed64.Two, Fixed64.Zero), new(right, Fixed64.Two, Fixed64.Zero)
            }, new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
            { Material = PhysicsMaterial.Frictionless };
            member.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
            members[i] = member;
        }
        if (count > 1) _context.Physics.CreateStaticSurface(members);
        LSCollider2D collider = Shape == "Capsule" ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : Shape == "Polygon" ? new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half))
            : new LSCircleCollider2D(Fixed64.Half);
        collider.Material = PhysicsMaterial.Frictionless;
        collider.MixedHalfThicknessOverride = Fixed64.Half;
        _body = new SolidBody2D(new BenchmarkMatterAgent(_context, new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero)), collider)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        _body.Initialize(new(-Fixed64.Two, Shape == "Capsule" ? Fixed64.One : Fixed64.Half));
        _body.UseManualGrounding();
        _direction = Fixed64.One;
        for (int frame = 0; frame < 32; frame++) Step();
        for (int batch = 0; batch < 4; batch++) FullStep();
    }

    [Benchmark(OperationsPerInvoke = 64)]
    public void FullStep()
    {
        for (int frame = 0; frame < 64; frame++)
        {
            if (_body.Position.X >= Fixed64.Two) _direction = -Fixed64.One;
            else if (_body.Position.X <= -Fixed64.Two) _direction = Fixed64.One;
            _body.AddLinearImpulse(Vector2d.Right * (_direction - _body.LinearVelocity.X));
            Step();
            if (_body.LinearVelocity.X != _direction)
                throw new InvalidOperationException("A frictionless mixed join must preserve tangential velocity.");
        }
    }

    private void Step()
    {
        // Inward planar motion requires real mixed response, unlike a horizontal floor observation.
        _body.AddLinearImpulse(new(Fixed64.Zero, -Fixed64.FromFraction(1, 16)));
        _context.Simulate();
        _context.LateSimulate();
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
