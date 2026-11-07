using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using GridForge.Configuration;

namespace Gravitas.Benchmarks;

/// <summary>Driven planar seam crossings; independent pieces are an unreduced diagnostic control.</summary>
[MemoryDiagnoser]
public class JoinedStaticSurface2DSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody2D _body;
    private Fixed64 _direction = Fixed64.One;

    [Params("Circle", "Capsule", "Polygon")]
    public string Shape { get; set; } = "Circle";

    [Params("Continuous", "Joined", "Joined16", "Joined64", "JoinedFar64", "ContinuousFar64", "Independent")]
    public string Layout { get; set; } = "Continuous";

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        _context.SetFrameRate(16);
        bool farMembers = Layout.EndsWith("Far64", StringComparison.Ordinal);
        _context.World.TryAddGrid(new GridConfiguration(new(-16, 0, -16), new((Fixed64)(farMembers ? 288 : 16), Fixed64.Zero, (Fixed64)16)), out _);
        _context.Environment.Gravity = Fixed64.One;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        int count = Layout == "Continuous" ? 1 : Layout == "Joined16" ? 16 : Layout == "Joined64" || farMembers ? 64 : 2;
        var members = new LSCollider2D[count];
        int localCount = farMembers ? Layout == "JoinedFar64" ? 2 : 1 : count;
        Fixed64 width = (Fixed64)16 / localCount;
        for (int i = 0; i < count; i++) members[i] = farMembers && i >= localCount
            ? Floor((Fixed64)(32 + 4 * (i - localCount)), Fixed64.Two)
            : Floor(-8 + width * ((Fixed64)i + Fixed64.Half), width);
        // Match loaded population independently of local floor subdivision.
        if (farMembers && !members[count - 1].IsPartitioned)
            throw new InvalidOperationException("Remote continuity members must be loaded in the benchmark world.");
        if (Layout.StartsWith("Joined", StringComparison.Ordinal)) _context.Physics2D.CreateStaticSurface(members);
        LSCollider2D shape = Shape == "Capsule" ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : Shape == "Polygon" ? new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half))
            : new LSCircleCollider2D(Fixed64.Half);
        shape.Material = PhysicsMaterial.Frictionless;
        _body = new SolidBody2D(new BenchmarkMatterAgent(_context, Vector3d.Zero), shape)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        _body.Initialize(new(-Fixed64.Two, Shape == "Capsule" ? Fixed64.One : Fixed64.Half));
        _body.UseManualGrounding();
        for (int frame = 0; frame < 32; frame++) { _context.Simulate(); _context.LateSimulate(); }
        for (int batch = 0; batch < 4; batch++) FullStep();
    }

    [Benchmark(OperationsPerInvoke = 64)]
    public void FullStep()
    {
        for (int frame = 0; frame < 64; frame++)
        {
            if (_body.Position.X >= Fixed64.Two) _direction = -Fixed64.One;
            else if (_body.Position.X <= -Fixed64.Two) _direction = Fixed64.One;
            // Identical host drive prevents the known independent-rim resistance
            // from turning its long-running diagnostic into a stationary workload.
            _body.AddLinearImpulse(Vector2d.Right * (_direction - _body.LinearVelocity.X));
            _context.Simulate(); _context.LateSimulate();
            if (Layout != "Independent" && _body.LinearVelocity.X != _direction)
                throw new InvalidOperationException("A frictionless planar join must preserve tangential velocity.");
        }
    }

    private LSCollider2D Floor(Fixed64 x, Fixed64 width)
    {
        var floor = new LSAABBoxCollider2D(new Vector2d(width, Fixed64.Two)) { Material = PhysicsMaterial.Frictionless };
        floor.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new(x, Fixed64.Zero, -Fixed64.One)));
        return floor;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
