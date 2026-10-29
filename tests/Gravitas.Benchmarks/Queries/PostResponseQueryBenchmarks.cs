using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Support;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Complete sliding-contact steps with optional post-response queries.</summary>
[MemoryDiagnoser]
public class PostResponseQueryBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;
    private Fixed64 _direction;

    [Params(false, true)]
    public bool Mixed { get; set; }

    [Params("Capsule", "Compound")]
    public string Shape { get; set; } = "Capsule";

    [Params(false, true)]
    public bool Query { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(16);
        _context.Settings.RuntimeMode = Mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD;
        _context.Settings.Mixed2DHalfThickness = (Fixed64)2;
        _context.SetFrameRate(16);
        _context.Environment.Gravity = Fixed64.Zero;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        var wallAgent = new BenchmarkMatterAgent(_context, new Vector3d(0, 0, -1));
        if (Mixed)
            new LSAABBoxCollider2D(new Vector2d(16, 2)) { Material = PhysicsMaterial.Frictionless }
                .InitializeWithNoBody(wallAgent);
        else
            new LSCuboidCollider { Size = new(16, 4, 2), Material = PhysicsMaterial.Frictionless }
                .InitializeWithNoBody(wallAgent);

        Vector3d start = new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half);
        LSCollider collider;
        if (Shape == "Capsule")
            collider = BenchmarkPhysicsScene.CreateDynamicCapsule(_context, start, FixedQuaternion.Identity);
        else
        {
            collider = new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Left * Fixed64.Half),
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right * Fixed64.Half));
            var body = new SolidBody(new BenchmarkMatterAgent(_context, start), collider) { Mass = Fixed64.One };
            body.Initialize(start, FixedQuaternion.Identity);
        }
        collider.Material = PhysicsMaterial.Frictionless;
        _body = collider.Body;
        // Keep compound rotation enabled to include ordinary inertia ownership.
        _body.FreezeAxes = Shape == "Capsule"
            ? BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation : BodyFreezeAxes3D.PositionY;
        _body.SleepEnabled = false;
        _body.UseManualGrounding();
        _direction = Fixed64.One;
        for (int i = 0; i < 16; i++) FullStep();
    }

    [Benchmark(OperationsPerInvoke = 64)]
    public long FullStep()
    {
        long result = 0;
        for (int frame = 0; frame < 64; frame++)
        {
            if (_body.Position3d.X >= Fixed64.Two) _direction = -Fixed64.One;
            else if (_body.Position3d.X <= -Fixed64.Two) _direction = Fixed64.One;
            _body.AddLinearImpulse(Vector3d.Right * (_direction - _body.LinearVelocity.X)
                + Vector3d.Backward * Fixed64.FromFraction(1, 16));
            _context.Simulate();
            _context.LateSimulate();
            if (Query)
            {
                Vector3d position = _body.Position3d;
                if (!_context.Query3D.Raycast(new Vector3d(position.X, position.Y, (Fixed64)2),
                        Vector3d.Backward, (Fixed64)4, out var hit, PhysicsLayerMask.All))
                    throw new InvalidOperationException("The post-step ray must hit the sliding body.");
                result ^= hit.Distance.m_rawValue;
            }
            result ^= _body.Position3d.Z.m_rawValue;
        }
        return result;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
