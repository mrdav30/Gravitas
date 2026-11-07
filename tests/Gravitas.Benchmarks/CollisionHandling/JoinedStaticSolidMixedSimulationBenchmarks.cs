using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;

namespace Gravitas.Benchmarks;

/// <summary>Spatial bodies crossing continuous or joined static 2D solid walls through mixed response.</summary>
[MemoryDiagnoser]
public class JoinedStaticSolidMixedSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;
    private Fixed64 _direction;

    [Params("Box", "Hull", "Sphere", "PointCapsule", "Capsule", "HorizontalCapsule", "Cylinder", "Cone", "OpenMesh", "ConcaveMesh")]
    public string Shape { get; set; } = "Box";

    [Params("Continuous", "Joined2", "Joined16", "Joined64")]
    public string Layout { get; set; } = "Continuous";

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateMixedContext(16, 8);
        _context.SetFrameRate(16);
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
        var members = new LSCollider2D[count];
        Fixed64 width = (Fixed64)16 / count;
        for (int i = 0; i < count; i++)
        {
            Fixed64 left = -8 + width * i;
            var member = new LSAABBoxCollider2D(new Vector2d(width, Fixed64.Two))
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
            member.InitializeWithNoBody(new BenchmarkMatterAgent(_context,
                new Vector3d(left + width * Fixed64.Half, Fixed64.Zero, -Fixed64.One)));
            members[i] = member;
        }
        if (count > 1) _context.Physics2D.CreateStaticSurface(members);
        Vector3d start = new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half);
        LSCollider collider = Shape switch
        {
            "Box" => BenchmarkPhysicsScene.CreateDynamicCuboid(_context, start),
            "Hull" => BenchmarkPhysicsScene.CreateDynamicConvexCube(_context, start),
            "Sphere" => CreateRounded(new LSSphereCollider { Radius = Fixed64.Half }),
            "PointCapsule" => CreateRounded(new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One }),
            "Capsule" => CreateRounded(new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 3, 1) }),
            "HorizontalCapsule" => CreateRounded(new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 3, 1) }, true),
            "Cylinder" => CreateRounded(new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }),
            "Cone" => CreateRounded(new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }),
            "OpenMesh" => CreateRounded(new LSMeshCollider(new[] { new Vector3d(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
                new(Fixed64.Half, Fixed64.Zero, -Fixed64.Half), new(-Fixed64.Half, Fixed64.Zero, Fixed64.Half),
                new(Fixed64.Half, Fixed64.Zero, Fixed64.Half) }, new[] { 0, 2, 1, 1, 2, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)),
            "ConcaveMesh" => CreateRounded(new LSMeshCollider(new[] { new Vector3d(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
                new(Fixed64.Half, Fixed64.Zero, -Fixed64.Half), new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), Vector3d.Zero,
                new(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new(-Fixed64.Half, Fixed64.Zero, Fixed64.Half) },
                new[] { 0, 3, 1, 1, 3, 2, 0, 5, 3, 3, 5, 4 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)),
            _ => throw new ArgumentException("Unknown moving shape.", nameof(Shape))
        };
        collider.Material = PhysicsMaterial.Frictionless;
        _body = collider.Body!;
        _body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        _body.SleepEnabled = false;
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
            if (_body.Position3d.X >= Fixed64.Two) _direction = -Fixed64.One;
            else if (_body.Position3d.X <= -Fixed64.Two) _direction = Fixed64.One;
            _body.AddLinearImpulse(Vector3d.Right * (_direction - _body.LinearVelocity.X));
            Step();
            if (_body.LinearVelocity.X != _direction || _body.Position3d.Z <= Fixed64.Zero)
                throw new InvalidOperationException($"A mixed solid wall must preserve tangential motion and block inward penetration: {Shape}/{Layout}, frame {frame}, position {_body.Position3d}, velocity {_body.LinearVelocity}, direction {_direction}.");
        }
    }

    private LSCollider CreateRounded(LSCollider collider, bool horizontal = false)
    {
        var body = new SolidBody(new BenchmarkMatterAgent(_context, new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half)), collider) { Mass = Fixed64.One };
        body.Initialize(new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half), horizontal
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized : FixedQuaternion.Identity);
        return collider;
    }

    private void Step()
    {
        _body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
        _context.Simulate();
        _context.LateSimulate();
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
