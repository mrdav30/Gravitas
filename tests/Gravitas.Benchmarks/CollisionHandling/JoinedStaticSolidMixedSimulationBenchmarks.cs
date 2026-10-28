using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Spatial bodies crossing continuous or joined static 2D solid walls through mixed response.</summary>
[MemoryDiagnoser]
public class JoinedStaticSolidMixedSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;
    private Fixed64 _direction;

    [Params("Box", "Hull", "Sphere", "PointCapsule", "Capsule", "HorizontalCapsule")]
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
                throw new InvalidOperationException("A mixed solid wall must preserve tangential motion and block inward penetration.");
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


/// <summary>Pose refresh and finite capsule contact generation at a concave join or oblique height step.</summary>
[MemoryDiagnoser]
public class JoinedStaticCapsuleMixedContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSCapsuleCollider _capsule;
    private LSCollider2D[] _members;
    private readonly Gravitas.CollisionHandling.ContactManifold _manifold = new();
    private Vector3d _position;
    private bool _alternate;

    [Params("ConcaveL", "HeightStep")]
    public string Layout { get; set; } = "ConcaveL";

    [GlobalSetup]
    public void Setup()
    {
        bool step = Layout switch
        {
            "ConcaveL" => false,
            "HeightStep" => true,
            _ => throw new ArgumentException("Unknown joined capsule layout.", nameof(Layout))
        };
        _context = BenchmarkPhysicsScene.CreateMixedContext(16, 8);
        var left = new LSAABBoxCollider2D(new Vector2d(2, step ? 2 : 4))
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
        var right = new LSAABBoxCollider2D(new Vector2d(2, 2))
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = step ? Fixed64.One : Fixed64.Two };
        left.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new Vector3d(-1, 0, step ? -1 : 0)));
        right.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new Vector3d(1, 0, -1)));
        _members = new LSCollider2D[] { left, right };
        _context.Physics2D.CreateStaticSurface(_members);
        _position = step ? new(Fixed64.Quarter, 3 * Fixed64.Half, -Fixed64.One)
            : new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter);
        FixedQuaternion rotation = step
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Two).Normalized
            : FixedQuaternion.Identity;
        _capsule = BenchmarkPhysicsScene.CreateDynamicCapsule(_context, _position, rotation);
        _capsule.Material = PhysicsMaterial.Frictionless;
        _alternate = false;
        for (int i = 0; i < 32; i++) BuildContacts();
    }

    [Benchmark]
    public int BuildContacts()
    {
        // Refresh a real pose each invocation; timing a stationary cached
        // manifold would omit the finite boundary work this fixture measures.
        _alternate = !_alternate;
        _capsule.Body!.SetPosition(_position + Vector3d.Right * (_alternate ? Fixed64.MinIncrement : -Fixed64.MinIncrement));
        int count = 0;
        foreach (LSCollider2D member in _members)
        {
            Gravitas.CollisionHandling.CollisionDetectionMixed.TryBuildDiscreteManifold(_capsule, member, _manifold);
            foreach (var row in _manifold)
                if (row.Normal == Vector3d.Zero || row.Depth < Fixed64.Zero || row.DepthIsClamped
                    || !row.AnchorA.TryGetWorldPoint(out _) || !row.AnchorB.TryGetWorldPoint(out _))
                    throw new InvalidOperationException("Joined capsule contacts must retain finite physical response rows.");
            count += _manifold.Count;
        }
        if (count == 0) throw new InvalidOperationException("The joined capsule fixture must keep its boundary contacts.");
        return count;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
