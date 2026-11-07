using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;

namespace Gravitas.Benchmarks;

/// <summary>Pose refresh and finite rounded-solid contact generation at a concave join or oblique height step.</summary>
[MemoryDiagnoser]
public class JoinedStaticRoundedMixedContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSCollider _moving;
    private LSCollider2D[] _members;
    private readonly Gravitas.CollisionHandling.ContactManifold _manifold = new();
    private Vector3d _position;
    private bool _alternate;

    [Params("Capsule", "Cylinder", "Cone")]
    public string Shape { get; set; } = "Capsule";

    [Params("ConcaveL", "HeightStep")]
    public string Layout { get; set; } = "ConcaveL";

    [GlobalSetup]
    public void Setup()
    {
        bool step = Layout switch
        {
            "ConcaveL" => false,
            "HeightStep" => true,
            _ => throw new ArgumentException("Unknown joined rounded-solid layout.", nameof(Layout))
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
        _moving = Shape switch
        {
            "Capsule" => BenchmarkPhysicsScene.CreateDynamicCapsule(_context, _position, rotation),
            "Cylinder" => BenchmarkPhysicsScene.CreateDynamicCylinder(_context, _position, rotation),
            "Cone" => BenchmarkPhysicsScene.CreateDynamicCone(_context, _position, rotation),
            _ => throw new ArgumentException("Unknown joined rounded solid.", nameof(Shape))
        };
        _moving.Material = PhysicsMaterial.Frictionless;
        _alternate = false;
        for (int i = 0; i < 32; i++) BuildContacts();
    }

    [Benchmark]
    public int BuildContacts()
    {
        // Refresh a real pose each invocation; timing a stationary cached
        // manifold would omit the finite boundary work this fixture measures.
        _alternate = !_alternate;
        _moving.Body!.SetPosition(_position + Vector3d.Right * (_alternate ? Fixed64.MinIncrement : -Fixed64.MinIncrement));
        int count = 0;
        foreach (LSCollider2D member in _members)
        {
            Gravitas.CollisionHandling.CollisionDetectionMixed.TryBuildDiscreteManifold(_moving, member, _manifold);
            foreach (var row in _manifold)
                if (row.Normal == Vector3d.Zero || row.Depth < Fixed64.Zero || row.DepthIsClamped
                    || !row.AnchorA.TryGetWorldPoint(out _) || !row.AnchorB.TryGetWorldPoint(out _))
                    throw new InvalidOperationException("Joined rounded-solid contacts must retain finite physical response rows.");
            count += _manifold.Count;
        }
        if (count == 0) throw new InvalidOperationException("The joined rounded-solid fixture must keep its boundary contacts.");
        return count;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
