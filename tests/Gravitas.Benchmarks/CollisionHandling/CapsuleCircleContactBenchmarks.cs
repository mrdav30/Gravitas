using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CapsuleCircleContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSCapsuleCollider _capsule;
    private LSCircleCollider2D _circle;
    private CollisionPair _cylinderPair;

    [Params("Side", "Cap", "EndpointRim", "EndpointGap", "ObliqueRim", "ZeroCore")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 radius = Fixed64.One;
        Fixed64 height = (Fixed64)4;
        Fixed64 circleRadius = Fixed64.One;
        Vector3d center = new(Fixed64.FromFraction(7, 4), Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.Identity;
        switch (Geometry)
        {
            case "Side":
                break;
            case "Cap":
                center = new Vector3d(Fixed64.Zero, Fixed64.FromFraction(11, 4), Fixed64.Zero);
                break;
            case "ZeroCore":
                height = Fixed64.Two;
                break;
            case "EndpointRim":
            case "EndpointGap":
                Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
                rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -q, q);
                height = (Fixed64)22;
                circleRadius = (Fixed64)10;
                Fixed64 offset = Geometry == "EndpointRim" ? Fixed64.Half : Fixed64.FromFraction(3, 4);
                center = new Vector3d((Fixed64)20 + offset, Fixed64.One + offset, Fixed64.Zero);
                break;
            case "ObliqueRim":
                const long scale = 607_400_100L;
                rotation = new FixedQuaternion(Fixed64.FromRaw(5 * scale), Fixed64.Zero,
                    Fixed64.FromRaw(-3 * scale), Fixed64.FromRaw(4 * scale));
                radius = Fixed64.FromFraction(21, 4);
                height = radius * Fixed64.Two + Fixed64.Two;
                center = new Vector3d(4, 5, 0);
                break;
            default:
                throw new InvalidOperationException("Unknown contact geometry.");
        }
        _capsule = new LSCapsuleCollider
        {
            Radius = radius,
            Size = new Vector3d(radius * Fixed64.Two, height, radius * Fixed64.Two)
        };
        var agent = new BenchmarkMatterAgent(_context, center);
        agent.Transform.LocalRotation = rotation;
        _capsule.InitializeWithNoBody(agent);
        _circle = new LSCircleCollider2D(circleRadius) { MixedHalfThicknessOverride = Fixed64.One };
        _circle.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(circleRadius, Fixed64.Two));
        cylinder.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        _cylinderPair = new CollisionPair(cylinder, _capsule);
        bool expected = Geometry != "EndpointGap";
        bool hit = CollisionDetectionMixed.TryCollide(_capsule, _circle, out MixedContact contact);
        bool cylinderHit = CollisionDetection.DoCollisionCheck(_cylinderPair);
        if (hit != expected || cylinderHit != expected || (expected && Geometry != "EndpointRim"
                && contact.Depth != Fixed64.FromFraction(1, 4)))
            throw new InvalidOperationException("Capsule/circle contact failed the benchmark preflight.");
        if (expected && contact.Depth != _cylinderPair.Manifold.PrimaryContact.Depth)
            throw new InvalidOperationException("Mixed and 3D contact depths disagree.");
        if (Geometry == "EndpointRim" && (contact.Depth <= Fixed64.Zero || contact.Depth >= Fixed64.One))
            throw new InvalidOperationException("Endpoint rim depth failed the benchmark preflight.");
    }

    [Benchmark]
    public bool Contact() => CollisionDetectionMixed.TryCollide(_capsule, _circle, out _);

    [Benchmark]
    public bool CylinderContact() => CollisionDetection.DoCollisionCheck(_cylinderPair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
