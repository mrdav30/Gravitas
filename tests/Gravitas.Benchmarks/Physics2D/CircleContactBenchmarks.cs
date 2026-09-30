using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CircleContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSCircleCollider2D _first;
    private LSCircleCollider2D _second;

    [Params("Axis", "Diagonal", "Rotated", "Coincident", "LargeAxis", "Separated")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Fixed64 radius = Fixed64.One;
        Vector2d center = Vector2d.Right;
        Fixed64 rotation = Fixed64.Zero;
        Fixed64 expectedDepth = Fixed64.One;
        Vector2d expectedNormal = Vector2d.Right;
        bool expectedHit = true;
        switch (Geometry)
        {
            case "Axis":
                break;
            case "Diagonal":
                radius = (Fixed64)5;
                center = new Vector2d(3, 4);
                expectedDepth = (Fixed64)5;
                expectedNormal = new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
                break;
            case "Rotated":
                rotation = Fixed64.One;
                break;
            case "Coincident":
                center = Vector2d.Zero;
                expectedDepth = Fixed64.Two;
                break;
            case "LargeAxis":
                radius = (Fixed64)25000;
                center = new Vector2d(30000, 0);
                expectedDepth = (Fixed64)20000;
                break;
            case "Separated":
                center = new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2));
                expectedHit = false;
                expectedDepth = Fixed64.Zero;
                expectedNormal = Vector2d.Zero;
                break;
            default:
                throw new InvalidOperationException("Unknown circle contact geometry.");
        }

        _first = new LSCircleCollider2D(radius);
        _second = new LSCircleCollider2D(radius);
        var firstBody = new SolidBody2D(new BenchmarkMatterAgent(_context, Vector3d.Zero), _first);
        var secondBody = new SolidBody2D(new BenchmarkMatterAgent(_context, Vector3d.Zero), _second);
        firstBody.Initialize(Vector2d.Zero, rotation, BodyMotionType.Static);
        secondBody.Initialize(center, -rotation, BodyMotionType.Static);
        bool hit = CollisionDetection2D.TryCollide(_first, _second, out Contact2D contact);
        if (hit != expectedHit || contact.Depth != expectedDepth ||
            contact.Normal != expectedNormal || contact.DepthIsClamped)
            throw new InvalidOperationException($"Incorrect circle contact for {Geometry}.");
    }

    [Benchmark]
    public bool Contact() => CollisionDetection2D.TryCollide(_first, _second, out _);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
