using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Configuration;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Contained-start ray witnesses with an outside-start control for each 2D shape.</summary>
[MemoryDiagnoser]
public class InitialOverlapRaycast2DBenchmarks
{
    private GravitasWorldContext _context;
    private LSCollider2D _target;
    private Vector2d _start;
    private Vector2d _end;

    [Params("Circle", "Capsule", "Box", "Polygon", "Compound")]
    public string Shape { get; set; }

    [Params(false, true)]
    public bool ContainedStart { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        if (!_context.World.TryAddGrid(new GridConfiguration(new Vector3d(-4, 0, -4), new Vector3d(4, 0, 4)), out _))
            throw new InvalidOperationException("Raycast benchmark grid was not admitted.");
        _target = Shape switch
        {
            "Circle" => new LSCircleCollider2D(Fixed64.Half),
            "Capsule" => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            "Box" => new LSAABBoxCollider2D(Vector2d.One),
            "Polygon" => new LSPolygonCollider2D(new[]
            {
                new Vector2d(-Fixed64.Half, -Fixed64.Half), new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half), new Vector2d(-Fixed64.Half, Fixed64.Half)
            }),
            _ => new LSCompoundCollider2D(
                CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero),
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * Fixed64.Two))
        };
        _target.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        _start = new Vector2d(ContainedStart ? Fixed64.Quarter : -Fixed64.Two, Fixed64.FromFraction(1, 8));
        _end = _start + Vector2d.Right;
        if (!ContainedStart)
            _end = Vector2d.Right;
        for (int i = 0; i < 8; i++)
        {
            if (!_context.Query2D.Raycast(_start, _end, out var hit)
                || hit.Collider != _target || hit.Normal == Vector2d.Zero
                || (ContainedStart && (hit.Point != _start || hit.Distance != Fixed64.Zero))
                || (!ContainedStart && hit.Distance <= Fixed64.Zero))
                throw new InvalidOperationException("Raycast benchmark witness contract changed.");
        }
    }

    [Benchmark]
    public long Raycast() => _context.Query2D.Raycast(_start, _end, out var hit)
        ? hit.Normal.X.m_rawValue ^ hit.Normal.Y.m_rawValue ^ hit.Distance.m_rawValue
        : -1;

    [GlobalCleanup]
    public void Cleanup() => _context?.Dispose();
}
