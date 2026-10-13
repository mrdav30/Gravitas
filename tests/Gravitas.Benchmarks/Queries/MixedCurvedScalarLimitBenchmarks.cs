using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Translated curved-target admission, isolated from broad-phase grid traversal.</summary>
[MemoryDiagnoser]
public class MixedCurvedScalarLimitBenchmarks
{
    private GravitasWorldContext _context;
    private LSCollider _target;
    private Vector2d _start;
    private Vector2d _end;
    private Vector2d _direction;
    private Vector3d _direction3D;
    private Fixed64 _length;

    [Params("Cylinder", "Capsule", "Cone")]
    public string Shape { get; set; }

    [Params("Origin", "Max", "Min")]
    public string CoordinateFrame { get; set; }

    [Params(false, true)]
    public bool ContainedStart { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools: true);
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 sign = CoordinateFrame == "Min" ? -Fixed64.One : Fixed64.One;
        Fixed64 edge = CoordinateFrame switch
        {
            "Max" => Fixed64.MaxValue,
            "Min" => Fixed64.MinValue,
            _ => Fixed64.Zero
        };
        Fixed64 center = edge - sign * Fixed64.FromFraction(1, 100);
        Vector3d size = new(Fixed64.One, Fixed64.FromFraction(1, 10), Fixed64.One);
        _target = Shape switch
        {
            "Cylinder" => new LSCylinderCollider { Radius = Fixed64.Quarter, Size = size },
            "Capsule" => new LSCapsuleCollider
            {
                Radius = Fixed64.Quarter,
                Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 5), Fixed64.One)
            },
            _ => new LSConeCollider { Radius = Fixed64.Quarter, Size = size }
        };
        var agent = new BenchmarkMatterAgent(_context, new Vector3d(center, Fixed64.Zero, Fixed64.Zero));
        agent.Transform.LocalRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -sign, (Fixed64)2).Normalized;
        _target.InitializeWithNoBody(agent);
        Fixed64 containedX = edge - sign * Fixed64.FromFraction(1, 20);
        _start = new Vector2d(ContainedStart
            ? containedX
            : edge - sign * Fixed64.FromFraction(3, 5), Fixed64.Zero);
        _end = new Vector2d(ContainedStart
            ? containedX - sign * Fixed64.FromFraction(1, 1000)
            : containedX, Fixed64.Zero);
        Vector2d displacement = _end - _start;
        _length = displacement.Magnitude;
        _direction = displacement / _length;
        _direction3D = new Vector3d(_direction.X, Fixed64.Zero, _direction.Y);
        for (int i = 0; i < 8; i++)
        {
            bool admitted = SweepCircleReducer();
            if (!admitted)
                throw new InvalidOperationException("The curved-target fixture must admit its contact.");
        }
    }

    [Benchmark]
    public bool SweepCircleReducer() => _context.QueryMixed.TrySweepCircleAgainst3DCollider(
        _target, _start, _end, _direction, _length, Fixed64.FromFraction(1, 100),
        Fixed64.Zero, Fixed64.Quarter, _direction3D, null, out _);

    [GlobalCleanup]
    public void Cleanup() => _context?.Dispose();
}
