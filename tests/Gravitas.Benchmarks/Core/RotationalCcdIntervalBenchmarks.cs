using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Benchmarking;
using Gravitas.Colliders;
using System;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
[InvocationCount(1)]
public class RotationalCcdIntervalBenchmarks
{
    private static readonly FixedQuaternion QuarterTurn3D = FixedQuaternion.FromEulerAnglesInDegrees(
        Fixed64.Zero,
        (Fixed64)90,
        Fixed64.Zero);
    private static readonly Fixed64 QuarterTurn2D = FixedMath.DegToRad((Fixed64)90);

    private GravitasWorldContext _context3D = null!;
    private GravitasWorldContext _context2D = null!;
    private SolidBody _source3D = null!;
    private SolidBody2D _source2D = null!;

    [Params(1, 8, 32)]
    public int CandidateCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context3D = CreateContext3D(16, 16);
        _context2D = CreateContext2D(16, 16);
        _source3D = CreateKinematicBlade3D(_context3D);
        _source2D = CreateKinematicBlade2D(_context2D);

        Fixed64 candidateRadius = Fixed64.FromFraction(307_072, 100_000);
        Fixed64 quarterTurn = Fixed64.Pi * Fixed64.Half;
        Vector2d candidatePosition = Vector2d.Rotate(
            new Vector2d(candidateRadius, Fixed64.Zero),
            quarterTurn * Fixed64.Half);
        for (int i = 0; i < CandidateCount; i++)
        {
            CreateStaticCuboid3D(
                _context3D,
                new Vector3d(candidatePosition.X, Fixed64.Zero, candidatePosition.Y),
                new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.One, Fixed64.FromFraction(1, 10)));
            CreateStaticAabb2D(
                _context2D,
                candidatePosition,
                Vector2d.One * Fixed64.FromFraction(1, 10));
        }

        PrimeAndValidateScenarios();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context3D.Dispose();
        _context2D.Dispose();
    }

    [Benchmark]
    public FixedQuaternion DenseUnresolved3DRotationalNearMiss()
    {
        _source3D.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
        _source3D.Agent.Transform.LocalRotation = QuarterTurn3D;
        _context3D.LateSimulate();
        return _source3D.Rotation;
    }

    [Benchmark]
    public Fixed64 DenseUnresolved2DRotationalNearMiss()
    {
        _source2D.ResetPosition(Vector2d.Zero);
        _source2D.Agent.Transform.LocalRotationXZRadians = QuarterTurn2D;
        _context2D.LateSimulate();
        return _source2D.Rotation;
    }

    private void PrimeAndValidateScenarios()
    {
        _source3D.Agent.Transform.LocalRotation = QuarterTurn3D;
        _context3D.LateSimulate();
        if (_source3D.Rotation == QuarterTurn3D)
            throw new InvalidOperationException("The 3D benchmark must exercise the unresolved near-miss interval path.");

        _source2D.Agent.Transform.LocalRotationXZRadians = QuarterTurn2D;
        _context2D.LateSimulate();
        if (_source2D.Rotation == QuarterTurn2D)
            throw new InvalidOperationException("The 2D benchmark must exercise the unresolved near-miss interval path.");

        _source3D.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
        _source2D.ResetPosition(Vector2d.Zero);
        _source2D.Agent.Transform.LocalRotationXZRadians = Fixed64.Zero;
    }

    private static SolidBody CreateKinematicBlade3D(GravitasWorldContext context)
    {
        var agent = new BenchmarkMatterAgent(context, Vector3d.Zero);
        var body = new SolidBody(
            agent,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            })
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        return body;
    }

    private static SolidBody2D CreateKinematicBlade2D(GravitasWorldContext context)
    {
        var agent = new BenchmarkMatterAgent(context, Vector3d.Zero);
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        var body = new SolidBody2D(agent, collider)
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
        return body;
    }
}
