using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Queries;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class PiecewiseTranslationalCcdBenchmarks
{
    private GravitasWorldContext _context2D;
    private GravitasWorldContext _context3D;
    private SolidBody2D _source2D;
    private SolidBody2D _target2D;
    private SolidBody _source3D;
    private SolidBody _target3D;

    [Params(1, 2, 4)]
    public int TargetSegmentCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context2D = CreateContext2D(16, 16);
        _context3D = CreateContext3D(16, 16);
        _context2D.Settings.ContinuousCollisionMaxToiIterations = 4;
        _context3D.Settings.ContinuousCollisionMaxToiIterations = 4;
        _source2D = CreateCircle2D(
            _context2D,
            new Vector2d((Fixed64)(-5), Fixed64.Zero));
        _target2D = CreateCircle2D(
            _context2D,
            new Vector2d(Fixed64.Zero, (Fixed64)3));
        _source3D = CreateSphere3D(
            _context3D,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        _target3D = CreateSphere3D(
            _context3D,
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        _source2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        _target2D.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, -Fixed64.One));
        _source3D.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        _target3D.ApplyCollisionLinearVelocityDelta(Vector3d.Down);
        _context2D.AdvanceLateSimulateToken();
        _context3D.AdvanceLateSimulateToken();
        _context2D.Physics2D.PrepareContinuousCollisionFrame();
        _context3D.Physics.PrepareContinuousCollisionFrame();
        BuildTargetTrajectory2D();
        BuildTargetTrajectory3D();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context2D.Dispose();
        _context3D.Dispose();
    }

    [Benchmark]
    public Physics2DHit ReduceNoHitPiecewise2DTarget()
    {
        _source2D.TryGetDynamicRelativeContinuousCollisionHit(
            _target2D,
            _source2D.Position,
            Vector2d.Right * (Fixed64)10,
            _source2D.ResolveContinuousCollisionProxyRadius(),
            (Fixed64)10,
            Fixed64.Zero,
            out Physics2DHit hit,
            out _);
        return hit;
    }

    [Benchmark]
    public Physics3DHit ReduceNoHitPiecewise3DTarget()
    {
        _source3D.TryGetDynamicRelativeContinuousCollisionHit(
            _target3D,
            _source3D.Position3d,
            Vector3d.Right * (Fixed64)10,
            _source3D.ResolveContinuousCollisionProxyRadius(),
            (Fixed64)10,
            Fixed64.Zero,
            out Physics3DHit hit,
            out _);
        return hit;
    }

    private void BuildTargetTrajectory2D()
    {
        Fixed64 segmentSpan = Fixed64.One / (Fixed64)TargetSegmentCount;
        Vector2d position = new(Fixed64.Zero, (Fixed64)3);
        Vector2d velocity = new(Fixed64.Zero, -Fixed64.One);
        for (int i = 1; i < TargetSegmentCount; i++)
        {
            position += velocity * segmentSpan;
            velocity = -velocity;
            Fixed64 remainingTime = Fixed64.One
                - Fixed64.FromFraction(i, TargetSegmentCount);
            _target2D.ApplyContinuousCollisionHandoffState(
                position,
                Fixed64.Zero,
                velocity,
                Fixed64.Zero,
                remainingTime);
        }
    }

    private void BuildTargetTrajectory3D()
    {
        Fixed64 segmentSpan = Fixed64.One / (Fixed64)TargetSegmentCount;
        Vector3d position = new(Fixed64.Zero, (Fixed64)3, Fixed64.Zero);
        Vector3d velocity = Vector3d.Down;
        for (int i = 1; i < TargetSegmentCount; i++)
        {
            position += velocity * segmentSpan;
            velocity = -velocity;
            Fixed64 remainingTime = Fixed64.One
                - Fixed64.FromFraction(i, TargetSegmentCount);
            _target3D.ApplyContinuousCollisionHandoff(
                position,
                FixedQuaternion.Identity,
                velocity,
                Vector3d.Zero,
                remainingTime);
        }
    }
}
