using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class RotationalMovingPairCcdBenchmarks
{
    private static readonly Fixed64 QuarterTurn2D = FixedMath.DegToRad((Fixed64)90);
    private static readonly FixedQuaternion QuarterTurn3D =
        FixedQuaternion.FromAxisAngle(Vector3d.Up, QuarterTurn2D);

    private GravitasWorldContext _pure3D;
    private GravitasWorldContext _pure2D;
    private GravitasWorldContext _mixed3DSource;
    private GravitasWorldContext _mixed2DSource;
    private SwiftList<SolidBody> _pure3DSources;
    private SwiftList<SolidBody> _pure3DTargets;
    private SwiftList<SolidBody2D> _pure2DSources;
    private SwiftList<SolidBody2D> _pure2DTargets;
    private SwiftList<SolidBody> _mixed3DSources;
    private SwiftList<SolidBody2D> _mixed2DTargets;
    private SwiftList<SolidBody2D> _mixed2DSources;
    private SwiftList<SolidBody> _mixed3DTargets;
    private Vector3d[] _sourcePositions3D;
    private Vector2d[] _sourcePositions2D;
    private Vector3d[] _targetPositions3D;
    private Vector2d[] _targetPositions2D;
    private Vector2d[] _mixed2DTargetPositions;
    private Vector3d[] _mixed3DTargetPositions;

    [Params(1, 8, 32)]
    public int PairCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extent = PairCount * 12 + 8;
        _pure3D = CreateContext3D(extent, 8);
        _pure2D = CreateContext2D(extent, 8);
        _mixed3DSource = CreateMixedContext(extent, 8);
        _mixed2DSource = CreateMixedContext(extent, 8);
        _pure3DSources = new SwiftList<SolidBody>(PairCount);
        _pure3DTargets = new SwiftList<SolidBody>(PairCount);
        _pure2DSources = new SwiftList<SolidBody2D>(PairCount);
        _pure2DTargets = new SwiftList<SolidBody2D>(PairCount);
        _mixed3DSources = new SwiftList<SolidBody>(PairCount);
        _mixed2DTargets = new SwiftList<SolidBody2D>(PairCount);
        _mixed2DSources = new SwiftList<SolidBody2D>(PairCount);
        _mixed3DTargets = new SwiftList<SolidBody>(PairCount);
        _sourcePositions3D = new Vector3d[PairCount];
        _sourcePositions2D = new Vector2d[PairCount];
        _targetPositions3D = new Vector3d[PairCount];
        _targetPositions2D = new Vector2d[PairCount];
        _mixed2DTargetPositions = new Vector2d[PairCount];
        _mixed3DTargetPositions = new Vector3d[PairCount];

        Vector2d targetOffset2D = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        for (int i = 0; i < PairCount; i++)
        {
            Vector2d source2D = new((Fixed64)(i * 12), Fixed64.Zero);
            Vector3d source3D = source2D.ToVector3d(Fixed64.Zero);
            Vector2d target2D = source2D + targetOffset2D;
            Vector3d target3D = source3D + QuarterTurn3D
                * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero);
            Vector2d mixed2DTarget = target3D.ToVector2d();
            Vector3d mixed3DTarget = target2D.ToVector3d(Fixed64.Zero);
            _sourcePositions2D[i] = source2D;
            _targetPositions2D[i] = target2D;
            _sourcePositions3D[i] = source3D;
            _targetPositions3D[i] = target3D;
            _mixed2DTargetPositions[i] = mixed2DTarget;
            _mixed3DTargetPositions[i] = mixed3DTarget;

            _pure3DSources.Add(CreateBlade3D(_pure3D, source3D));
            _pure3DTargets.Add(CreateSphere3D(_pure3D, target3D));
            _pure2DSources.Add(CreateBlade2D(_pure2D, source2D));
            _pure2DTargets.Add(CreateCircle2D(_pure2D, target2D));
            _mixed3DSources.Add(CreateBlade3D(_mixed3DSource, source3D));
            _mixed2DTargets.Add(CreateCircle2D(_mixed3DSource, mixed2DTarget));
            _mixed2DSources.Add(CreateBlade2D(_mixed2DSource, source2D));
            _mixed3DTargets.Add(CreateSphere3D(_mixed2DSource, mixed3DTarget));
        }

        Run3D(_pure3D, _pure3DSources, _pure3DTargets);
        Validate3DTargetsResponded(_pure3DTargets, "pure 3D");
        Run2D(_pure2D, _pure2DSources, _pure2DTargets);
        Validate2DTargetsResponded(_pure2DTargets, "pure 2D");
        Run3D(_mixed3DSource, _mixed3DSources, null);
        Validate2DTargetsResponded(_mixed2DTargets, "mixed 3D-to-2D");
        Run2D(_mixed2DSource, _mixed2DSources, null);
        Validate3DTargetsResponded(_mixed3DTargets, "mixed 2D-to-3D");
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _pure3D.Dispose();
        _pure2D.Dispose();
        _mixed3DSource.Dispose();
        _mixed2DSource.Dispose();
    }

    [Benchmark]
    public Vector3d Pure3DRotationalMovingPairs() =>
        Run3D(_pure3D, _pure3DSources, _pure3DTargets);

    [Benchmark]
    public Vector2d Pure2DRotationalMovingPairs() =>
        Run2D(_pure2D, _pure2DSources, _pure2DTargets);

    [Benchmark]
    public Vector3d Mixed3DTo2DRotationalMovingPairs()
    {
        Reset2DTargets(_mixed2DTargets, _mixed2DTargetPositions);
        return Run3D(_mixed3DSource, _mixed3DSources, null)
            + Sum2D(_mixed2DTargets).ToVector3d(Fixed64.Zero);
    }

    [Benchmark]
    public Vector3d Mixed2DTo3DRotationalMovingPairs()
    {
        Reset3DTargets(_mixed3DTargets, _mixed3DTargetPositions);
        return Run2D(_mixed2DSource, _mixed2DSources, null).ToVector3d(Fixed64.Zero)
            + Sum3D(_mixed3DTargets);
    }

    private Vector3d Run3D(
        GravitasWorldContext context,
        SwiftList<SolidBody> sources,
        SwiftList<SolidBody> targets)
    {
        if (targets != null)
            Reset3DTargets(targets, _targetPositions3D);
        for (int i = 0; i < sources.Count; i++)
        {
            SolidBody source = sources[i];
            Vector3d position = _sourcePositions3D[i];
            source.ResetPosition(position, FixedQuaternion.Identity);
            source.Collider.RebuildRuntimeShapeOnly();
            source.Agent.Transform.LocalPosition = position;
            source.Agent.Transform.LocalRotation = QuarterTurn3D;
        }

        context.LateSimulate();
        return Sum3D(sources) + (targets == null ? Vector3d.Zero : Sum3D(targets));
    }

    private Vector2d Run2D(
        GravitasWorldContext context,
        SwiftList<SolidBody2D> sources,
        SwiftList<SolidBody2D> targets)
    {
        if (targets != null)
            Reset2DTargets(targets, _targetPositions2D);
        for (int i = 0; i < sources.Count; i++)
        {
            SolidBody2D source = sources[i];
            Vector2d position = _sourcePositions2D[i];
            source.ResetPosition(position);
            source.Agent.Transform.LocalPosition = position.ToVector3d(Fixed64.Zero);
            source.Agent.Transform.LocalRotationXZRadians = QuarterTurn2D;
        }

        context.LateSimulate();
        return Sum2D(sources) + (targets == null ? Vector2d.Zero : Sum2D(targets));
    }

    private static void Reset3DTargets(
        SwiftList<SolidBody> targets,
        Vector3d[] positions)
    {
        for (int i = 0; i < targets.Count; i++)
        {
            SolidBody target = targets[i];
            target.ResetPosition(positions[i], FixedQuaternion.Identity);
            target.Collider.RebuildRuntimeShapeOnly();
        }
    }

    private static void Reset2DTargets(
        SwiftList<SolidBody2D> targets,
        Vector2d[] positions)
    {
        for (int i = 0; i < targets.Count; i++)
            targets[i].ResetPosition(positions[i]);
    }

    private static void Validate3DTargetsResponded(
        SwiftList<SolidBody> targets,
        string scenario)
    {
        for (int i = 0; i < targets.Count; i++)
        {
            SolidBody target = targets[i];
            if (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.MagnitudeSquared
                <= Fixed64.Epsilon)
            {
                throw new System.InvalidOperationException(
                    $"Rotational moving-pair benchmark setup did not exercise target {i} in the {scenario} scenario.");
            }
        }
    }

    private static void Validate2DTargetsResponded(
        SwiftList<SolidBody2D> targets,
        string scenario)
    {
        for (int i = 0; i < targets.Count; i++)
        {
            SolidBody2D target = targets[i];
            if (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.Abs()
                <= Fixed64.Epsilon)
            {
                throw new System.InvalidOperationException(
                    $"Rotational moving-pair benchmark setup did not exercise target {i} in the {scenario} scenario.");
            }
        }
    }

    private static SolidBody CreateBlade3D(GravitasWorldContext context, Vector3d position)
    {
        var body = new SolidBody(
            new BenchmarkMatterAgent(context, position),
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            })
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };
        body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        return body;
    }

    private static SolidBody2D CreateBlade2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        var body = new SolidBody2D(
            new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero)),
            collider)
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Kinematic);
        return body;
    }
}
