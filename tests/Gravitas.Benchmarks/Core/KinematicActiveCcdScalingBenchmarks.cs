using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Benchmarking;
using Gravitas.Colliders;
using SwiftCollections;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
[InvocationCount(1)]
public class KinematicActiveCcdScalingBenchmarks
{
    private const int BatchFrames = 8;
    private static readonly Vector3d Translation3D = Vector3d.Right * (Fixed64)5;
    private static readonly Vector2d Translation2D = Vector2d.Right * (Fixed64)5;
    private static readonly Vector3d TargetOffset3D = Vector3d.Right * (Fixed64)3;
    private static readonly Vector2d TargetOffset2D = Vector2d.Right * (Fixed64)3;
    private static readonly FixedQuaternion QuarterTurn3D = FixedQuaternion.FromEulerAnglesInDegrees(
        Fixed64.Zero,
        (Fixed64)90,
        Fixed64.Zero);
    private static readonly Fixed64 QuarterTurn2D = FixedMath.DegToRad((Fixed64)90);

    private GravitasWorldContext _sparse3DContext;
    private GravitasWorldContext _firstHit3DContext;
    private GravitasWorldContext _dense3DContext;
    private GravitasWorldContext _rotational3DContext;
    private GravitasWorldContext _sparse2DContext;
    private GravitasWorldContext _firstHit2DContext;
    private GravitasWorldContext _dense2DContext;
    private GravitasWorldContext _rotational2DContext;
    private GravitasWorldContext _mixed3DSourceContext;
    private GravitasWorldContext _mixed2DSourceContext;
    private SwiftList<StiffBody> _sparse3DSources;
    private SwiftList<StiffBody> _firstHit3DSources;
    private SwiftList<StiffBody> _dense3DSources;
    private SwiftList<StiffBody> _rotational3DSources;
    private SwiftList<StiffBody2D> _sparse2DSources;
    private SwiftList<StiffBody2D> _firstHit2DSources;
    private SwiftList<StiffBody2D> _dense2DSources;
    private SwiftList<StiffBody2D> _rotational2DSources;
    private SwiftList<StiffBody> _mixed3DSources;
    private SwiftList<StiffBody2D> _mixed2DTargets;
    private SwiftList<StiffBody2D> _mixed2DSources;
    private SwiftList<StiffBody> _mixed3DTargets;
    private Vector3d[] _sparse3DPositions;
    private Vector3d[] _firstHit3DPositions;
    private Vector3d[] _dense3DPositions;
    private Vector3d[] _rotational3DPositions;
    private Vector2d[] _sparse2DPositions;
    private Vector2d[] _firstHit2DPositions;
    private Vector2d[] _dense2DPositions;
    private Vector2d[] _rotational2DPositions;
    private Vector3d[] _mixed3DSourcePositions;
    private Vector2d[] _mixed2DTargetPositions;
    private Vector2d[] _mixed2DSourcePositions;
    private Vector3d[] _mixed3DTargetPositions;

    [Params(64, 256)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int mixedCount = BodyCount / 2;
        _sparse3DContext = CreateContext3D(ContinuousCollisionBenchmarkLayout.SparseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(BodyCount));
        _firstHit3DContext = CreateContext3D(ContinuousCollisionBenchmarkLayout.SparseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(BodyCount));
        _dense3DContext = CreateContext3D(ContinuousCollisionBenchmarkLayout.DenseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.DenseExtentZ(BodyCount));
        _rotational3DContext = CreateContext3D(ContinuousCollisionBenchmarkLayout.SparseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(BodyCount) + 16);
        _sparse2DContext = CreateContext2D(ContinuousCollisionBenchmarkLayout.SparseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(BodyCount));
        _firstHit2DContext = CreateContext2D(ContinuousCollisionBenchmarkLayout.SparseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(BodyCount));
        _dense2DContext = CreateContext2D(ContinuousCollisionBenchmarkLayout.DenseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.DenseExtentZ(BodyCount));
        _rotational2DContext = CreateContext2D(ContinuousCollisionBenchmarkLayout.SparseExtentX(BodyCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(BodyCount) + 16);
        _mixed3DSourceContext = CreateMixedContext(ContinuousCollisionBenchmarkLayout.SparseExtentX(mixedCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(mixedCount) + 16);
        _mixed2DSourceContext = CreateMixedContext(ContinuousCollisionBenchmarkLayout.SparseExtentX(mixedCount) + 16, ContinuousCollisionBenchmarkLayout.SparseExtentZ(mixedCount) + 16);

        _sparse3DSources = new SwiftList<StiffBody>(BodyCount);
        _firstHit3DSources = new SwiftList<StiffBody>(BodyCount);
        _dense3DSources = new SwiftList<StiffBody>(BodyCount);
        _rotational3DSources = new SwiftList<StiffBody>(BodyCount);
        _sparse2DSources = new SwiftList<StiffBody2D>(BodyCount);
        _firstHit2DSources = new SwiftList<StiffBody2D>(BodyCount);
        _dense2DSources = new SwiftList<StiffBody2D>(BodyCount);
        _rotational2DSources = new SwiftList<StiffBody2D>(BodyCount);
        _mixed3DSources = new SwiftList<StiffBody>(mixedCount);
        _mixed2DTargets = new SwiftList<StiffBody2D>(mixedCount);
        _mixed2DSources = new SwiftList<StiffBody2D>(mixedCount);
        _mixed3DTargets = new SwiftList<StiffBody>(mixedCount);

        _sparse3DPositions = new Vector3d[BodyCount];
        _firstHit3DPositions = new Vector3d[BodyCount];
        _dense3DPositions = new Vector3d[BodyCount];
        _rotational3DPositions = new Vector3d[BodyCount];
        _sparse2DPositions = new Vector2d[BodyCount];
        _firstHit2DPositions = new Vector2d[BodyCount];
        _dense2DPositions = new Vector2d[BodyCount];
        _rotational2DPositions = new Vector2d[BodyCount];
        _mixed3DSourcePositions = new Vector3d[mixedCount];
        _mixed2DTargetPositions = new Vector2d[mixedCount];
        _mixed2DSourcePositions = new Vector2d[mixedCount];
        _mixed3DTargetPositions = new Vector3d[mixedCount];

        CreatePureScenes();
        CreateMixedScenes(mixedCount);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sparse3DContext.Dispose();
        _firstHit3DContext.Dispose();
        _dense3DContext.Dispose();
        _rotational3DContext.Dispose();
        _sparse2DContext.Dispose();
        _firstHit2DContext.Dispose();
        _dense2DContext.Dispose();
        _rotational2DContext.Dispose();
        _mixed3DSourceContext.Dispose();
        _mixed2DSourceContext.Dispose();
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector3d Pure3DKinematicNoHitBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic3DSources(_sparse3DSources, _sparse3DPositions, Translation3D);
            _sparse3DContext.LateSimulate();
            total += Sum3D(_sparse3DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector3d Pure3DKinematicFirstHitBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic3DSources(_firstHit3DSources, _firstHit3DPositions, Translation3D);
            _firstHit3DContext.LateSimulate();
            total += Sum3D(_firstHit3DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector3d Pure3DKinematicDenseHitBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic3DSources(_dense3DSources, _dense3DPositions, Translation3D);
            _dense3DContext.LateSimulate();
            total += Sum3D(_dense3DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector3d Pure3DKinematicRotationalBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic3DRotationalSources(_rotational3DSources, _rotational3DPositions);
            _rotational3DContext.LateSimulate();
            total += Sum3D(_rotational3DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector2d Pure2DKinematicNoHitBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic2DSources(_sparse2DSources, _sparse2DPositions, Translation2D);
            _sparse2DContext.LateSimulate();
            total += Sum2D(_sparse2DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector2d Pure2DKinematicFirstHitBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic2DSources(_firstHit2DSources, _firstHit2DPositions, Translation2D);
            _firstHit2DContext.LateSimulate();
            total += Sum2D(_firstHit2DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector2d Pure2DKinematicDenseHitBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic2DSources(_dense2DSources, _dense2DPositions, Translation2D);
            _dense2DContext.LateSimulate();
            total += Sum2D(_dense2DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector2d Pure2DKinematicRotationalBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic2DRotationalSources(_rotational2DSources, _rotational2DPositions);
            _rotational2DContext.LateSimulate();
            total += Sum2D(_rotational2DSources);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector3d MixedKinematic3DSourceFirstHitBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic3DSources(_mixed3DSources, _mixed3DSourcePositions, Translation3D);
            Reset2DBodyPositions(_mixed2DTargets, _mixed2DTargetPositions);
            _mixed3DSourceContext.LateSimulate();
            total += Sum3D(_mixed3DSources) + Sum2D(_mixed2DTargets).ToVector3d(Fixed64.Zero);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = BatchFrames)]
    public Vector3d MixedKinematic2DSourceFirstHitBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int frame = 0; frame < BatchFrames; frame++)
        {
            ResetKinematic2DSources(_mixed2DSources, _mixed2DSourcePositions, Translation2D);
            Reset3DBodyPositions(_mixed3DTargets, _mixed3DTargetPositions);
            _mixed2DSourceContext.LateSimulate();
            total += Sum2D(_mixed2DSources).ToVector3d(Fixed64.Zero) + Sum3D(_mixed3DTargets);
        }

        return total;
    }

    private void CreatePureScenes()
    {
        for (int i = 0; i < BodyCount; i++)
        {
            Vector3d sparse3D = ContinuousCollisionBenchmarkLayout.Sparse3DPosition(i);
            Vector3d dense3D = ContinuousCollisionBenchmarkLayout.Dense3DPosition(i);
            Vector2d sparse2D = sparse3D.ToVector2d();
            Vector2d dense2D = dense3D.ToVector2d();

            _sparse3DPositions[i] = sparse3D;
            _firstHit3DPositions[i] = sparse3D;
            _dense3DPositions[i] = dense3D;
            _rotational3DPositions[i] = sparse3D;
            _sparse2DPositions[i] = sparse2D;
            _firstHit2DPositions[i] = sparse2D;
            _dense2DPositions[i] = dense2D;
            _rotational2DPositions[i] = sparse2D;

            _sparse3DSources.Add(CreateKinematicSphere3D(_sparse3DContext, sparse3D));
            _firstHit3DSources.Add(CreateKinematicSphere3D(_firstHit3DContext, sparse3D));
            _dense3DSources.Add(CreateKinematicSphere3D(_dense3DContext, dense3D));
            _rotational3DSources.Add(CreateKinematicThinCuboid3D(_rotational3DContext, sparse3D));
            CreateStaticSphere3D(_firstHit3DContext, sparse3D + TargetOffset3D);
            CreateStaticSphere3D(_dense3DContext, dense3D + TargetOffset3D);
            CreateStaticSphere3D(_rotational3DContext, sparse3D + new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));

            _sparse2DSources.Add(CreateKinematicCircle2D(_sparse2DContext, sparse2D));
            _firstHit2DSources.Add(CreateKinematicCircle2D(_firstHit2DContext, sparse2D));
            _dense2DSources.Add(CreateKinematicCircle2D(_dense2DContext, dense2D));
            _rotational2DSources.Add(CreateKinematicThinPolygon2D(_rotational2DContext, sparse2D));
            CreateStaticCircle2D(_firstHit2DContext, sparse2D + TargetOffset2D);
            CreateStaticCircle2D(_dense2DContext, dense2D + TargetOffset2D);
            CreateStaticCircle2D(_rotational2DContext, sparse2D + new Vector2d((Fixed64)2, (Fixed64)2));
        }
    }

    private void CreateMixedScenes(int mixedCount)
    {
        for (int i = 0; i < mixedCount; i++)
        {
            Vector3d source3D = ContinuousCollisionBenchmarkLayout.Sparse3DPosition(i);
            Vector2d target2D = source3D.ToVector2d() + TargetOffset2D;
            Vector2d source2D = ContinuousCollisionBenchmarkLayout.Sparse2DPosition(i);
            Vector3d target3D = source2D.ToVector3d(Fixed64.Zero) + TargetOffset3D;
            _mixed3DSourcePositions[i] = source3D;
            _mixed2DTargetPositions[i] = target2D;
            _mixed2DSourcePositions[i] = source2D;
            _mixed3DTargetPositions[i] = target3D;
            _mixed3DSources.Add(CreateKinematicSphere3D(_mixed3DSourceContext, source3D));
            _mixed2DTargets.Add(CreateCircle2D(_mixed3DSourceContext, target2D));
            _mixed2DSources.Add(CreateKinematicCircle2D(_mixed2DSourceContext, source2D));
            _mixed3DTargets.Add(CreateSphere3D(_mixed2DSourceContext, target3D));
        }
    }

    private static StiffBody CreateKinematicSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new StiffBody(agent, new LSSphereCollider())
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            IsKinematic = true,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static StiffBody CreateKinematicThinCuboid3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new StiffBody(
            agent,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            })
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            IsKinematic = true,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static StiffBody2D CreateKinematicCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero));
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            IsKinematic = true,
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    private static StiffBody2D CreateKinematicThinPolygon2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero));
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        var body = new StiffBody2D(agent, collider)
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            IsKinematic = true,
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    private static void ResetKinematic3DSources(SwiftList<StiffBody> sources, Vector3d[] positions, Vector3d displacement)
    {
        for (int i = 0; i < sources.Count; i++)
        {
            StiffBody source = sources[i];
            Vector3d position = positions[i];
            source.ResetPosition(position, FixedQuaternion.Identity);
            source.Agent.Transform.Position = position + displacement;
        }
    }

    private static void ResetKinematic3DRotationalSources(SwiftList<StiffBody> sources, Vector3d[] positions)
    {
        for (int i = 0; i < sources.Count; i++)
        {
            StiffBody source = sources[i];
            Vector3d position = positions[i];
            source.ResetPosition(position, FixedQuaternion.Identity);
            source.Agent.Transform.Rotation = QuarterTurn3D;
        }
    }

    private static void ResetKinematic2DSources(SwiftList<StiffBody2D> sources, Vector2d[] positions, Vector2d displacement)
    {
        for (int i = 0; i < sources.Count; i++)
        {
            StiffBody2D source = sources[i];
            Vector2d position = positions[i];
            source.SetPosition(position);
            source.SetRotation(Fixed64.Zero);
            source.Agent.Transform.Position = (position + displacement).ToVector3d(Fixed64.Zero);
            source.Agent.Transform.Rotation = FixedQuaternion.Identity;
        }
    }

    private static void ResetKinematic2DRotationalSources(SwiftList<StiffBody2D> sources, Vector2d[] positions)
    {
        for (int i = 0; i < sources.Count; i++)
        {
            StiffBody2D source = sources[i];
            Vector2d position = positions[i];
            source.SetPosition(position);
            source.SetRotation(Fixed64.Zero);
            source.Agent.Transform.Position = position.ToVector3d(Fixed64.Zero);
            source.Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                FixedMath.RadToDeg(QuarterTurn2D),
                Fixed64.Zero);
        }
    }
}
