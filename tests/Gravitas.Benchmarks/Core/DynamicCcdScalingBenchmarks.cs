using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
[InvocationCount(1)]
public class DynamicCcdScalingBenchmarks
{
    private const int SparseColumns = 64;
    private const int DensePairColumns = 32;
    private const int SparseSpacing = 8;
    private const int DensePairSpacing = 8;
    private const int MixedSparseOffsetZ = 4;
    private const int MixedBatchFrames = 8;

    private static readonly Vector3d Force3D = Vector3d.Right * (Fixed64)2;
    private static readonly Vector2d Force2D = Vector2d.Right * (Fixed64)2;

    private GravitasWorldContext _sparse3DContext;
    private GravitasWorldContext _dense3DContext;
    private GravitasWorldContext _sparse2DContext;
    private GravitasWorldContext _dense2DContext;
    private GravitasWorldContext _sparseMixedContext;
    private GravitasWorldContext _denseMixedContext;

    private SwiftList<StiffBody> _sparse3DBodies;
    private SwiftList<StiffBody> _dense3DBodies;
    private SwiftList<StiffBody2D> _sparse2DBodies;
    private SwiftList<StiffBody2D> _dense2DBodies;
    private SwiftList<StiffBody> _sparseMixed3DBodies;
    private SwiftList<StiffBody2D> _sparseMixed2DBodies;
    private SwiftList<StiffBody> _denseMixed3DBodies;
    private SwiftList<StiffBody2D> _denseMixed2DBodies;
    private SwiftList<PhysicsMixedHit> _mixedQueryHits;

    private Vector3d[] _sparse3DPositions;
    private Vector3d[] _dense3DPositions;
    private Vector2d[] _sparse2DPositions;
    private Vector2d[] _dense2DPositions;
    private Vector3d[] _sparseMixed3DPositions;
    private Vector2d[] _sparseMixed2DPositions;
    private Vector3d[] _denseMixed3DPositions;
    private Vector2d[] _denseMixed2DPositions;

    [Params(64, 256)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int mixedPerDimension = BodyCount / 2;
        _sparse3DContext = CreateContext3D(ExtentForSparse(BodyCount), ExtentForSparseRows(BodyCount));
        _dense3DContext = CreateContext3D(ExtentForDense(BodyCount), ExtentForDenseRows(BodyCount));
        _sparse2DContext = CreateContext2D(ExtentForSparse(BodyCount), ExtentForSparseRows(BodyCount));
        _dense2DContext = CreateContext2D(ExtentForDense(BodyCount), ExtentForDenseRows(BodyCount));
        _sparseMixedContext = CreateMixedContext(
            ExtentForSparse(mixedPerDimension),
            MixedSparseOffsetZ + ExtentForSparseRows(mixedPerDimension));
        _denseMixedContext = CreateMixedContext(
            ExtentForDense(mixedPerDimension * 2),
            ExtentForDenseRows(mixedPerDimension * 2));

        _sparse3DBodies = new SwiftList<StiffBody>(BodyCount);
        _dense3DBodies = new SwiftList<StiffBody>(BodyCount);
        _sparse2DBodies = new SwiftList<StiffBody2D>(BodyCount);
        _dense2DBodies = new SwiftList<StiffBody2D>(BodyCount);
        _sparseMixed3DBodies = new SwiftList<StiffBody>(mixedPerDimension);
        _sparseMixed2DBodies = new SwiftList<StiffBody2D>(mixedPerDimension);
        _denseMixed3DBodies = new SwiftList<StiffBody>(mixedPerDimension);
        _denseMixed2DBodies = new SwiftList<StiffBody2D>(mixedPerDimension);
        _mixedQueryHits = new SwiftList<PhysicsMixedHit>(mixedPerDimension);

        _sparse3DPositions = new Vector3d[BodyCount];
        _dense3DPositions = new Vector3d[BodyCount];
        _sparse2DPositions = new Vector2d[BodyCount];
        _dense2DPositions = new Vector2d[BodyCount];
        _sparseMixed3DPositions = new Vector3d[mixedPerDimension];
        _sparseMixed2DPositions = new Vector2d[mixedPerDimension];
        _denseMixed3DPositions = new Vector3d[mixedPerDimension];
        _denseMixed2DPositions = new Vector2d[mixedPerDimension];

        for (int i = 0; i < BodyCount; i++)
        {
            Vector3d sparse3D = Sparse3DPosition(i);
            Vector3d dense3D = Dense3DPosition(i);
            Vector2d sparse2D = sparse3D.ToVector2d();
            Vector2d dense2D = dense3D.ToVector2d();
            _sparse3DPositions[i] = sparse3D;
            _dense3DPositions[i] = dense3D;
            _sparse2DPositions[i] = sparse2D;
            _dense2DPositions[i] = dense2D;
            _sparse3DBodies.Add(CreateSphere3D(_sparse3DContext, sparse3D));
            _dense3DBodies.Add(CreateSphere3D(_dense3DContext, dense3D));
            _sparse2DBodies.Add(CreateCircle2D(_sparse2DContext, sparse2D));
            _dense2DBodies.Add(CreateCircle2D(_dense2DContext, dense2D));
        }

        for (int i = 0; i < mixedPerDimension; i++)
        {
            Vector3d sparse3D = Sparse3DPosition(i);
            Vector2d sparse2D = Sparse2DPosition(i) + new Vector2d(Fixed64.Zero, (Fixed64)MixedSparseOffsetZ);
            Vector3d dense3D = DenseMixed3DPosition(i);
            Vector2d dense2D = DenseMixed2DPosition(i);
            _sparseMixed3DPositions[i] = sparse3D;
            _sparseMixed2DPositions[i] = sparse2D;
            _denseMixed3DPositions[i] = dense3D;
            _denseMixed2DPositions[i] = dense2D;
            _sparseMixed3DBodies.Add(CreateSphere3D(_sparseMixedContext, sparse3D));
            _sparseMixed2DBodies.Add(CreateCircle2D(_sparseMixedContext, sparse2D));
            _denseMixed3DBodies.Add(CreateSphere3D(_denseMixedContext, dense3D));
            _denseMixed2DBodies.Add(CreateCircle2D(_denseMixedContext, dense2D));
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sparse3DContext.Dispose();
        _dense3DContext.Dispose();
        _sparse2DContext.Dispose();
        _dense2DContext.Dispose();
        _sparseMixedContext.Dispose();
        _denseMixedContext.Dispose();
        _sparse3DContext = null;
        _dense3DContext = null;
        _sparse2DContext = null;
        _dense2DContext = null;
        _sparseMixedContext = null;
        _denseMixedContext = null;
        _sparse3DBodies = null;
        _dense3DBodies = null;
        _sparse2DBodies = null;
        _dense2DBodies = null;
        _sparseMixed3DBodies = null;
        _sparseMixed2DBodies = null;
        _denseMixed3DBodies = null;
        _denseMixed2DBodies = null;
        _mixedQueryHits = null;
        _sparse3DPositions = null;
        _dense3DPositions = null;
        _sparse2DPositions = null;
        _dense2DPositions = null;
        _sparseMixed3DPositions = null;
        _sparseMixed2DPositions = null;
        _denseMixed3DPositions = null;
        _denseMixed2DPositions = null;
    }

    [Benchmark]
    public Vector3d Sparse3DDynamicCcd()
    {
        Reset3DBodies(_sparse3DBodies, _sparse3DPositions, pairedDirections: false);
        _sparse3DContext.LateSimulate();
        return Sum3D(_sparse3DBodies);
    }

    [Benchmark]
    public Vector3d Dense3DDynamicCcd()
    {
        Reset3DBodies(_dense3DBodies, _dense3DPositions, pairedDirections: true);
        _dense3DContext.LateSimulate();
        return Sum3D(_dense3DBodies);
    }

    [Benchmark]
    public Vector2d Sparse2DDynamicCcd()
    {
        Reset2DBodies(_sparse2DBodies, _sparse2DPositions, pairedDirections: false);
        _sparse2DContext.LateSimulate();
        return Sum2D(_sparse2DBodies);
    }

    [Benchmark]
    public Vector2d Dense2DDynamicCcd()
    {
        Reset2DBodies(_dense2DBodies, _dense2DPositions, pairedDirections: true);
        _dense2DContext.LateSimulate();
        return Sum2D(_dense2DBodies);
    }

    [Benchmark]
    public Vector3d SparseMixedDynamicCcd()
    {
        Reset3DBodies(_sparseMixed3DBodies, _sparseMixed3DPositions, pairedDirections: false);
        Reset2DBodies(_sparseMixed2DBodies, _sparseMixed2DPositions, pairedDirections: false);
        _sparseMixedContext.LateSimulate();
        return Sum3D(_sparseMixed3DBodies) + ToVector3D(Sum2D(_sparseMixed2DBodies));
    }

    [Benchmark]
    public Vector3d DenseMixedDynamicCcd()
    {
        Reset3DBodies(_denseMixed3DBodies, _denseMixed3DPositions, pairedDirections: false);
        Reset2DBodies(_denseMixed2DBodies, _denseMixed2DPositions, pairedDirections: true);
        _denseMixedContext.LateSimulate();
        return Sum3D(_denseMixed3DBodies) + ToVector3D(Sum2D(_denseMixed2DBodies));
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public Vector3d SparseMixedDynamicCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < MixedBatchFrames; i++)
        {
            Reset3DBodies(_sparseMixed3DBodies, _sparseMixed3DPositions, pairedDirections: false);
            Reset2DBodies(_sparseMixed2DBodies, _sparseMixed2DPositions, pairedDirections: false);
            _sparseMixedContext.LateSimulate();
            total += Sum3D(_sparseMixed3DBodies) + ToVector3D(Sum2D(_sparseMixed2DBodies));
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public Vector3d DenseMixedDynamicCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < MixedBatchFrames; i++)
        {
            Reset3DBodies(_denseMixed3DBodies, _denseMixed3DPositions, pairedDirections: false);
            Reset2DBodies(_denseMixed2DBodies, _denseMixed2DPositions, pairedDirections: true);
            _denseMixedContext.LateSimulate();
            total += Sum3D(_denseMixed3DBodies) + ToVector3D(Sum2D(_denseMixed2DBodies));
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int SparseMixedStatic2DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepStatic2DQueries(_sparseMixedContext, _sparseMixed3DBodies, _sparseMixed3DPositions);

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int DenseMixedStatic2DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepStatic2DQueries(_denseMixedContext, _denseMixed3DBodies, _denseMixed3DPositions);

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int SparseMixedStatic3DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepStatic3DQueries(_sparseMixedContext, _sparseMixed2DBodies, _sparseMixed2DPositions);

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int DenseMixedStatic3DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepStatic3DQueries(_denseMixedContext, _denseMixed2DBodies, _denseMixed2DPositions);

        return total;
    }

    private static void Reset3DBodies(SwiftList<StiffBody> bodies, Vector3d[] positions, bool pairedDirections)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            StiffBody body = bodies[i];
            body.ResetPosition(positions[i], FixedQuaternion.Identity);
            body.AddForce(Get3DForce(i, pairedDirections));
        }
    }

    private static void Reset2DBodies(SwiftList<StiffBody2D> bodies, Vector2d[] positions, bool pairedDirections)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            StiffBody2D body = bodies[i];
            body.Sleep();
            body.SetPosition(positions[i]);
            body.AddForce(Get2DForce(i, pairedDirections));
        }
    }

    private static Vector3d Get3DForce(int index, bool pairedDirections) =>
        pairedDirections && (index & 1) == 1 ? -Force3D : Force3D;

    private static Vector2d Get2DForce(int index, bool pairedDirections) =>
        pairedDirections && (index & 1) == 1 ? -Force2D : Force2D;

    private static Vector3d Sum3D(SwiftList<StiffBody> bodies)
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < bodies.Count; i++)
            total += bodies[i].Position3d;

        return total;
    }

    private static Vector2d Sum2D(SwiftList<StiffBody2D> bodies)
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < bodies.Count; i++)
            total += bodies[i].Position;

        return total;
    }

    private static Vector3d ToVector3D(Vector2d value) =>
        new(value.X, Fixed64.Zero, value.Y);

    private int SweepStatic2DQueries(GravitasWorldContext context, SwiftList<StiffBody> bodies, Vector3d[] positions)
    {
        int total = 0;
        context.AdvanceLateSimulateToken();
        for (int i = 0; i < bodies.Count; i++)
        {
            Vector3d start = positions[i];
            Vector3d end = start + Force3D;
            total += context.QueryMixed.SweepSphereAgainstStatic2DAll(
                start,
                end,
                Fixed64.Half,
                PhysicsLayerMask.All,
                _mixedQueryHits,
                bodies[i].Collider,
                includeTriggers: false,
                cacheTargetPartitions: true);
            total += context.QueryMixed.LastQueryCandidateCount;
        }

        return total;
    }

    private int SweepStatic3DQueries(GravitasWorldContext context, SwiftList<StiffBody2D> bodies, Vector2d[] positions)
    {
        int total = 0;
        context.AdvanceLateSimulateToken();
        for (int i = 0; i < bodies.Count; i++)
        {
            Vector2d start = positions[i];
            Vector2d end = start + Force2D;
            total += context.QueryMixed.SweepCircleAgainstStatic3DAll(
                start,
                end,
                Fixed64.Half,
                bodies[i].Collider.MixedSlabCenterY,
                bodies[i].Collider.MixedHalfThickness,
                PhysicsLayerMask.All,
                _mixedQueryHits,
                bodies[i].Collider,
                includeTriggers: false,
                cacheTargetPartitions: true);
            total += context.QueryMixed.LastQueryCandidateCount;
        }

        return total;
    }

    private static GravitasWorldContext CreateContext3D(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        ConfigureContext(context);
        AddGrid(context, extentX, extentZ);
        return context;
    }

    private static GravitasWorldContext CreateContext2D(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        ConfigureContext(context);
        AddGrid(context, extentX, extentZ);
        return context;
    }

    private static GravitasWorldContext CreateMixedContext(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ConfigureContext(context);
        AddGrid(context, extentX, extentZ);
        return context;
    }

    private static void ConfigureContext(GravitasWorldContext context)
    {
        context.SetFrameRate(1);
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.None;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
    }

    private static void AddGrid(GravitasWorldContext context, int extentX, int extentZ)
    {
        if (!context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), (Fixed64)(-8), (Fixed64)(-16)),
                new Vector3d((Fixed64)extentX, (Fixed64)8, (Fixed64)extentZ)),
            out _))
        {
            throw new InvalidOperationException("Unable to create dynamic CCD benchmark grid.");
        }
    }

    private static StiffBody CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new StiffBody(agent, new LSSphereCollider())
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            GroundProbeMode = GroundProbeMode.Ray,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static StiffBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    private static Vector3d Sparse3DPosition(int index)
    {
        int x = index % SparseColumns;
        int z = index / SparseColumns;
        return new Vector3d((Fixed64)(x * SparseSpacing), Fixed64.Zero, (Fixed64)(z * SparseSpacing));
    }

    private static Vector2d Sparse2DPosition(int index)
    {
        Vector3d position = Sparse3DPosition(index);
        return new Vector2d(position.X, position.Z);
    }

    private static Vector3d Dense3DPosition(int index)
    {
        int pair = index / 2;
        int side = index & 1;
        int x = pair % DensePairColumns;
        int z = pair / DensePairColumns;
        Fixed64 centerX = (Fixed64)(x * DensePairSpacing);
        Fixed64 offsetX = side == 0 ? (Fixed64)(-2) : (Fixed64)2;
        return new Vector3d(centerX + offsetX, Fixed64.Zero, (Fixed64)(z * DensePairSpacing));
    }

    private static Vector2d Dense2DPosition(int index)
    {
        Vector3d position = Dense3DPosition(index);
        return new Vector2d(position.X, position.Z);
    }

    private static Vector3d DenseMixed3DPosition(int index)
    {
        int x = index % DensePairColumns;
        int z = index / DensePairColumns;
        return new Vector3d((Fixed64)(x * DensePairSpacing - 2), Fixed64.Zero, (Fixed64)(z * DensePairSpacing));
    }

    private static Vector2d DenseMixed2DPosition(int index)
    {
        int x = index % DensePairColumns;
        int z = index / DensePairColumns;
        return new Vector2d((Fixed64)(x * DensePairSpacing + 2), (Fixed64)(z * DensePairSpacing));
    }

    private static int ExtentForSparse(int count) =>
        Math.Max(32, SparseColumns * SparseSpacing + 32);

    private static int ExtentForSparseRows(int count)
    {
        int rows = (count + SparseColumns - 1) / SparseColumns;
        return Math.Max(32, rows * SparseSpacing + 32);
    }

    private static int ExtentForDense(int count) =>
        Math.Max(32, DensePairColumns * DensePairSpacing + 32);

    private static int ExtentForDenseRows(int count)
    {
        int pairs = count / 2;
        int rows = (pairs + DensePairColumns - 1) / DensePairColumns;
        return Math.Max(32, rows * DensePairSpacing + 32);
    }
}
