using BenchmarkDotNet.Attributes;
using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Configuration;
using SwiftCollections.Diagnostics;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ReplayHashBenchmarks
{
    private GravitasWorldContext _sparse3DContext;
    private GravitasWorldContext _churned3DContext;
    private GravitasWorldContext _dense3DContext;
    private GravitasWorldContext _sparse2DContext;
    private GravitasWorldContext _churned2DContext;
    private GravitasWorldContext _mixedContext;
    private GravitasWorldContext _churnedMixedContext;
    private GravitasWorldContext _solverCacheContext;

    private int _sparse3DBodyCount;
    private int _churned3DLiveColliderCount;
    private int _dense3DBodyCount;
    private int _sparse2DBodyCount;
    private int _churned2DLiveColliderCount;
    private int _mixedColliderCount;
    private int _churnedMixedColliderCount;
    private int _solverCacheContactSeedCount;

    [Params(64, 256)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount);

        _sparse3DContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        _sparse3DBodyCount = BenchmarkPhysicsScene.CreateDynamicSphereGrid(_sparse3DContext, ColliderCount);
        _sparse3DContext.LateSimulate();

        _churned3DContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _churned3DLiveColliderCount = CreateChurnedStaticColliders3D(_churned3DContext, ColliderCount);
        _churned3DContext.LateSimulate();

        _dense3DContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _dense3DBodyCount = BenchmarkPhysicsScene.CreateOverlappingDynamicSpherePairs(_dense3DContext, ColliderCount / 2);
        _dense3DContext.Simulate();

        _sparse2DContext = Create2DContext(gridExtent);
        _sparse2DBodyCount = CreateCircleGrid2D(_sparse2DContext, ColliderCount);
        _sparse2DContext.LateSimulate();

        _churned2DContext = Create2DContext(gridExtent);
        _churned2DLiveColliderCount = CreateChurnedStaticColliders2D(_churned2DContext, ColliderCount);
        _churned2DContext.LateSimulate();

        _mixedContext = CreateMixedContext(gridExtent);
        _mixedColliderCount = CreateMixedPairs(_mixedContext, ColliderCount / 2);
        _mixedContext.Simulate();

        _churnedMixedContext = CreateMixedContext(gridExtent);
        _churnedMixedColliderCount = CreateChurnedMixedStaticColliders(_churnedMixedContext, ColliderCount);
        _churnedMixedContext.LateSimulate();

        _solverCacheContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _solverCacheContactSeedCount = BenchmarkPhysicsScene.CreateOverlappingDynamicSpherePairs(
            _solverCacheContext,
            ColliderCount / 2);
        _solverCacheContext.Simulate();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sparse3DContext?.Dispose();
        _churned3DContext?.Dispose();
        _dense3DContext?.Dispose();
        _sparse2DContext?.Dispose();
        _churned2DContext?.Dispose();
        _mixedContext?.Dispose();
        _churnedMixedContext?.Dispose();
        _solverCacheContext?.Dispose();

        _sparse3DContext = null;
        _churned3DContext = null;
        _dense3DContext = null;
        _sparse2DContext = null;
        _churned2DContext = null;
        _mixedContext = null;
        _churnedMixedContext = null;
        _solverCacheContext = null;
    }

    [Benchmark(Description = "replay-hash-3d-sparse")]
    public ChronicleHash ReplayHash3DSparse()
    {
        SwiftThrowHelper.ThrowIfTrue(_sparse3DBodyCount != ColliderCount);
        return _sparse3DContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-3d-churned-ids")]
    public ChronicleHash ReplayHash3DChurnedIds()
    {
        SwiftThrowHelper.ThrowIfTrue(_churned3DLiveColliderCount != ColliderCount);
        return _churned3DContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-3d-dense")]
    public ChronicleHash ReplayHash3DDense()
    {
        SwiftThrowHelper.ThrowIfTrue(_dense3DBodyCount != ColliderCount);
        return _dense3DContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-2d-sparse")]
    public ChronicleHash ReplayHash2DSparse()
    {
        SwiftThrowHelper.ThrowIfTrue(_sparse2DBodyCount != ColliderCount);
        return _sparse2DContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-2d-churned-ids")]
    public ChronicleHash ReplayHash2DChurnedIds()
    {
        SwiftThrowHelper.ThrowIfTrue(_churned2DLiveColliderCount != ColliderCount);
        return _churned2DContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-mixed")]
    public ChronicleHash ReplayHashMixed()
    {
        SwiftThrowHelper.ThrowIfTrue(_mixedColliderCount != ColliderCount);
        return _mixedContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-mixed-churned-ids")]
    public ChronicleHash ReplayHashMixedChurnedIds()
    {
        SwiftThrowHelper.ThrowIfTrue(_churnedMixedColliderCount != ColliderCount);
        return _churnedMixedContext.ComputeReplayHash();
    }

    [Benchmark(Description = "replay-hash-with-solver-caches")]
    public ChronicleHash ReplayHashWithSolverCaches()
    {
        SwiftThrowHelper.ThrowIfTrue(_solverCacheContactSeedCount != ColliderCount);
        return _solverCacheContext.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
    }

    private static GravitasWorldContext Create2DContext(int gridExtent)
    {
        GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(gridExtent);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.Environment.Gravity = Fixed64.Zero;
        return context;
    }

    private static GravitasWorldContext CreateMixedContext(int gridExtent)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;

        SwiftThrowHelper.ThrowIfTrue(
            !context.World.TryAddGrid(
                new GridConfiguration(
                    new Vector3d(-4, -4, -4),
                    new Vector3d(gridExtent, 4, gridExtent)),
                out _),
            message: "Unable to add replay-hash benchmark GridForge grid.");

        return context;
    }

    private static int CreateCircleGrid2D(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
            _ = CreateCircle2D(context, Position2DForGridIndex(i));

        return count;
    }

    private static int CreateMixedPairs(GravitasWorldContext context, int pairCount)
    {
        for (int i = 0; i < pairCount; i++)
        {
            Vector2d position = Position2DForGridIndex(i);
            _ = BenchmarkPhysicsScene.CreateStaticCollider(
                context,
                new LSSphereCollider { Radius = Fixed64.Half },
                new Vector3d(position.X, Fixed64.Zero, position.Y));
            _ = CreateCircle2D(context, position);
        }

        return pairCount * 2;
    }

    private static int CreateChurnedStaticColliders3D(GravitasWorldContext context, int liveCount)
    {
        const int churnMultiplier = 8;
        int retiredCount = liveCount * (churnMultiplier - 1);

        for (int i = 0; i < retiredCount; i++)
        {
            LSSphereCollider collider = BenchmarkPhysicsScene.CreateStaticCollider(
                context,
                new LSSphereCollider(),
                Position3DForGridIndex(i % liveCount));
            collider.Deactivate();
        }

        for (int i = 0; i < liveCount; i++)
        {
            _ = BenchmarkPhysicsScene.CreateStaticCollider(
                context,
                new LSSphereCollider(),
                Position3DForGridIndex(i));
        }

        return context.Physics.ColliderCount;
    }

    private static int CreateChurnedStaticColliders2D(GravitasWorldContext context, int liveCount)
    {
        const int churnMultiplier = 8;
        int retiredCount = liveCount * (churnMultiplier - 1);

        for (int i = 0; i < retiredCount; i++)
        {
            LSCircleCollider2D collider = CreateBodylessCircle2D(context, Position2DForGridIndex(i % liveCount));
            collider.Deactivate();
        }

        for (int i = 0; i < liveCount; i++)
            _ = CreateBodylessCircle2D(context, Position2DForGridIndex(i));

        return context.Physics2D.ColliderCount;
    }

    private static int CreateChurnedMixedStaticColliders(GravitasWorldContext context, int liveCount)
    {
        const int churnMultiplier = 8;
        int perDimension = liveCount / 2;
        int retiredPerDimension = perDimension * (churnMultiplier - 1);

        for (int i = 0; i < retiredPerDimension; i++)
        {
            int index = i % perDimension;
            LSSphereCollider collider3D = BenchmarkPhysicsScene.CreateStaticCollider(
                context,
                new LSSphereCollider(),
                Position3DForGridIndex(index));
            LSCircleCollider2D collider2D = CreateBodylessCircle2D(context, Position2DForGridIndex(index));
            collider3D.Deactivate();
            collider2D.Deactivate();
        }

        for (int i = 0; i < perDimension; i++)
        {
            _ = BenchmarkPhysicsScene.CreateStaticCollider(
                context,
                new LSSphereCollider(),
                Position3DForGridIndex(i));
            _ = CreateBodylessCircle2D(context, Position2DForGridIndex(i));
        }

        return context.Physics.ColliderCount + context.Physics2D.ColliderCount;
    }

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(
            context,
            new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle2D(
        GravitasWorldContext context,
        Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(
            context,
            new Vector3d(position.X, Fixed64.Zero, position.Y));
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static Vector2d Position2DForGridIndex(int index)
    {
        const int columns = 8;
        int x = index % columns;
        int z = index / columns;
        return new Vector2d((Fixed64)(x * 2), (Fixed64)(z * 2));
    }

    private static Vector3d Position3DForGridIndex(int index)
    {
        const int columns = 8;
        int x = index % columns;
        int z = index / columns;
        return new Vector3d((Fixed64)(x * 2), Fixed64.Zero, (Fixed64)(z * 2));
    }
}
