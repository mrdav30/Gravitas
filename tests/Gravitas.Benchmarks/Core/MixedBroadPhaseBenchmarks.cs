using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Configuration;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedBroadPhaseBenchmarks
{
    private GravitasWorldContext _sparseContext;
    private GravitasWorldContext _denseContext;
    private GravitasWorldContext _churnContext;
    private SwiftList<SolidBody2D> _churnBodies2D;
    private bool _churnToggle;

    [Params(32, 1024)]
    public int ColliderCount { get; set; }

    [GlobalSetup(Target = nameof(SparseCandidateGathering))]
    public void SetupSparse()
    {
        _sparseContext = CreateMixedContext(
            SparseExtentX(ColliderCount),
            SparseExtentZ(ColliderCount),
            clearAllPools: true);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector2d position = SparsePositionForIndex(i);
            _ = CreateTriggerSphere3D(_sparseContext, new Vector3d(position.X, Fixed64.Zero, position.Y));
            _ = CreateCircle2D(_sparseContext, position);
        }

        AdvanceMixedFrame(_sparseContext);
        ValidateCandidates(_sparseContext, "sparse");
    }

    [GlobalSetup(Target = nameof(DenseCandidateGathering))]
    public void SetupDense()
    {
        _denseContext = CreateMixedContext(
            DenseExtentX(ColliderCount),
            DenseExtentZ(ColliderCount),
            clearAllPools: true);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector2d position = DensePositionForIndex(i);
            _ = CreateTriggerSphere3D(_denseContext, new Vector3d(position.X, Fixed64.Zero, position.Y));
            _ = CreateCircle2D(_denseContext, position);
        }

        AdvanceMixedFrame(_denseContext);
        ValidateCandidates(_denseContext, "dense");
    }

    [GlobalSetup(Target = nameof(RetainedPartitionCleanupAfterChurn))]
    public void SetupChurn()
    {
        _churnContext = CreateMixedContext(
            SparseExtentX(ColliderCount) + 64,
            SparseExtentZ(ColliderCount) + 64,
            clearAllPools: true);
        _churnBodies2D = new SwiftList<SolidBody2D>(ColliderCount);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector2d position = SparsePositionForIndex(i);
            Vector2d churnPosition = position + new Vector2d((Fixed64)16, Fixed64.Zero);
            _ = CreateTriggerSphere3D(_churnContext, new Vector3d(churnPosition.X, Fixed64.Zero, churnPosition.Y));
            _churnBodies2D.Add(CreateCircle2D(_churnContext, churnPosition));
        }

        AdvanceMixedFrame(_churnContext);
        ValidateCandidates(_churnContext, "churn");
    }

    [GlobalCleanup(Target = nameof(SparseCandidateGathering))]
    public void CleanupSparse()
    {
        _sparseContext?.Dispose();
        _sparseContext = null;
    }

    [GlobalCleanup(Target = nameof(DenseCandidateGathering))]
    public void CleanupDense()
    {
        _denseContext?.Dispose();
        _denseContext = null;
    }

    [GlobalCleanup(Target = nameof(RetainedPartitionCleanupAfterChurn))]
    public void CleanupChurn()
    {
        _churnContext?.Dispose();
        _churnContext = null;
        _churnBodies2D = null;
    }

    [Benchmark]
    public int SparseCandidateGathering()
    {
        AdvanceMixedFrame(_sparseContext);
        return _sparseContext.MixedCollisions.LastBroadPhaseCandidateCount;
    }

    [Benchmark]
    public int DenseCandidateGathering()
    {
        AdvanceMixedFrame(_denseContext);
        return _denseContext.MixedCollisions.LastBroadPhaseCandidateCount;
    }

    [Benchmark]
    public int RetainedPartitionCleanupAfterChurn()
    {
        Vector2d offset = _churnToggle
            ? new Vector2d((Fixed64)16, Fixed64.Zero)
            : new Vector2d((Fixed64)32, Fixed64.Zero);

        for (int i = 0; i < _churnBodies2D.Count; i++)
            _churnBodies2D[i].SetPosition(SparsePositionForIndex(i) + offset);

        _churnToggle = !_churnToggle;
        AdvanceMixedFrame(_churnContext);
        return _churnContext.MixedCollisions.RetainedPartitionCount
            + _churnContext.MixedCollisions.InactivePartitionCount;
    }

    private static void AdvanceMixedFrame(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static void ValidateCandidates(GravitasWorldContext context, string scenario)
    {
        if (context.MixedCollisions.LastBroadPhaseCandidateCount == 0)
        {
            throw new InvalidOperationException(
                $"Mixed broad-phase benchmark setup produced no {scenario} candidates.");
        }
    }

    private static GravitasWorldContext CreateMixedContext(int extentX, int extentZ, bool clearAllPools = false)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        if (!context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)extentX, (Fixed64)4, (Fixed64)extentZ)),
            out _))
        {
            throw new InvalidOperationException("Unable to create mixed broad-phase benchmark grid.");
        }

        return context;
    }

    private static LSSphereCollider CreateTriggerSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var collider = new LSSphereCollider
        {
            IsTrigger = true
        };
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static SolidBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Dynamic);
        return body;
    }

    private static Vector2d SparsePositionForIndex(int index)
    {
        const int width = 64;
        int x = index % width;
        int z = index / width;
        return new Vector2d((Fixed64)(x * 3), (Fixed64)(z * 3));
    }

    private static Vector2d DensePositionForIndex(int index)
    {
        const int width = 16;
        int cluster = index / 4;
        int local = index % 4;
        int x = cluster % width;
        int z = cluster / width;
        Fixed64 localOffset = local == 0 ? Fixed64.Zero : Fixed64.FromFraction(local, 8);
        return new Vector2d((Fixed64)(x * 2) + localOffset, (Fixed64)(z * 2));
    }

    private static int SparseExtentX(int count)
    {
        int width = count < 64 ? count : 64;
        return 16 + (width * 3);
    }

    private static int SparseExtentZ(int count)
    {
        int rows = (count + 63) / 64;
        return 16 + (rows * 3);
    }

    private static int DenseExtentX(int count)
    {
        int clusterCount = (count + 3) / 4;
        int columns = clusterCount < 16 ? clusterCount : 16;
        return 16 + (columns * 2);
    }

    private static int DenseExtentZ(int count)
    {
        int clusterCount = (count + 3) / 4;
        int rows = (clusterCount + 15) / 16;
        return 16 + (rows * 2);
    }
}
