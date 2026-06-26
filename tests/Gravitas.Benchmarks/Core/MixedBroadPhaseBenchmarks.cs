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

    [Params(64, 1024, 4096)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extentX = ExtentForCount(ColliderCount);
        int extentZ = ExtentForRows(ColliderCount);
        _sparseContext = CreateMixedContext(extentX, extentZ, clearAllPools: true);
        _denseContext = CreateMixedContext(64, 64);
        _churnContext = CreateMixedContext(extentX + 64, extentZ + 64);
        _churnBodies2D = new SwiftList<SolidBody2D>(ColliderCount);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector2d position = SparsePositionForIndex(i);
            _ = CreateSphere3D(_sparseContext, new Vector3d(position.X, Fixed64.Zero, position.Y), immovable: false);
            _ = CreateCircle2D(_sparseContext, position, immovable: true);

            Vector2d churnPosition = position + new Vector2d((Fixed64)16, Fixed64.Zero);
            _ = CreateSphere3D(_churnContext, new Vector3d(churnPosition.X, Fixed64.Zero, churnPosition.Y), immovable: false);
            _churnBodies2D.Add(CreateCircle2D(_churnContext, churnPosition, immovable: true));
        }

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector2d position = DensePositionForIndex(i);
            _ = CreateSphere3D(_denseContext, new Vector3d(position.X, Fixed64.Zero, position.Y), immovable: false);
            _ = CreateCircle2D(_denseContext, position, immovable: true);
        }

        _sparseContext.Simulate();
        _denseContext.Simulate();
        _churnContext.Simulate();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sparseContext?.Dispose();
        _denseContext?.Dispose();
        _churnContext?.Dispose();
        _sparseContext = null;
        _denseContext = null;
        _churnContext = null;
        _churnBodies2D = null;
    }

    [Benchmark]
    public int SparseCandidateGathering()
    {
        _sparseContext.Simulate();
        return _sparseContext.MixedCollisions.LastBroadPhaseCandidateCount;
    }

    [Benchmark]
    public int DenseCandidateGathering()
    {
        _denseContext.Simulate();
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
        _churnContext.Simulate();
        return _churnContext.MixedCollisions.RetainedPartitionCount
            + _churnContext.MixedCollisions.InactivePartitionCount;
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

    private static SolidBody CreateSphere3D(GravitasWorldContext context, Vector3d position, bool immovable)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static SolidBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position, bool immovable)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
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

    private static int ExtentForCount(int count)
    {
        int width = count < 64 ? count : 64;
        return 16 + (width * 3);
    }

    private static int ExtentForRows(int count)
    {
        int rows = (count + 63) / 64;
        return 16 + (rows * 3);
    }
}
