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
public class MixedQueryBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _sparseContext;
    private GravitasWorldContext _denseContext;
    private GravitasWorldContext _cornerMissContext;
    private GravitasWorldContext _denseCuboidContext;
    private GravitasWorldContext _denseCapsuleContext;
    private GravitasWorldContext _denseCylinderContext;
    private SwiftList<PhysicsMixedHit> _hits;
    private Vector2d _sparseEnd;
    private Vector2d _denseEnd;
    private Vector2d _cornerMissEnd;

    [Params(64, 1024)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int sparseExtentX = 16 + (ColliderCount * 3);
        int denseExtentX = 16 + ColliderCount;

        _sparseContext = CreateMixedContext(sparseExtentX, 16, clearAllPools: true);
        _denseContext = CreateMixedContext(denseExtentX, 16);
        _cornerMissContext = CreateMixedContext(denseExtentX, 16);
        _denseCuboidContext = CreateMixedContext(denseExtentX, 16);
        _denseCapsuleContext = CreateMixedContext(denseExtentX, 16);
        _denseCylinderContext = CreateMixedContext(denseExtentX, 16);
        _hits = new SwiftList<PhysicsMixedHit>(ColliderCount);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector3d densePosition = new((Fixed64)i, Fixed64.Zero, DenseZForIndex(i));
            _ = CreateSphere3D(_sparseContext, new Vector3d((Fixed64)(i * 3), Fixed64.Zero, Fixed64.Zero));
            _ = CreateSphere3D(_denseContext, densePosition);
            _ = CreateSphere3D(
                _cornerMissContext,
                new Vector3d((Fixed64)i, Fixed64.FromFraction(9, 10), Fixed64.Zero));
            _ = CreateStatic3D(_denseCuboidContext, new LSCuboidCollider(), densePosition);
            _ = CreateStatic3D(
                _denseCapsuleContext,
                new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition);
            _ = CreateStatic3D(
                _denseCylinderContext,
                new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition);
        }

        _sparseContext.Simulate();
        _denseContext.Simulate();
        _cornerMissContext.Simulate();
        _denseCuboidContext.Simulate();
        _denseCapsuleContext.Simulate();
        _denseCylinderContext.Simulate();

        _sparseEnd = new Vector2d((Fixed64)(ColliderCount * 3), Fixed64.Zero);
        _denseEnd = new Vector2d((Fixed64)ColliderCount, Fixed64.Zero);
        _cornerMissEnd = new Vector2d((Fixed64)ColliderCount, Fixed64.FromFraction(9, 10));
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sparseContext?.Dispose();
        _denseContext?.Dispose();
        _cornerMissContext?.Dispose();
        _denseCuboidContext?.Dispose();
        _denseCapsuleContext?.Dispose();
        _denseCylinderContext?.Dispose();
        _sparseContext = null;
        _denseContext = null;
        _cornerMissContext = null;
        _denseCuboidContext = null;
        _denseCapsuleContext = null;
        _denseCylinderContext = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_SparseSphereTargets()
    {
        return _sparseContext.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            _sparseEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseSphereTargets()
    {
        return _denseContext.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            _denseEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_CornerProxyMissSphereTargets()
    {
        return _cornerMissContext.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.FromFraction(9, 10)),
            _cornerMissEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCuboidTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseCuboidContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCuboidTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseCuboidContext);
        return _denseCuboidContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCapsuleTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseCapsuleContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCapsuleTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseCapsuleContext);
        return _denseCapsuleContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCylinderTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseCylinderContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCylinderTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseCylinderContext);
        return _denseCylinderContext.QueryMixed.LastQueryCandidateCount;
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
            throw new InvalidOperationException("Unable to create mixed query benchmark grid.");
        }

        return context;
    }

    private static StiffBody CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        return CreateStatic3D(context, new LSSphereCollider(), position);
    }

    private static StiffBody CreateStatic3D(GravitasWorldContext context, LSCollider collider, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private int SweepCircleAgainstDensePrimitive3D(GravitasWorldContext context)
    {
        return context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            _denseEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    private static Fixed64 DenseZForIndex(int index)
    {
        int lane = index % 4;
        return lane switch
        {
            0 => Fixed64.Zero,
            1 => Fixed64.FromFraction(1, 4),
            2 => -Fixed64.FromFraction(1, 4),
            _ => Fixed64.FromFraction(3, 4)
        };
    }
}
