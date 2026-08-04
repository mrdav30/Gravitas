using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using GridForge.Grids.Storage;
using GridForge.Grids.Topology;
using GridForge.Spatial;
using SwiftCollections;
using SwiftCollections.Diagnostics;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedSparseGridSpanBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _context;
    private SwiftList<PhysicsMixedHit> _hits;

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools: true);
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;

        Fixed64 extent = (Fixed64)100_000;
        SwiftThrowHelper.ThrowIfTrue(
            !_context.World.TryAddGrid(
                new GridConfiguration(
                    new Vector3d(-extent, -extent, -extent),
                    new Vector3d(extent, extent, extent),
                    topologyMetrics: GridTopologyMetrics.Rectangular(extent),
                    storageKind: GridStorageKind.Sparse),
                new[] { new VoxelIndex(1, 1, 1) },
                out _),
            message: "Unable to add extreme sparse-span benchmark grid.");

        var agent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        var body = new SolidBody(agent, new LSSphereCollider())
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Static);
        _context.Simulate();
        _hits = new SwiftList<PhysicsMixedHit>(1);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context?.Dispose();
        _context = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_ExtremeSparseGridSpan()
    {
        return _context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-200_000), Fixed64.Zero),
            new Vector2d((Fixed64)200_000, Fixed64.Zero),
            (Fixed64)100_000,
            Fixed64.Zero,
            Fixed64.One,
            IncludeLayerZero,
            _hits);
    }
}
