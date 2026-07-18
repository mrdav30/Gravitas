using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using SwiftCollections;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class OrientedBoxRaycastBenchmarks
{
    private GravitasWorldContext _context = null!;
    private LSCuboidCollider _box = null!;
    private RaycastSegmentWorker _worker = null!;
    private SwiftList<Vector3d> _hits = null!;
    private Vector3d _worldStart;
    private Vector3d _worldEnd;

    [Params(1, 50_000)]
    public int HalfExtent { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = CreateContext3D(4, 4);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)23,
            (Fixed64)37,
            (Fixed64)11);
        var agent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        agent.Transform.LocalRotation = rotation;
        var body = new SolidBody(agent, new LSCuboidCollider())
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, rotation, isDynamic: false);
        _box = (LSCuboidCollider)body.Collider;

        Fixed64 halfExtent = (Fixed64)HalfExtent;
        _box.Size = Vector3d.One * (halfExtent * 2);
        _box.RebuildRuntimeShapeOnly();
        Vector3d localStart = new(-halfExtent * 4, -halfExtent * 2, halfExtent);
        Vector3d localEnd = new(halfExtent * 4, halfExtent * 6, halfExtent);
        _worldStart = rotation * localStart;
        _worldEnd = rotation * localEnd;
        _worker = new RaycastSegmentWorker();
        _hits = new SwiftList<Vector3d>();

        if (!TangentSegmentAgainstOrientedBox() || _hits.Count != 1)
            throw new System.InvalidOperationException("The benchmark scenario must produce exactly one tangent hit.");
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    [Benchmark]
    public bool TangentSegmentAgainstOrientedBox()
    {
        _hits.FastClear();
        _worker.PrepareSegmentCheck(_worldStart, _worldEnd);
        return _worker.CheckOBBoxOverlaps(_box, ref _hits);
    }
}
