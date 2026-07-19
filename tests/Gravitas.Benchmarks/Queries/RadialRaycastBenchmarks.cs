using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class RadialRaycastBenchmarks
{
    private readonly RaycastSegmentWorker _worker = new();
    private SwiftList<Vector3d> _hits = new(2);
    private FixedBoundSphere _sphere;
    private LSCircleCollider2D _circle = null!;
    private Vector3d _mixedStart;
    private Vector3d _mixedEnd;
    private Fixed64 _mixedLength;
    private Fixed64 _mixedSphereRadius;
    private PhysicsMixedHit _mixedHit;

    [Params(1, 100_000)]
    public int Scale { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        Fixed64 scale = (Fixed64)Scale;
        _worker.PrepareSegmentCheck(
            new Vector3d(-scale * 2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(scale * 2, Fixed64.Zero, Fixed64.Zero));
        _sphere = new FixedBoundSphere(
            new Vector3d(Fixed64.Zero, scale * Fixed64.FromFraction(3, 5), Fixed64.Zero),
            scale);
        _circle = new LSCircleCollider2D(scale * Fixed64.FromFraction(3, 4));
        _mixedStart = new Vector3d(-scale * 2, Fixed64.Zero, scale * Fixed64.FromFraction(3, 5));
        _mixedEnd = new Vector3d(scale * 2, Fixed64.Zero, scale * Fixed64.FromFraction(3, 5));
        _mixedLength = scale * 4;
        _mixedSphereRadius = scale * Fixed64.FromFraction(1, 4);

        if (!SphereSegmentInterval() || _hits.Count != 2)
            throw new System.InvalidOperationException("The radial benchmark scenario must produce two hits.");
        if (!MixedCircleSlabInterval())
            throw new System.InvalidOperationException("The mixed radial benchmark scenario must produce a hit.");
    }

    [Benchmark]
    public bool SphereSegmentInterval()
    {
        _hits.FastClear();
        return _worker.CheckSphereOverlaps(_sphere, ref _hits);
    }

    [Benchmark]
    public bool MixedCircleSlabInterval() =>
        GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
            _mixedStart,
            _mixedEnd,
            Vector3d.Right,
            _mixedLength,
            _mixedSphereRadius,
            _circle,
            out _mixedHit);
}
