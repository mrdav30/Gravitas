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
    private readonly SweptSphereQueryWorker _sweptSphereWorker = new();
    private readonly SweptSphereQueryWorker _roundedRimWorker = new();
    private SwiftList<Vector3d> _hits = new(2);
    private GravitasWorldContext _context = null!;
    private FixedBoundSphere _sphere;
    private LSCapsuleCollider _capsule = null!;
    private LSCylinderCollider _cylinder = null!;
    private LSCylinderCollider _roundedRimCylinder = null!;
    private LSCircleCollider2D _circle = null!;
    private Vector3d _sweepCenter;
    private Fixed64 _sweepDistance;
    private Vector3d _mixedStart;
    private Vector3d _mixedEnd;
    private Fixed64 _mixedLength;
    private Fixed64 _mixedSphereRadius;
    private PhysicsMixedHit _mixedHit;
    private Vector3d _mixedRimStart;
    private Vector3d _mixedRimEnd;
    private Fixed64 _mixedRimLength;

    [Params(1, 100_000)]
    public int Scale { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        Fixed64 scale = (Fixed64)Scale;
        _context = BenchmarkPhysicsScene.CreateContext(8, clearAllPools: true);
        _capsule = BenchmarkPhysicsScene.CreateDynamicCapsule(
            _context,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        _cylinder = BenchmarkPhysicsScene.CreateDynamicCylinder(
            _context,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        _worker.PrepareSegmentCheck(
            new Vector3d(-scale * 2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(scale * 2, Fixed64.Zero, Fixed64.Zero));
        _sweptSphereWorker.Prepare(
            new Vector3d(-scale * 2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(scale * 2, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Quarter);
        _roundedRimCylinder = BenchmarkPhysicsScene.CreateDynamicCylinder(
            _context,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        // Keep the registered footprint small; this benchmark calls the reducer directly.
        _roundedRimCylinder.Radius = scale * Fixed64.Half;
        _roundedRimCylinder.Size = new Vector3d(scale, scale, scale);
        _roundedRimCylinder.RebuildRuntimeShapeOnly();
        _roundedRimWorker.Prepare(
            new Vector3d(scale * 2, scale * Fixed64.FromFraction(9, 10), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, scale * Fixed64.FromFraction(9, 10), Fixed64.Zero),
            scale * Fixed64.Half);
        _sphere = new FixedBoundSphere(
            new Vector3d(Fixed64.Zero, scale * Fixed64.FromFraction(3, 5), Fixed64.Zero),
            scale);
        _circle = new LSCircleCollider2D(scale * Fixed64.FromFraction(3, 4));
        _circle.MixedHalfThicknessOverride = scale;
        if (!_circle.RebuildRuntimeShapeOnly())
            throw new System.InvalidOperationException("The mixed circle benchmark target must build its runtime slab.");
        _mixedStart = new Vector3d(-scale * 2, Fixed64.Zero, scale * Fixed64.FromFraction(3, 5));
        _mixedEnd = new Vector3d(scale * 2, Fixed64.Zero, scale * Fixed64.FromFraction(3, 5));
        _mixedLength = scale * 4;
        _mixedSphereRadius = scale * Fixed64.FromFraction(1, 4);
        _mixedRimStart = new Vector3d(scale * 2, scale * Fixed64.FromFraction(6, 5), Fixed64.Zero);
        _mixedRimEnd = new Vector3d(scale * Fixed64.Half, scale * Fixed64.FromFraction(6, 5), Fixed64.Zero);
        _mixedRimLength = scale * Fixed64.FromFraction(3, 2);

        if (!SphereSegmentInterval() || _hits.Count != 2)
            throw new System.InvalidOperationException("The radial benchmark scenario must produce two hits.");
        if (!CapsuleSegmentInterval() || _hits.Count != 2)
            throw new System.InvalidOperationException("The capsule-ray benchmark scenario must produce two hits.");
        if (!CylinderSegmentInterval() || _hits.Count != 2)
            throw new System.InvalidOperationException("The cylinder-ray benchmark scenario must produce two hits.");
        if (!SweptSphereAgainstCapsule() || !SweptSphereAgainstCylinder())
            throw new System.InvalidOperationException("The finite-axis sweep benchmark scenarios must produce hits.");
        if (!SweptSphereAgainstRoundedCylinderRim())
            throw new System.InvalidOperationException("The rounded-cylinder rim benchmark scenario must produce a hit.");
        if (!MixedCircleSlabInterval())
            throw new System.InvalidOperationException("The mixed radial benchmark scenario must produce a hit.");
        if (!MixedCircleSlabRoundedRimInterval())
            throw new System.InvalidOperationException("The mixed rounded-rim benchmark scenario must produce a hit.");
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    [Benchmark]
    public bool SphereSegmentInterval()
    {
        _hits.FastClear();
        return _worker.CheckSphereOverlaps(_sphere, ref _hits);
    }

    [Benchmark]
    public bool CapsuleSegmentInterval()
    {
        _hits.FastClear();
        return _worker.CheckCapsuleOverlaps(_capsule, ref _hits);
    }

    [Benchmark]
    public bool CylinderSegmentInterval()
    {
        _hits.FastClear();
        return _worker.CheckCylinderOverlaps(_cylinder, ref _hits);
    }

    [Benchmark]
    public bool SweptSphereAgainstCapsule() =>
        _sweptSphereWorker.TrySweep(_capsule, out _sweepCenter, out _sweepDistance);

    [Benchmark]
    public bool SweptSphereAgainstCylinder() =>
        _sweptSphereWorker.TrySweep(_cylinder, out _sweepCenter, out _sweepDistance);

    [Benchmark]
    public bool SweptSphereAgainstRoundedCylinderRim() =>
        _roundedRimWorker.TrySweep(_roundedRimCylinder, out _sweepCenter, out _sweepDistance);

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

    [Benchmark]
    public bool MixedCircleSlabRoundedRimInterval() =>
        GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
            _mixedRimStart,
            _mixedRimEnd,
            -Vector3d.Right,
            _mixedRimLength,
            _mixedSphereRadius,
            _circle,
            out _mixedHit);
}
