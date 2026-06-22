using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class QueryServiceBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _context;
    private GravitasWorldContext _overlappingContext;
    private GravitasWorldContext _meshContext;
    private GravitasWorldContext _capsuleSourceContext;
    private GravitasWorldContext _cuboidSourceContext;
    private GravitasWorldContext _cylinderSourceContext;
    private GravitasWorldContext _convexMeshSourceContext;
    private GravitasWorldContext _compoundSourceContext;
    private LSCapsuleCollider _capsuleSource;
    private LSCuboidCollider _cuboidSource;
    private LSCylinderCollider _cylinderSource;
    private LSMeshCollider _convexMeshSource;
    private LSCompoundCollider _compoundSource;
    private Vector3d _rayStart;
    private Vector3d _rayEnd;
    private Vector3d _sourceSweepDisplacement;
    private SwiftList<Physics3DHit> _raycastHits;
    private SwiftList<Physics3DHit> _overlappingRaycastHits;
    private SwiftList<Physics3DHit> _circlecastHits;
    private SwiftList<Physics3DHit> _sweepSphereHits;
    private SwiftList<Physics3DHit> _meshSweepSphereHits;
    private SwiftList<Physics3DHit> _capsuleSourceHits;
    private SwiftList<Physics3DHit> _cuboidSourceHits;
    private SwiftList<Physics3DHit> _cylinderSourceHits;
    private SwiftList<Physics3DHit> _convexMeshSourceHits;
    private SwiftList<Physics3DHit> _compoundSourceHits;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extent = BenchmarkPhysicsScene.GridExtentForLine(ColliderCount);

        _context = BenchmarkPhysicsScene.CreateContext(extent, clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_context, ColliderCount);

        _overlappingContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_overlappingContext, ColliderCount);

        _meshContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateStaticMeshWallLine(_meshContext, ColliderCount);

        FixedQuaternion sideways = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);

        _capsuleSourceContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_capsuleSourceContext, ColliderCount);
        _capsuleSource = BenchmarkPhysicsScene.CreateDynamicCapsule(
            _capsuleSourceContext,
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            sideways);

        _cuboidSourceContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_cuboidSourceContext, ColliderCount);
        _cuboidSource = BenchmarkPhysicsScene.CreateDynamicCuboid(
            _cuboidSourceContext,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));

        _cylinderSourceContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_cylinderSourceContext, ColliderCount);
        _cylinderSource = BenchmarkPhysicsScene.CreateDynamicCylinder(
            _cylinderSourceContext,
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            sideways);

        _convexMeshSourceContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_convexMeshSourceContext, ColliderCount);
        _convexMeshSource = BenchmarkPhysicsScene.CreateDynamicConvexCube(
            _convexMeshSourceContext,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));

        _compoundSourceContext = BenchmarkPhysicsScene.CreateContext(extent);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_compoundSourceContext, ColliderCount);
        _compoundSource = BenchmarkPhysicsScene.CreateDynamicConvexMeshCompound(
            _compoundSourceContext,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));

        _rayStart = new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        _rayEnd = new Vector3d((Fixed64)(ColliderCount * 2), Fixed64.Zero, Fixed64.Zero);
        _sourceSweepDisplacement = new Vector3d((Fixed64)(ColliderCount * 2 + 3), Fixed64.Zero, Fixed64.Zero);
        _raycastHits = new SwiftList<Physics3DHit>(ColliderCount);
        _overlappingRaycastHits = new SwiftList<Physics3DHit>(ColliderCount);
        _circlecastHits = new SwiftList<Physics3DHit>(ColliderCount);
        _sweepSphereHits = new SwiftList<Physics3DHit>(ColliderCount);
        _meshSweepSphereHits = new SwiftList<Physics3DHit>(ColliderCount);
        _capsuleSourceHits = new SwiftList<Physics3DHit>(ColliderCount);
        _cuboidSourceHits = new SwiftList<Physics3DHit>(ColliderCount);
        _cylinderSourceHits = new SwiftList<Physics3DHit>(ColliderCount);
        _convexMeshSourceHits = new SwiftList<Physics3DHit>(ColliderCount);
        _compoundSourceHits = new SwiftList<Physics3DHit>(ColliderCount);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _overlappingContext.Dispose();
        _meshContext.Dispose();
        _capsuleSourceContext.Dispose();
        _cuboidSourceContext.Dispose();
        _cylinderSourceContext.Dispose();
        _convexMeshSourceContext.Dispose();
        _compoundSourceContext.Dispose();
        _context = null;
        _overlappingContext = null;
        _meshContext = null;
        _capsuleSourceContext = null;
        _cuboidSourceContext = null;
        _cylinderSourceContext = null;
        _capsuleSource = null;
        _cuboidSource = null;
        _cylinderSource = null;
        _convexMeshSourceContext = null;
        _compoundSourceContext = null;
        _convexMeshSource = null;
        _compoundSource = null;
        _raycastHits = null;
        _overlappingRaycastHits = null;
        _circlecastHits = null;
        _sweepSphereHits = null;
        _meshSweepSphereHits = null;
        _capsuleSourceHits = null;
        _cuboidSourceHits = null;
        _cylinderSourceHits = null;
        _convexMeshSourceHits = null;
        _compoundSourceHits = null;
    }

    [Benchmark]
    public int RaycastAllAcrossPopulatedContext() =>
        CountRaycastHits(_context, _raycastHits);

    [Benchmark]
    public int OverlapCircleAllAcrossPopulatedContext() =>
        _context.Query3D.OverlapCircleAll(Vector3d.Zero, (Fixed64)4, IncludeLayerZero, _circlecastHits);

    [Benchmark]
    public bool DirectionalOverlapCircleAcrossPopulatedContext() =>
        _context.Query3D.OverlapCircleInDirection(
            Vector3d.Zero,
            (Fixed64)4,
            Vector3d.Right,
            out _,
            (Fixed64)(ColliderCount * 2),
            IncludeLayerZero);

    [Benchmark]
    public int RaycastAcrossTwoOverlappingContexts() =>
        CountRaycastHits(_context, _raycastHits)
        + CountRaycastHits(_overlappingContext, _overlappingRaycastHits);

    [Benchmark]
    public int SweepSphereAllAcrossPopulatedContext() =>
        _context.Query3D.SweepSphereAll(_rayStart, _rayEnd, Fixed64.Half, IncludeLayerZero, _sweepSphereHits);

    [Benchmark]
    public int SweepSphereAllAcrossMeshTargetContext() =>
        _meshContext.Query3D.SweepSphereAll(_rayStart, _rayEnd, Fixed64.Half, IncludeLayerZero, _meshSweepSphereHits);

    [Benchmark]
    public int SweepCapsuleAllAcrossSphereTargets() =>
        _capsuleSourceContext.Query3D.SweepCapsuleAll(
            _capsuleSource,
            _sourceSweepDisplacement,
            IncludeLayerZero,
            _capsuleSourceHits);

    [Benchmark]
    public int SweepCuboidAllAcrossSphereTargets() =>
        _cuboidSourceContext.Query3D.SweepCuboidAll(
            _cuboidSource,
            _sourceSweepDisplacement,
            IncludeLayerZero,
            _cuboidSourceHits);

    [Benchmark]
    public int SweepCylinderAllAcrossSphereTargets() =>
        _cylinderSourceContext.Query3D.SweepCylinderAll(
            _cylinderSource,
            _sourceSweepDisplacement,
            IncludeLayerZero,
            _cylinderSourceHits);

    [Benchmark]
    public int SweepConvexMeshAllAcrossSphereTargets() =>
        _convexMeshSourceContext.Query3D.SweepConvexMeshAll(
            _convexMeshSource,
            _sourceSweepDisplacement,
            IncludeLayerZero,
            _convexMeshSourceHits);

    [Benchmark]
    public int SweepCompoundAllAcrossSphereTargets() =>
        _compoundSourceContext.Query3D.SweepCompoundAll(
            _compoundSource,
            _sourceSweepDisplacement,
            IncludeLayerZero,
            _compoundSourceHits);

    private int CountRaycastHits(GravitasWorldContext context, SwiftList<Physics3DHit> results)
    {
        return context.Query3D.RaycastAll(_rayStart, _rayEnd, IncludeLayerZero, results);
    }
}
