using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshQuery3DTriangleScalingBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _sweptSphereContext;
    private GravitasWorldContext _convexSourceContext;
    private LSMeshCollider _convexSource;
    private SwiftList<Physics3DHit> _hits;
    private Vector3d _start;
    private Vector3d _end;
    private Vector3d _displacement;

    [Params(8, 16, 32)]
    public int Subdivision { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _sweptSphereContext = BenchmarkPhysicsScene.CreateContext(8, clearAllPools: true);
        _convexSourceContext = BenchmarkPhysicsScene.CreateContext(8);

        BenchmarkPhysicsScene.CreateStaticCollider(
            _sweptSphereContext,
            BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(Subdivision),
            Vector3d.Zero);
        _convexSource = BenchmarkPhysicsScene.CreateDynamicConvexCube(
            _convexSourceContext,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        BenchmarkPhysicsScene.CreateStaticCollider(
            _convexSourceContext,
            BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(Subdivision, MeshColliderMode.Concave),
            Vector3d.Zero);

        _sweptSphereContext.Simulate();
        _convexSourceContext.Simulate();

        _hits = new SwiftList<Physics3DHit>(1);
        _start = new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero);
        _end = new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero);
        _displacement = new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sweptSphereContext?.Dispose();
        _convexSourceContext?.Dispose();
        _sweptSphereContext = null;
        _convexSourceContext = null;
        _convexSource = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepSphereAll_DenseTriangleMeshTarget()
    {
        return _sweptSphereContext.Query3D.SweepSphereAll(
            _start,
            _end,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepSphereAll_DenseTriangleMeshTarget_ColliderCandidateCount()
    {
        _ = SweepSphereAll_DenseTriangleMeshTarget();
        return _sweptSphereContext.Query3D.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepSphereAll_DenseTriangleMeshTarget_TriangleCandidateCount()
    {
        _ = SweepSphereAll_DenseTriangleMeshTarget();
        return _sweptSphereContext.Query3D.LastMeshTriangleCandidateCount;
    }

    [Benchmark]
    public int SweepConvexMeshAll_DenseConcaveTriangleMeshTarget()
    {
        return _convexSourceContext.Query3D.SweepConvexMeshAll(
            _convexSource,
            _displacement,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepConvexMeshAll_DenseConcaveTriangleMeshTarget_ColliderCandidateCount()
    {
        _ = SweepConvexMeshAll_DenseConcaveTriangleMeshTarget();
        return _convexSourceContext.Query3D.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepConvexMeshAll_DenseConcaveTriangleMeshTarget_TriangleCandidateCount()
    {
        _ = SweepConvexMeshAll_DenseConcaveTriangleMeshTarget();
        return _convexSourceContext.Query3D.LastMeshTriangleCandidateCount;
    }
}
