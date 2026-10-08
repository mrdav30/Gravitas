using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ConvexMeshInitialContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSCuboidCollider _source;
    private SwiftList<Physics3DHit> _hits;
    private Vector3d _travel;

    // Direct convex, direct concave, and a compound convex mesh leaf.
    [Params(0, 1, 2)]
    public int TargetKind { get; set; }

    [Params(false, true)]
    public bool InitialContact { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(8, clearAllPools: true);
        LSMeshCollider mesh = BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(
            1, TargetKind == 1 ? MeshColliderMode.Concave : MeshColliderMode.Convex);
        LSCollider target = TargetKind == 2
            ? new LSCompoundCollider(CompoundColliderPart.ConvexMesh(
                mesh.Mesh.LocalVertices.ToArray(), mesh.Mesh.Triangles.ToArray(),
                Vector3d.Zero, MeshInertiaPolicy.SurfaceApproximation))
            : mesh;
        BenchmarkPhysicsScene.CreateStaticCollider(_context, target, Vector3d.Zero);
        _source = BenchmarkPhysicsScene.CreateDynamicCuboid(
            _context, Vector3d.Left * (InitialContact ? Fixed64.Quarter : (Fixed64)3));
        _context.Simulate();
        _hits = new SwiftList<Physics3DHit>(1);
        _travel = InitialContact ? Vector3d.Left : Vector3d.Right * (Fixed64)6;
        // Retain worker buffers before measuring the complete public query.
        SweepCuboidAll();
    }

    [Benchmark]
    public int SweepCuboidAll() => _context.Query3D.SweepCuboidAll(
        _source, _travel, PhysicsLayerMask.All, _hits);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
