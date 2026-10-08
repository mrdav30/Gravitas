using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshSphereContactBenchmarks
{
    private GravitasWorldContext _context;
    private SwiftList<Physics3DHit> _hits;
    private Vector3d _start;
    private Vector3d _end;

    [Params(false, true)]
    public bool Compound { get; set; }

    [Params(false, true)]
    public bool InitialOverlap { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(8, clearAllPools: true);
        LSMeshCollider mesh = BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(1);
        LSCollider target = Compound
            ? new LSCompoundCollider(CompoundColliderPart.ConvexMesh(
                mesh.Mesh.LocalVertices.ToArray(),
                mesh.Mesh.Triangles.ToArray(),
                Vector3d.Zero,
                MeshInertiaPolicy.SurfaceApproximation))
            : mesh;
        BenchmarkPhysicsScene.CreateStaticCollider(_context, target, Vector3d.Zero);
        _context.Simulate();
        _hits = new SwiftList<Physics3DHit>(1);
        _start = Vector3d.Left * (InitialOverlap ? Fixed64.FromFraction(1, 4) : (Fixed64)3);
        _end = InitialOverlap ? Vector3d.Left : Vector3d.Right * (Fixed64)3;
    }

    [Benchmark]
    public int SweepSphereAll() => _context.Query3D.SweepSphereAll(
        _start, _end, Fixed64.Half, PhysicsLayerMask.All, _hits);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
