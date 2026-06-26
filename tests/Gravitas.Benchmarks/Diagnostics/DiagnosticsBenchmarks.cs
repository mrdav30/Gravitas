using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class DiagnosticsBenchmarks
{
    private GravitasWorldContext _disabledContext;
    private GravitasWorldContext _enabledEventContext;
    private GravitasWorldContext _enabledDrawContext;
    private GravitasWorldContext _disabledMeshContext;
    private GravitasWorldContext _enabledMeshContext;
    private SwiftList<SolidBody> _disabledBodies;
    private SwiftList<SolidBody> _enabledEventBodies;
    private SwiftList<SolidBody> _enabledDrawBodies;
    private LSMeshCollider _disabledMeshCollider;
    private LSMeshCollider _enabledMeshCollider;

    [Params(64)]
    public int ColliderCount { get; set; }

    [Params(128)]
    public int MeshQuadCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount);

        _disabledContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        _disabledBodies = new SwiftList<SolidBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_disabledContext, ColliderCount, _disabledBodies);

        _enabledEventContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _enabledEventBodies = new SwiftList<SolidBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_enabledEventContext, ColliderCount, _enabledEventBodies);
        _enabledEventContext.Diagnostics.Enable(eventCapacity: ColliderCount * 2, drawCommandCapacity: 0);

        _enabledDrawContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        _enabledDrawBodies = new SwiftList<SolidBody>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_enabledDrawContext, ColliderCount, _enabledDrawBodies);
        _enabledDrawContext.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: ColliderCount);

        int meshExtent = BenchmarkPhysicsScene.GridExtentForLine(MeshQuadCount + 1);
        int meshTriangleCount = MeshQuadCount * 2;
        _disabledMeshContext = BenchmarkPhysicsScene.CreateContext(meshExtent);
        _disabledMeshCollider = CreateStaticMeshCollider(_disabledMeshContext, MeshQuadCount);

        _enabledMeshContext = BenchmarkPhysicsScene.CreateContext(meshExtent);
        _enabledMeshCollider = CreateStaticMeshCollider(_enabledMeshContext, MeshQuadCount);
        _enabledMeshContext.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: meshTriangleCount);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _disabledContext.Dispose();
        _enabledEventContext.Dispose();
        _enabledDrawContext.Dispose();
        _disabledMeshContext.Dispose();
        _enabledMeshContext.Dispose();

        _disabledContext = null;
        _enabledEventContext = null;
        _enabledDrawContext = null;
        _disabledMeshContext = null;
        _enabledMeshContext = null;
        _disabledBodies = null;
        _enabledEventBodies = null;
        _enabledDrawBodies = null;
        _disabledMeshCollider = null;
        _enabledMeshCollider = null;
    }

    [Benchmark(Baseline = true)]
    public int ForceAndTorqueDiagnosticsDisabled()
    {
        ApplyForceAndTorque(_disabledBodies);
        return _disabledContext.Diagnostics.EventCount;
    }

    [Benchmark]
    public int ForceAndTorqueDiagnosticsEnabled()
    {
        _enabledEventContext.Diagnostics.Clear();
        ApplyForceAndTorque(_enabledEventBodies);
        return _enabledEventContext.Diagnostics.EventCount;
    }

    [Benchmark]
    public int ColliderCaptureDiagnosticsDisabled()
    {
        CaptureColliders(_disabledContext, _disabledBodies);
        return _disabledContext.Diagnostics.DrawCommandCount;
    }

    [Benchmark]
    public int ColliderCaptureDiagnosticsEnabled()
    {
        _enabledDrawContext.Diagnostics.Clear();
        CaptureColliders(_enabledDrawContext, _enabledDrawBodies);
        return _enabledDrawContext.Diagnostics.DrawCommandCount;
    }

    [Benchmark]
    public int MeshCaptureDiagnosticsDisabled()
    {
        _disabledMeshContext.Diagnostics.CaptureCollider(_disabledMeshCollider, GravitasDiagnosticColor.White);
        return _disabledMeshContext.Diagnostics.DrawCommandCount;
    }

    [Benchmark]
    public int MeshCaptureDiagnosticsEnabled()
    {
        _enabledMeshContext.Diagnostics.Clear();
        _enabledMeshContext.Diagnostics.CaptureCollider(_enabledMeshCollider, GravitasDiagnosticColor.White);
        return _enabledMeshContext.Diagnostics.DrawCommandCount;
    }

    private static void ApplyForceAndTorque(SwiftList<SolidBody> bodies)
    {
        var force = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        var torque = new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero);

        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody body = bodies[i];
            body.AddForce(force);
            body.AddTorque(torque);
        }
    }

    private static void CaptureColliders(GravitasWorldContext context, SwiftList<SolidBody> bodies)
    {
        for (int i = 0; i < bodies.Count; i++)
            context.Diagnostics.CaptureCollider(bodies[i].Collider, GravitasDiagnosticColor.Cyan);
    }

    private static LSMeshCollider CreateStaticMeshCollider(GravitasWorldContext context, int quadCount)
    {
        var agent = new BenchmarkMatterAgent(context, Vector3d.Zero);
        LSMeshCollider collider = CreateStripMesh(quadCount);
        collider.InitializeWithNoBody(agent);

        return collider;
    }

    private static LSMeshCollider CreateStripMesh(int quadCount)
    {
        var vertices = new Vector3d[(quadCount + 1) * 2];
        var triangles = new int[quadCount * 6];

        for (int i = 0; i <= quadCount; i++)
        {
            vertices[i * 2] = new Vector3d((Fixed64)i, Fixed64.Zero, Fixed64.Zero);
            vertices[i * 2 + 1] = new Vector3d((Fixed64)i, Fixed64.Zero, Fixed64.One);
        }

        for (int i = 0; i < quadCount; i++)
        {
            int vertex = i * 2;
            int triangle = i * 6;
            triangles[triangle] = vertex;
            triangles[triangle + 1] = vertex + 1;
            triangles[triangle + 2] = vertex + 2;
            triangles[triangle + 3] = vertex + 2;
            triangles[triangle + 4] = vertex + 1;
            triangles[triangle + 5] = vertex + 3;
        }

        return new LSMeshCollider(vertices, triangles);
    }
}
