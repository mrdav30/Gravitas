using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CoplanarPatchInitialSweepBenchmarks
{
    private GravitasWorldContext _context;
    private LSCapsuleCollider _source;
    private readonly SwiftList<Physics3DHit> _hits = new(1);

    [Params(2, 8)]
    public int Subdivision { get; set; }

    [Params(false, true)]
    public bool Notched { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(8, clearAllPools: true);
        LSMeshCollider template = BenchmarkPhysicsScene.CreateSubdividedVerticalQuadMesh(Subdivision);
        Vector3d[] vertices = template.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++) vertices[i] *= (Fixed64)4;
        ReadOnlySpan<int> authored = template.Mesh.Triangles;
        var triangles = new SwiftList<int>(authored.Length);
        for (int i = 0; i < authored.Length; i += 3)
        {
            Vector3d sum = vertices[authored[i]] + vertices[authored[i + 1]] + vertices[authored[i + 2]];
            if (Notched && sum.Y > Fixed64.Zero && sum.Z > Fixed64.Zero) continue;
            triangles.Add(authored[i]); triangles.Add(authored[i + 1]); triangles.Add(authored[i + 2]);
        }
        var usedVertices = new SwiftList<Vector3d>(vertices.Length);
        var remap = new int[vertices.Length];
        Array.Fill(remap, -1);
        for (int i = 0; i < triangles.Count; i++)
        {
            int index = triangles[i];
            if (remap[index] < 0) { remap[index] = usedVertices.Count; usedVertices.Add(vertices[index]); }
            triangles[i] = remap[index];
        }
        var target = new LSMeshCollider(usedVertices.ToArray(), triangles.ToArray(), MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        BenchmarkPhysicsScene.CreateStaticCollider(_context, target, Vector3d.Zero);
        _source = BenchmarkPhysicsScene.CreateDynamicCapsule(_context,
            new Vector3d(-Fixed64.Quarter, -Fixed64.FromFraction(3, 10), -Fixed64.FromFraction(3, 10)),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi));
        _source.Size = new Vector3d(1, 2, 1);
        _context.Simulate();
        if (SweepCapsuleAll() != 1)
            throw new InvalidOperationException("The subdivided filled patch must intersect the initial capsule.");
    }

    [Benchmark]
    public int SweepCapsuleAll() => _context.Query3D.SweepCapsuleAll(
        _source, Vector3d.Left, PhysicsLayerMask.All, _hits);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
