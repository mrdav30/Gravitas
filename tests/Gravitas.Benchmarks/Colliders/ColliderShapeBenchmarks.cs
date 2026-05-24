using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ColliderShapeBenchmarks
{
    private GravitasWorldContext _context;
    private LSCapsuleCollider[] _capsules;
    private StiffBody[] _bodies;
    private Vector3d[] _meshVertices;
    private int[] _meshTriangles;
    private SwiftList<int> _meshHits;
    private int _tick;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount),
            clearAllPools: true);
        _capsules = new LSCapsuleCollider[ColliderCount];
        _bodies = new StiffBody[ColliderCount];

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector3d position = new(i % 8 * 2, 0, i / 8 * 2);
            var agent = new BenchmarkMatterAgent(_context, position);
            var collider = new LSCapsuleCollider();
            var body = new StiffBody(agent, collider)
            {
                Mass = Fixed64.One,
                PreventAngularForces = true
            };

            body.Initialize(position, FixedQuaternion.Identity);
            collider.Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One);
            collider.Simulate();

            _capsules[i] = collider;
            _bodies[i] = body;
        }

        _meshVertices = new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up,
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero)
        };
        _meshTriangles = new[] { 0, 1, 2, 1, 3, 2 };
        _meshHits = new SwiftList<int>(2);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _capsules = null;
        _bodies = null;
        _meshVertices = null;
        _meshTriangles = null;
        _meshHits = null;
    }

    [Benchmark]
    public uint RebuildCapsuleRuntimeShapeState()
    {
        bool evenTick = (_tick & 1) == 0;
        Fixed64 radius = evenTick ? Fixed64.Fraction(1, 4) : Fixed64.Fraction(1, 3);
        Fixed64 height = evenTick ? (Fixed64)3 : (Fixed64)4;
        FixedQuaternion rotation = evenTick
            ? FixedQuaternion.Identity
            : FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);

        uint versionSum = 0;
        for (int i = 0; i < _capsules.Length; i++)
        {
            _capsules[i].Radius = radius;
            _capsules[i].Size = new Vector3d(Fixed64.One, height, Fixed64.One);
            _bodies[i].SetRotation(rotation);
            _capsules[i].Simulate();
            versionSum += _capsules[i].RuntimeShapeVersion;
        }

        _tick++;
        return versionSum;
    }

    [Benchmark]
    public int BuildValidatedMeshTriangleBVH()
    {
        var mesh = new PhysicsMesh(
            _meshVertices,
            _meshTriangles,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        _meshHits.FastClear();
        mesh.TriangleBVH.Query(
            new FixedBoundVolume(Vector3d.Zero, Vector3d.One),
            _meshHits);
        return _meshHits.Count;
    }
}
