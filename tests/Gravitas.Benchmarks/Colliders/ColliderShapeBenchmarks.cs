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
    private LSMeshCollider _meshCollider;
    private StiffBody _meshBody;
    private SwiftList<int> _meshHits;
    private LSMeshCollider _concaveMeshCollider;
    private StiffBody _concaveMeshBody;
    private SwiftList<int> _concaveMeshHits;
    private LSCompoundCollider _compoundCollider;
    private StiffBody _compoundBody;
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

        var meshAgent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        _meshCollider = new LSMeshCollider(_meshVertices, _meshTriangles);
        _meshBody = new StiffBody(meshAgent, _meshCollider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = true
        };

        _meshBody.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        _meshCollider.Simulate();
        _meshCollider.GetTrianglesInBounds(new FixedBoundVolume(Vector3d.Zero, Vector3d.One), _meshHits);

        _concaveMeshHits = new SwiftList<int>(8);
        var concaveAgent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        _concaveMeshCollider = new LSMeshCollider(
            CreateUChannelVertices(),
            new[]
            {
                0, 1, 2, 2, 1, 3,
                4, 5, 6, 6, 5, 7,
                8, 9, 10, 10, 9, 11
            },
            MeshColliderMode.Concave);
        _concaveMeshBody = new StiffBody(concaveAgent, _concaveMeshCollider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = true
        };
        _concaveMeshBody.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        _concaveMeshCollider.Simulate();
        _concaveMeshCollider.GetTrianglesInBounds(
            new FixedBoundVolume(
                new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.One),
                new Vector3d((Fixed64)3, (Fixed64)2, (Fixed64)3)),
            _concaveMeshHits);

        var compoundAgent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        _compoundCollider = new LSCompoundCollider(
            new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero) }),
            new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero) }),
            new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero) }),
            new CompoundColliderPart(new LSCuboidCollider { LocalOffset = new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2) }));
        _compoundBody = new StiffBody(compoundAgent, _compoundCollider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = true
        };
        _compoundBody.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        _compoundCollider.Simulate();
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
        _meshCollider = null;
        _meshBody = null;
        _meshHits = null;
        _concaveMeshCollider = null;
        _concaveMeshBody = null;
        _concaveMeshHits = null;
        _compoundCollider = null;
        _compoundBody = null;
    }

    [Benchmark]
    public uint RebuildCapsuleRuntimeShapeState()
    {
        bool evenTick = (_tick & 1) == 0;
        Fixed64 radius = evenTick ? Fixed64.FromFraction(1, 4) : Fixed64.FromFraction(1, 3);
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

    [Benchmark]
    public int MoveMeshRuntimeShapeStateAndQueryTriangles()
    {
        Vector3d position = (_tick & 1) == 0
            ? Vector3d.Zero
            : new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);

        _meshBody.SetPosition(position);
        _meshCollider.Simulate();
        _meshCollider.GetTrianglesInBounds(
            new FixedBoundVolume(position, position + Vector3d.One),
            _meshHits);

        _tick++;
        return _meshHits.Count + _meshCollider.Mesh.TriangleBvhBuildCount;
    }

    [Benchmark]
    public int MoveDynamicConcaveMeshAndQueryTriangles()
    {
        Vector3d position = (_tick & 1) == 0
            ? Vector3d.Zero
            : new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);

        _concaveMeshBody.SetPosition(position);
        _concaveMeshCollider.Simulate();
        _concaveMeshCollider.GetTrianglesInBounds(
            new FixedBoundVolume(
                position + new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.One),
                position + new Vector3d((Fixed64)3, (Fixed64)2, (Fixed64)3)),
            _concaveMeshHits);

        _tick++;
        return _concaveMeshHits.Count + _concaveMeshCollider.Mesh.TriangleBvhBuildCount;
    }

    [Benchmark]
    public int MoveCompoundRuntimeShapeStateAcrossPartitions()
    {
        Vector3d position = (_tick & 1) == 0
            ? Vector3d.Zero
            : new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half);

        _compoundBody.SetPosition(position);
        _compoundCollider.Simulate();

        _tick++;
        return (_compoundCollider.PartitionCoordinates?.Count ?? 0) + (int)_compoundCollider.RuntimeShapeVersion;
    }

    private static Vector3d[] CreateUChannelVertices()
    {
        Fixed64 left = (Fixed64)(-2);
        Fixed64 right = (Fixed64)2;
        Fixed64 height = (Fixed64)2;
        Fixed64 depth = (Fixed64)4;

        return new[]
        {
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(left, height, Fixed64.Zero),
            new Vector3d(left, Fixed64.Zero, depth),
            new Vector3d(left, height, depth),
            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right, Fixed64.Zero, depth),
            new Vector3d(right, height, Fixed64.Zero),
            new Vector3d(right, height, depth),
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(left, height, Fixed64.Zero),
            new Vector3d(right, height, Fixed64.Zero)
        };
    }
}
