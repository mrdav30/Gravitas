using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionDetectionBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair[] _pairs;
    private CollisionPair[] _primitivePairs;
    private CollisionPair[] _cuboidFacePairs;
    private CollisionPair[] _cuboidSatPairs;
    private CollisionPair[] _meshCylinderPairs;
    private CollisionPair[] _meshCuboidPairs;
    private CollisionPair[] _meshMeshPairs;

    [Params(64)]
    public int PairCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(PairCount * 2),
            clearAllPools: true);
        _pairs = new CollisionPair[PairCount];

        for (int i = 0; i < _pairs.Length; i++)
        {
            Vector3d origin = PositionForPair(i);
            _pairs[i] = CreateDetectionPair(i, origin);
        }

        _primitivePairs = CreatePairSet(CreatePrimitivePair);
        _cuboidFacePairs = CreatePairSet(CreateCuboidFacePair);
        _cuboidSatPairs = CreatePairSet(CreateCuboidSatPair);
        _meshCylinderPairs = CreatePairSet(CreateMeshCylinderPair);
        _meshCuboidPairs = CreatePairSet(CreateMeshCuboidPair);
        _meshMeshPairs = CreatePairSet(CreateMeshMeshPair);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _pairs = null;
        _primitivePairs = null;
        _cuboidFacePairs = null;
        _cuboidSatPairs = null;
        _meshCylinderPairs = null;
        _meshCuboidPairs = null;
        _meshMeshPairs = null;
    }

    [Benchmark]
    public int CheckPreparedPrimitivePairs()
    {
        return CountCollisions(_pairs);
    }

    [Benchmark]
    public int CheckNonSatPrimitivePairs()
    {
        return CountCollisions(_primitivePairs);
    }

    [Benchmark]
    public int GeneratePrimitiveManifolds()
    {
        return CountManifoldContacts(_primitivePairs);
    }

    [Benchmark]
    public int GenerateCuboidFaceManifolds()
    {
        return CountManifoldContacts(_cuboidFacePairs);
    }

    [Benchmark]
    public int CheckCuboidCuboidSatPairs()
    {
        return CountCollisions(_cuboidSatPairs);
    }

    [Benchmark]
    public int CheckMeshCylinderPairs()
    {
        return CountCollisions(_meshCylinderPairs);
    }

    [Benchmark]
    public int CheckMeshCuboidPairs()
    {
        return CountCollisions(_meshCuboidPairs);
    }

    [Benchmark]
    public int CheckMeshMeshPairs()
    {
        return CountCollisions(_meshMeshPairs);
    }

    private static int CountCollisions(CollisionPair[] pairs)
    {
        int collisionCount = 0;
        for (int i = 0; i < pairs.Length; i++)
        {
            if (CollisionDetection.DoCollisionCheck(pairs[i]))
                collisionCount++;
        }

        return collisionCount;
    }

    private static int CountManifoldContacts(CollisionPair[] pairs)
    {
        int contactCount = 0;
        for (int i = 0; i < pairs.Length; i++)
        {
            if (CollisionDetection.DoCollisionCheck(pairs[i]))
                contactCount += pairs[i].Manifold.Count;
        }

        return contactCount;
    }

    private CollisionPair[] CreatePairSet(Func<int, Vector3d, CollisionPair> pairFactory)
    {
        var pairs = new CollisionPair[PairCount];
        for (int i = 0; i < pairs.Length; i++)
            pairs[i] = pairFactory(i, PositionForPair(i));

        return pairs;
    }

    private CollisionPair CreateDetectionPair(int index, Vector3d origin)
    {
        return (index % 9) switch
        {
            0 => new CollisionPair(
                CreateSphere(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            1 => new CollisionPair(
                CreateCapsule(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            2 => new CollisionPair(
                CreateCuboid(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            3 => new CollisionPair(
                CreateCuboid(origin),
                CreateCuboid(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            4 => new CollisionPair(
                CreateCylinder(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            5 => new CollisionPair(
                CreateCylinder(origin),
                CreateCapsule(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            6 => new CollisionPair(
                CreateCylinder(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            7 => new CollisionPair(
                CreateCuboid(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            _ => new CollisionPair(
                CreateMeshFloor(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Zero, Fixed64.Fraction(1, 4), Fixed64.Zero))),
        };
    }

    private CollisionPair CreatePrimitivePair(int index, Vector3d origin)
    {
        return (index % 7) switch
        {
            0 => new CollisionPair(
                CreateSphere(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            1 => new CollisionPair(
                CreateCapsule(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            2 => new CollisionPair(
                CreateCuboid(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            3 => new CollisionPair(
                CreateCylinder(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            4 => new CollisionPair(
                CreateCylinder(origin),
                CreateCapsule(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            5 => new CollisionPair(
                CreateCylinder(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            _ => new CollisionPair(
                CreateCuboid(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
        };
    }

    private CollisionPair CreateCuboidFacePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin),
            CreateCuboid(origin + new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateCuboidSatPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin, FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)35, Fixed64.Zero)),
            CreateCuboid(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateMeshCylinderPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateCylinder(origin + new Vector3d(Fixed64.Zero, Fixed64.Fraction(1, 4), Fixed64.Zero)));
    }

    private CollisionPair CreateMeshCuboidPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateCuboid(origin + new Vector3d(Fixed64.Zero, Fixed64.Fraction(1, 4), Fixed64.Zero)));
    }

    private CollisionPair CreateMeshMeshPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateMeshFloor(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
    }

    private LSSphereCollider CreateSphere(Vector3d position) =>
        CreateBody(new LSSphereCollider(), position).Collider;

    private LSCapsuleCollider CreateCapsule(Vector3d position) =>
        CreateBody(new LSCapsuleCollider(), position, preventAngularForces: true).Collider;

    private LSCuboidCollider CreateCuboid(Vector3d position, FixedQuaternion? rotation = null) =>
        CreateBody(new LSCuboidCollider(), position, rotation ?? FixedQuaternion.Identity).Collider;

    private LSCylinderCollider CreateCylinder(Vector3d position) =>
        CreateBody(new LSCylinderCollider(), position).Collider;

    private LSMeshCollider CreateMeshFloor(Vector3d position) =>
        CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)(-1)),
                    new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)(-1)),
                    new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.One),
                    new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One)
                },
                new[] { 0, 2, 1, 1, 2, 3 }),
            position,
            preventAngularForces: true).Collider;

    private ScenarioBody<TCollider> CreateBody<TCollider>(
        TCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null,
        bool preventAngularForces = false)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = preventAngularForces
        };

        body.Initialize(position, rotation ?? FixedQuaternion.Identity);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static Vector3d PositionForPair(int index)
    {
        int x = index % 8;
        int z = index / 8;
        return new Vector3d(x * 3, 0, z * 3);
    }

    private readonly struct ScenarioBody<TCollider>
        where TCollider : LSCollider
    {
        public ScenarioBody(StiffBody body, TCollider collider)
        {
            Body = body;
            Collider = collider;
        }

        public StiffBody Body { get; }

        public TCollider Collider { get; }
    }
}
