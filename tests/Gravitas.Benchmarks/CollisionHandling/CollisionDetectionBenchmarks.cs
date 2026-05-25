using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionDetectionBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair[] _pairs;

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
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _pairs = null;
    }

    [Benchmark]
    public int CheckPreparedPrimitivePairs()
    {
        int collisionCount = 0;
        for (int i = 0; i < _pairs.Length; i++)
        {
            if (CollisionDetection.DoCollisionCheck(_pairs[i]))
                collisionCount++;
        }

        return collisionCount;
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

    private LSSphereCollider CreateSphere(Vector3d position) =>
        CreateBody(new LSSphereCollider(), position).Collider;

    private LSCapsuleCollider CreateCapsule(Vector3d position) =>
        CreateBody(new LSCapsuleCollider(), position, preventAngularForces: true).Collider;

    private LSCuboidCollider CreateCuboid(Vector3d position) =>
        CreateBody(new LSCuboidCollider(), position).Collider;

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
        bool preventAngularForces = false)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = preventAngularForces
        };

        body.Initialize(position, FixedQuaternion.Identity);
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
