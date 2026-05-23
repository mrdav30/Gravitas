using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionResponseBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair[] _pairs;

    [Params(64)]
    public int PairCount { get; set; }

    [IterationSetup(Target = nameof(CalculateImpulseForPreparedPairs))]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(PairCount * 2),
            clearAllPools: true);
        _pairs = new CollisionPair[PairCount];

        for (int i = 0; i < _pairs.Length; i++)
        {
            Vector3d origin = PositionForPair(i);
            _pairs[i] = CreateResponsePair(i, origin);
            CollisionDetection.DoCollisionCheck(_pairs[i]);
        }
    }

    [IterationCleanup(Target = nameof(CalculateImpulseForPreparedPairs))]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _pairs = null;
    }

    [Benchmark]
    public int CalculateImpulseForPreparedPairs()
    {
        for (int i = 0; i < _pairs.Length; i++)
            CollisionResponse.CalculateImpulse(_pairs[i]);

        return _pairs.Length;
    }

    private CollisionPair CreateResponsePair(int index, Vector3d origin)
    {
        return (index & 3) switch
        {
            0 => CreateMovingSphereSpherePair(origin),
            1 => CreateMovingCuboidSpherePair(origin),
            2 => CreateMovingCapsuleSpherePair(origin),
            _ => CreateMovingCuboidCuboidPair(origin),
        };
    }

    private CollisionPair CreateMovingSphereSpherePair(Vector3d origin)
    {
        ScenarioBody<LSSphereCollider> left = CreateBody(new LSSphereCollider(), origin);
        ScenarioBody<LSSphereCollider> right = CreateBody(
            new LSSphereCollider(),
            origin + new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        return new CollisionPair(left.Collider, right.Collider);
    }

    private CollisionPair CreateMovingCuboidSpherePair(Vector3d origin)
    {
        ScenarioBody<LSCuboidCollider> cuboid = CreateBody(new LSCuboidCollider(), origin);
        ScenarioBody<LSSphereCollider> sphere = CreateBody(
            new LSSphereCollider(),
            origin + new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Fraction(1, 4), Fixed64.Zero));
        Push(sphere.Body, -60);
        return new CollisionPair(cuboid.Collider, sphere.Collider);
    }

    private CollisionPair CreateMovingCapsuleSpherePair(Vector3d origin)
    {
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody(new LSCapsuleCollider(), origin, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> sphere = CreateBody(
            new LSSphereCollider(),
            origin + new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(capsule.Body, 60);
        Push(sphere.Body, -60);
        return new CollisionPair(capsule.Collider, sphere.Collider);
    }

    private CollisionPair CreateMovingCuboidCuboidPair(Vector3d origin)
    {
        ScenarioBody<LSCuboidCollider> left = CreateBody(new LSCuboidCollider(), origin);
        ScenarioBody<LSCuboidCollider> right = CreateBody(
            new LSCuboidCollider(),
            origin + new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        return new CollisionPair(left.Collider, right.Collider);
    }

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

    private static void Push(StiffBody body, int xImpulse)
    {
        body.AddLinearImpulse(new Vector3d((Fixed64)xImpulse, Fixed64.Zero, Fixed64.Zero));
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
