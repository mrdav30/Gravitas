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

    [Params(16, 64)]
    public int PairCount { get; set; }

    [Params(ResponseContactShape.SingleContact, ResponseContactShape.FaceManifold)]
    public ResponseContactShape ContactShape { get; set; }

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
            _pairs[i] = CreateResponsePair(origin);
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

    private CollisionPair CreateResponsePair(Vector3d origin)
    {
        return ContactShape switch
        {
            ResponseContactShape.SingleContact => CreateMovingCuboidSpherePair(origin),
            _ => CreateMovingCuboidCuboidPair(origin),
        };
    }

    private CollisionPair CreateMovingCuboidSpherePair(Vector3d origin)
    {
        ScenarioBody<LSCuboidCollider> cuboid = CreateBody(new LSCuboidCollider(), origin);
        ScenarioBody<LSSphereCollider> sphere = CreateBody(
            new LSSphereCollider(),
            origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero));
        Push(sphere.Body, -60);
        return new CollisionPair(cuboid.Collider, sphere.Collider);
    }

    private CollisionPair CreateMovingCuboidCuboidPair(Vector3d origin)
    {
        ScenarioBody<LSCuboidCollider> left = CreateBody(new LSCuboidCollider(), origin);
        ScenarioBody<LSCuboidCollider> right = CreateBody(
            new LSCuboidCollider(),
            origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        return new CollisionPair(left.Collider, right.Collider);
    }

    private ScenarioBody<TCollider> CreateBody<TCollider>(TCollider collider, Vector3d position)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
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

    public enum ResponseContactShape
    {
        SingleContact,
        FaceManifold
    }
}
