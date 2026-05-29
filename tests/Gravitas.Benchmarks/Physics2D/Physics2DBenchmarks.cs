using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Physics2DBenchmarks
{
    private GravitasWorldContext _integrationContext;
    private GravitasWorldContext _collisionContext;
    private GravitasWorldContext _queryContext;
    private GravitasWorldContext _detectionContext;
    private SwiftList<StiffBody2D> _integrationBodies;
    private SwiftList<StiffBody2D> _collisionBodies;
    private SwiftList<Physics2DHit> _queryHits;
    private PreparedPair2D[] _shapePairs;

    [Params(64)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _integrationContext = GravitasWorldContext.CreateOwned();
        _collisionContext = GravitasWorldContext.CreateOwned();
        _queryContext = GravitasWorldContext.CreateOwned();
        _detectionContext = GravitasWorldContext.CreateOwned();
        _integrationBodies = new SwiftList<StiffBody2D>(BodyCount);
        _collisionBodies = new SwiftList<StiffBody2D>(BodyCount);
        _queryHits = new SwiftList<Physics2DHit>(BodyCount);
        _shapePairs = new PreparedPair2D[BodyCount];

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)3);
            StiffBody2D body = CreateBody(_integrationContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            _integrationBodies.Add(body);
            _ = CreateBody(_queryContext, CreateShape(i), position, immovable: true);
            _shapePairs[i] = CreatePreparedPair(i);
        }

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            StiffBody2D dynamicBody = CreateBody(_collisionContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            _ = CreateBody(_collisionContext, new LSCircleCollider2D(Fixed64.Half), position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
            _collisionBodies.Add(dynamicBody);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _integrationContext.Dispose();
        _collisionContext.Dispose();
        _queryContext.Dispose();
        _detectionContext.Dispose();

        _integrationContext = null;
        _collisionContext = null;
        _queryContext = null;
        _detectionContext = null;
        _integrationBodies = null;
        _collisionBodies = null;
        _queryHits = null;
        _shapePairs = null;
    }

    [Benchmark]
    public int IntegrateDynamicBodies()
    {
        for (int i = 0; i < _integrationBodies.Count; i++)
            _integrationBodies[i].AddForce(Vector2d.Right);

        _integrationContext.Physics2D.LateSimulate();
        return _integrationContext.Physics2D.BodyCount;
    }

    [Benchmark]
    public int ResolveOverlappingCirclePairs()
    {
        for (int i = 0; i < _collisionBodies.Count; i++)
            _collisionBodies[i].SetPosition(PositionForIndex(i, spacing: (Fixed64)2));

        _collisionContext.Physics2D.Simulate();
        return _collisionContext.Physics2D.BodyCount;
    }

    [Benchmark]
    public int CheckRequiredShapePairs()
    {
        int collisionCount = 0;
        for (int i = 0; i < _shapePairs.Length; i++)
        {
            PreparedPair2D pair = _shapePairs[i];
            if (CollisionDetection2D.TryCollide(pair.ColliderA, pair.ColliderB, out _))
                collisionCount++;
        }

        return collisionCount;
    }

    [Benchmark]
    public int OverlapCircleAll()
    {
        return _queryContext.Physics2D.OverlapCircleAll(
            new Vector2d((Fixed64)12, (Fixed64)12),
            (Fixed64)18,
            _queryHits);
    }

    private PreparedPair2D CreatePreparedPair(int index)
    {
        LSCollider2D colliderA = CreateShape(index);
        LSCollider2D colliderB = CreateShape(index + 1);
        Vector2d position = PositionForIndex(index, spacing: (Fixed64)4);
        _ = CreateBody(_detectionContext, colliderA, position, immovable: true);
        _ = CreateBody(_detectionContext, colliderB, position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        return new PreparedPair2D(colliderA, colliderB);
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var body = new StiffBody2D(context, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateShape(int index)
    {
        return (index % 3) switch
        {
            0 => new LSCircleCollider2D(Fixed64.One),
            1 => new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            _ => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One))
        };
    }

    private static Vector2d PositionForIndex(int index, Fixed64 spacing)
    {
        int width = 8;
        int x = index % width;
        int y = index / width;
        return new Vector2d((Fixed64)x * spacing, (Fixed64)y * spacing);
    }

    private readonly struct PreparedPair2D
    {
        public PreparedPair2D(LSCollider2D colliderA, LSCollider2D colliderB)
        {
            ColliderA = colliderA;
            ColliderB = colliderB;
        }

        public LSCollider2D ColliderA { get; }

        public LSCollider2D ColliderB { get; }
    }
}
