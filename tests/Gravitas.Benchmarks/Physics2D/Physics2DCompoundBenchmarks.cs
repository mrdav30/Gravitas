using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using GridForge.Configuration;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Physics2DCompoundBenchmarks
{
    private GravitasWorldContext _queryContext;
    private GravitasWorldContext _detectionContext;
    private SwiftList<LSCollider2D> _compoundColliders;
    private SwiftList<Physics2DHit> _queryHits;
    private PreparedPair2D[] _shapePairs;

    [Params(64, 1024)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _queryContext = GravitasWorldContext.CreateOwned();
        _detectionContext = GravitasWorldContext.CreateOwned();
        Configure2DContext(_queryContext, BodyCount);
        Configure2DContext(_detectionContext, BodyCount);
        _compoundColliders = new SwiftList<LSCollider2D>(BodyCount);
        _queryHits = new SwiftList<Physics2DHit>(BodyCount);
        _shapePairs = new PreparedPair2D[BodyCount];

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)3);
            StiffBody2D queryBody = CreateBody(_queryContext, CreateCompoundShape(), position, immovable: true);
            _compoundColliders.Add(queryBody.Collider);
            _shapePairs[i] = CreatePreparedPair(i);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _queryContext.Dispose();
        _detectionContext.Dispose();
        _queryContext = null;
        _detectionContext = null;
        _compoundColliders = null;
        _queryHits = null;
        _shapePairs = null;
    }

    [Benchmark]
    public uint SimulateUnchangedCompoundColliders()
    {
        uint versionTotal = 0;
        for (int i = 0; i < _compoundColliders.Count; i++)
        {
            LSCollider2D collider = _compoundColliders[i];
            collider.Simulate();
            versionTotal += collider.RuntimeShapeVersion;
        }

        return versionTotal;
    }

    [Benchmark]
    public int CheckCompoundShapePairs()
    {
        int collisionCount = 0;
        for (int i = 0; i < _shapePairs.Length; i++)
        {
            PreparedPair2D pair = _shapePairs[i];
            if (CollisionDetection2D.TryCollide(pair.WorkItem, pair.Manifold, i))
                collisionCount++;
        }

        return collisionCount;
    }

    [Benchmark]
    public int OverlapCircleAll_CompoundTargets()
    {
        return _queryContext.Query2D.OverlapCircleAll(
            new Vector2d((Fixed64)12, (Fixed64)12),
            (Fixed64)18,
            _queryHits);
    }

    [Benchmark]
    public int SweepCircleAll_CompoundTargets()
    {
        return _queryContext.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-8), (Fixed64)12),
            new Vector2d((Fixed64)64, (Fixed64)12),
            Fixed64.Half,
            _queryHits);
    }

    private static void Configure2DContext(GravitasWorldContext context, int bodyCount)
    {
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        int extent = bodyCount <= 64 ? 64 : 512;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), Fixed64.Zero, (Fixed64)(-16)),
                new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent)),
            out _);
    }

    private PreparedPair2D CreatePreparedPair(int index)
    {
        LSCollider2D colliderA = CreateCompoundShape();
        LSCollider2D colliderB = CreateShape(index);
        Vector2d position = PositionForIndex(index, spacing: (Fixed64)4);
        _ = CreateBody(_detectionContext, colliderA, position, immovable: true);
        _ = CreateBody(_detectionContext, colliderB, position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
        return new PreparedPair2D(new CollisionWorkItem2D(colliderA, colliderB, collisionType));
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new StiffBody2D(agent, collider)
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

    private static LSCollider2D CreateCompoundShape()
    {
        return new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Half, Fixed64.Zero)));
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
        public PreparedPair2D(CollisionWorkItem2D workItem)
        {
            WorkItem = workItem;
            Manifold = new ContactManifold2D();
        }

        public CollisionWorkItem2D WorkItem { get; }

        public ContactManifold2D Manifold { get; }
    }
}
