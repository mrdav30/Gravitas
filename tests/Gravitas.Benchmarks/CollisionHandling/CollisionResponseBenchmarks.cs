using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionResponseBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair[] _pairs;

    [Params(16, 64)]
    public int PairCount { get; set; }

    [Params(
        ResponseContactShape.SingleContact,
        ResponseContactShape.FaceManifold,
        ResponseContactShape.RestingFaceManifold,
        ResponseContactShape.CylinderContact,
        ResponseContactShape.MeshContact)]
    public ResponseContactShape ContactShape { get; set; }

    [IterationSetup]
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
            if (!CollisionDetection.DoCollisionCheck(_pairs[i]))
                throw new InvalidOperationException("Unable to prepare a 3D response contact pair.");
        }
    }

    [IterationCleanup]
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
            ResponseContactShape.FaceManifold => CreateMovingCuboidCuboidPair(origin),
            ResponseContactShape.RestingFaceManifold => CreateRestingCuboidStackPair(origin),
            ResponseContactShape.CylinderContact => CreateCylinderSpherePair(origin),
            _ => CreateMeshCuboidPair(origin),
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

    private CollisionPair CreateRestingCuboidStackPair(Vector3d origin)
    {
        ScenarioBody<LSCuboidCollider> floor = CreateBody(
            new LSCuboidCollider(),
            origin,
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = CreateBody(
            new LSCuboidCollider(),
            origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero));
        return new CollisionPair(floor.Collider, box.Collider);
    }

    private CollisionPair CreateCylinderSpherePair(Vector3d origin)
    {
        ScenarioBody<LSCylinderCollider> cylinder = CreateBody(
            new LSCylinderCollider(),
            origin,
            immovable: true);
        ScenarioBody<LSSphereCollider> sphere = CreateBody(
            new LSSphereCollider(),
            origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(sphere.Body, -60);
        return new CollisionPair(cylinder.Collider, sphere.Collider);
    }

    private CollisionPair CreateMeshCuboidPair(Vector3d origin)
    {
        ScenarioBody<LSMeshCollider> floor = CreateBody(
            CreateMeshFloor(),
            origin,
            preventAngularForces: true,
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = CreateBody(
            new LSCuboidCollider(),
            origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero));
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero));
        return new CollisionPair(floor.Collider, box.Collider);
    }

    private ScenarioBody<TCollider> CreateBody<TCollider>(TCollider collider, Vector3d position)
        where TCollider : LSCollider =>
        CreateBody(collider, position, preventAngularForces: false, immovable: false);

    private ScenarioBody<TCollider> CreateBody<TCollider>(
        TCollider collider,
        Vector3d position,
        bool preventAngularForces = false,
        bool immovable = false)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = preventAngularForces,
            Immovable = immovable
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static LSMeshCollider CreateMeshFloor() =>
        new(
            new[]
            {
                new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-2)),
                new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2)
            },
            new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);

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
        FaceManifold,
        RestingFaceManifold,
        CylinderContact,
        MeshContact
    }
}
