using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class PartitionCullingBenchmarks
{
    private GravitasWorldContext _repartitionContext;
    private SwiftList<SolidBody> _repartitionBodies;
    private SwiftList<Vector3d> _repartitionBasePositions;
    private bool _repartitionOffset;

    private GravitasWorldContext _churnContext;
    private PhysicsPartition _churnPartition;

    private GravitasWorldContext _sleepContext;
    private PhysicsPartition _sleepingPartition;

    private GravitasWorldContext _cullContext;
    private SolidBody _cullBody;
    private LSCollider _cullCollider;
    private CollisionPair _cullPair;
    private bool _cullOffset;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int gridExtent = BenchmarkPhysicsScene.GridExtentForGrid(ColliderCount);

        _repartitionContext = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        _repartitionBodies = new SwiftList<SolidBody>(ColliderCount);
        _repartitionBasePositions = new SwiftList<Vector3d>(ColliderCount);
        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_repartitionContext, ColliderCount, _repartitionBodies);
        for (int i = 0; i < _repartitionBodies.Count; i++)
            _repartitionBasePositions.Add(_repartitionBodies[i].Position3d);

        _churnContext = BenchmarkPhysicsScene.CreateContext(4);
        _churnPartition = _churnContext.Collisions.RentPartition();
        for (int i = 0; i < ColliderCount; i++)
            _churnPartition.AddDynamicObject(i);

        _sleepContext = BenchmarkPhysicsScene.CreateContext(4);
        _sleepingPartition = _sleepContext.Collisions.RentPartition();
        for (int i = 0; i < ColliderCount; i++)
        {
            _sleepingPartition.AddDynamicObject(i);
            _sleepingPartition.SetDynamicObjectAwake(i, awake: false);
        }

        _cullContext = BenchmarkPhysicsScene.CreateContext(gridExtent);
        SolidBody firstBody = CreateDynamicSphere(_cullContext, new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero));
        _cullBody = CreateDynamicSphere(_cullContext, new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        _cullCollider = _cullBody.Collider;
        _cullPair = new CollisionPair(firstBody.Collider, _cullCollider);
        _cullPair.UpdateCollision();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _repartitionContext.Dispose();
        _churnContext.Dispose();
        _sleepContext.Dispose();
        _cullContext.Dispose();

        _repartitionContext = null;
        _repartitionBodies = null;
        _repartitionBasePositions = null;
        _churnContext = null;
        _churnPartition = null;
        _sleepContext = null;
        _sleepingPartition = null;
        _cullContext = null;
        _cullBody = null;
        _cullCollider = null;
        _cullPair = null;
    }

    [Benchmark]
    public uint RepartitionTeleportedDynamicSpheres()
    {
        Vector3d offset = _repartitionOffset
            ? Vector3d.Right
            : Vector3d.Zero;
        _repartitionOffset = !_repartitionOffset;

        for (int i = 0; i < _repartitionBodies.Count; i++)
            _repartitionBodies[i].SetPosition(_repartitionBasePositions[i] + offset);

        _repartitionContext.Physics.Simulate();
        return _repartitionContext.Collisions.Version;
    }

    [Benchmark]
    public int RemoveAndReAddDynamicPartitionMembers()
    {
        for (int i = 0; i < ColliderCount; i++)
            _churnPartition.RemoveDynamicObject(i);

        for (int i = 0; i < ColliderCount; i++)
            _churnPartition.AddDynamicObject(i);

        return _churnPartition.ContainedDynamicObjects?.Count ?? 0;
    }

    [Benchmark]
    public int DistributeSleepingOnlyDynamicPartition()
    {
        _sleepContext.Collisions.CheckAndDistributeCollisions();
        return _sleepingPartition.AwakeDynamicObjectCount;
    }

    [Benchmark]
    public short RecheckCulledPairAfterColliderMove()
    {
        Vector3d position = _cullOffset
            ? new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero)
            : new Vector3d((Fixed64)8 + Fixed64.FromFraction(1, 16), Fixed64.Zero, Fixed64.Zero);
        _cullOffset = !_cullOffset;

        _cullBody.SetPosition(position);
        _cullPair.UpdateCollision();
        _cullBody.CheckChangedValues();
        return _cullPair.CullCounter;
    }

    private static SolidBody CreateDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }
}
