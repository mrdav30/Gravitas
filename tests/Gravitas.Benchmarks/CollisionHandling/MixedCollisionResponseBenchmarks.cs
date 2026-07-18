using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedCollisionResponseBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPairMixed[] _pairs;
    private MixedContact[] _contacts;

    [Params(16, 64)]
    public int PairCount { get; set; }

    [Params(ResponseMaterialMode.Default, ResponseMaterialMode.Distinct)]
    public ResponseMaterialMode MaterialMode { get; set; }

    [IterationSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(PairCount * 2),
            clearAllPools: true);
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        _pairs = new CollisionPairMixed[PairCount];
        _contacts = new MixedContact[PairCount];

        for (int i = 0; i < PairCount; i++)
            CreateResponsePair(PositionForPair(i), i);
    }

    [IterationCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _pairs = null;
        _contacts = null;
    }

    [Benchmark]
    public int ResolvePreparedMixedPairs()
    {
        for (int i = 0; i < _pairs.Length; i++)
            CollisionResponseMixed.Resolve(_pairs[i], _contacts[i]);

        return _pairs.Length;
    }

    [Benchmark]
    public int ResolvePreparedMixedIslandIterations()
    {
        int iterationLimit = _context.Settings.DiscreteSolverIterations;
        for (int iteration = 0; iteration < iterationLimit; iteration++)
        {
            bool applyPositionCorrection = iteration == 0;
            for (int i = 0; i < _pairs.Length; i++)
            {
                CollisionResponseMixed.Resolve(
                    _pairs[i],
                    _contacts[i],
                    iteration,
                    iterationLimit,
                    applyPositionCorrection);
            }
        }

        return _pairs.Length * iterationLimit;
    }

    private void CreateResponsePair(Vector3d origin, int index)
    {
        ScenarioBody<LSSphereCollider> sphere = CreateBody(
            new LSSphereCollider(),
            origin + new Vector3d(Fixed64.FromFraction(-1, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateCircle2D(new Vector2d(origin.X, origin.Z));
        ApplyMaterialMode(sphere.Collider, circle.Collider);
        sphere.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(15, 16) * sphere.Body.Mass);

        if (!CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact))
            throw new InvalidOperationException("Unable to prepare a mixed response contact pair.");

        _pairs[index] = new CollisionPairMixed(sphere.Collider, circle.Collider);
        _contacts[index] = contact;
    }

    private ScenarioBody<TCollider> CreateBody<TCollider>(TCollider collider, Vector3d position)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private SolidBody2D CreateCircle2D(Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(
            _context,
            new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static Vector3d PositionForPair(int index)
    {
        int x = index % 8;
        int z = index / 8;
        return new Vector3d(x * 3, 0, z * 3);
    }

    private void ApplyMaterialMode(LSCollider collider3D, LSCollider2D collider2D)
    {
        if (MaterialMode == ResponseMaterialMode.Default)
            return;

        collider3D.Material = RoughSurface;
        collider2D.Material = SlickSurface;
    }

    private readonly struct ScenarioBody<TCollider>
        where TCollider : LSCollider
    {
        public ScenarioBody(SolidBody body, TCollider collider)
        {
            Body = body;
            Collider = collider;
        }

        public SolidBody Body { get; }

        public TCollider Collider { get; }
    }

    public enum ResponseMaterialMode
    {
        Default,
        Distinct
    }

    private static readonly PhysicsMaterial RoughSurface = new(
        (Fixed64)2,
        Fixed64.One,
        Fixed64.FromFraction(1, 4),
        PhysicsMaterialCombine.Maximum,
        PhysicsMaterialCombine.Minimum);

    private static readonly PhysicsMaterial SlickSurface = new(
        Fixed64.Half,
        Fixed64.FromFraction(1, 4),
        Fixed64.FromFraction(3, 4),
        PhysicsMaterialCombine.Minimum,
        PhysicsMaterialCombine.Maximum);
}
