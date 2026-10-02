using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
[InvocationCount(1, 1)]
public class ColdCircleRegistrationBenchmarks
{
    private CirclePartitionBenchmarkScene _scene;

    [Params(64, 256, 1024)]
    public int ColliderCount { get; set; }

    [Params(1, 16)]
    public int CellSize { get; set; }

    [IterationSetup]
    public void Setup() => _scene = new CirclePartitionBenchmarkScene(ColliderCount, CellSize);

    [Benchmark]
    public int RegisterCirclePairs()
    {
        _scene.RegisterBodies();
        return _scene.Context.Physics2D.ColliderCount;
    }

    [IterationCleanup]
    public void Cleanup()
    {
        try
        {
            _scene.ValidateGeometry();
        }
        finally
        {
            _scene.Dispose();
        }
    }
}

[MemoryDiagnoser]
public class CirclePartitionMaintenanceBenchmarks
{
    private CirclePartitionBenchmarkScene _scene;
    private bool _offset;

    [Params(64, 256, 1024)]
    public int ColliderCount { get; set; }

    [Params(1, 16)]
    public int CellSize { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _offset = false;
        _scene = new CirclePartitionBenchmarkScene(ColliderCount, CellSize);
        _scene.RegisterBodies();
        _scene.ValidateGeometry();
        _scene.Context.Simulate();
        _scene.Context.LateSimulate();
        if (_scene.Context.Physics2D.LastBroadPhaseCandidateCount != ColliderCount / 2)
            throw new InvalidOperationException("Partition fixture candidate count changed.");
        _scene.ValidateRetainedPairs();

        // A bounded warmup populates both translated partition sets and query
        // scratch buffers without allowing settling to replace the intended scene.
        for (int i = 0; i < 4; i++)
        {
            RefreshTranslatedDynamicCircles();
            DistributeRetainedCandidates();
            if (ForceAutomaticGroundProbes() != 0)
                throw new InvalidOperationException("The above-body targets unexpectedly support a body.");
        }
        _scene.ValidateGeometry();
        _scene.ValidateRetainedPairs();
    }

    [Benchmark]
    public int RefreshTranslatedDynamicCircles()
    {
        _offset = !_offset;
        Vector2d offset = _offset ? Vector2d.Right : Vector2d.Zero;
        for (int i = 0; i < _scene.Bodies.Length; i++)
        {
            SolidBody2D body = _scene.Bodies[i];
            body.ResetPosition(_scene.Positions[i] + offset);
            body.Collider.Simulate();
        }
        return _scene.Context.Collisions2D.ActivePartitionCount;
    }

    [Benchmark]
    public uint DistributeRetainedCandidates()
    {
        // Keep processed pair keys: measure traversal, filtering and duplicate
        // suppression without repeating narrow phase or solving contacts.
        _scene.Context.Collisions2D.CheckAndDistributeCollisions();
        return _scene.Context.Collisions2D.Version;
    }

    [Benchmark]
    public int ForceAutomaticGroundProbes()
    {
        int grounded = 0;
        for (int i = 0; i < _scene.Bodies.Length; i++)
        {
            SolidBody2D body = _scene.Bodies[i];
            body.CheckGround();
            if (body.IsGrounded)
                grounded++;
        }
        return grounded;
    }

    [GlobalCleanup]
    public void Cleanup() => _scene?.Dispose();
}

internal sealed class CirclePartitionBenchmarkScene : IDisposable
{
    internal CirclePartitionBenchmarkScene(int colliderCount, int cellSize)
    {
        int pairCount = colliderCount / 2;
        Context = BenchmarkEnvironment.PrepareOwnedContext();
        Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        const int columns = 32;
        const int spacing = 16;
        int rows = (pairCount + columns - 1) / columns;
        if (!Context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-16, 0, -16),
            new Vector3d(columns * spacing + 16, 0, rows * spacing + 16),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)cellSize, Fixed64.One, (Fixed64)cellSize)), out _))
            throw new InvalidOperationException("Unable to create the partition scaling grid.");

        Bodies = new SolidBody2D[pairCount];
        Positions = new Vector2d[pairCount];
        Targets = new LSCircleCollider2D[pairCount];
        for (int i = 0; i < pairCount; i++)
            Positions[i] = new Vector2d(i % columns * spacing, i / columns * spacing);
    }

    internal GravitasWorldContext Context { get; }
    internal SolidBody2D[] Bodies { get; }
    internal Vector2d[] Positions { get; }
    private LSCircleCollider2D[] Targets { get; }

    internal void RegisterBodies()
    {
        for (int i = 0; i < Bodies.Length; i++)
        {
            Vector2d position = Positions[i];
            var body = new SolidBody2D(new BenchmarkMatterAgent(Context,
                new Vector3d(position.X, Fixed64.Zero, position.Y)), new LSCircleCollider2D((Fixed64)5))
            {
                Mass = Fixed64.One,
                SleepEnabled = false
            };
            body.Initialize(position);
            var targetCollider = new LSCircleCollider2D((Fixed64)5);
            Vector2d targetPosition = position + new Vector2d(3, 4);
            var target = new SolidBody2D(new BenchmarkMatterAgent(Context,
                new Vector3d(targetPosition.X, Fixed64.Zero, targetPosition.Y)), targetCollider);
            target.Initialize(targetPosition, motionType: BodyMotionType.Static);
            Bodies[i] = body;
            Targets[i] = targetCollider;
        }
    }

    internal void ValidateGeometry()
    {
        if (Context.Physics2D.BodyCount != Bodies.Length
            || Context.Physics2D.ColliderCount != Bodies.Length * 2)
            throw new InvalidOperationException("The partition fixture lost registered circles.");
        Vector2d expectedNormal = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        for (int i = 0; i < Bodies.Length; i++)
        {
            if (!CollisionDetection2D.TryCollide(Bodies[i].Collider, Targets[i], out Contact2D contact)
                || contact.Depth != (Fixed64)5 || contact.Normal != expectedNormal || contact.DepthIsClamped)
                throw new InvalidOperationException($"Incorrect partition fixture circle contact at pair {i}.");
        }
    }

    internal void ValidateRetainedPairs()
    {
        for (int i = 0; i < Bodies.Length; i++)
        {
            LSCollider2D first = Bodies[i].Collider;
            LSCollider2D second = Targets[i];
            bool hasPair = first.TryGetCollisionPair(second.Id, out CollisionPair2D pair)
                || second.TryGetCollisionPair(first.Id, out pair);
            if (!hasPair || !pair.IsColliding || pair.Manifold.Count != 1
                || first.CollisionPairCount + first.CollisionPairHolderCount != 1)
                throw new InvalidOperationException($"Incorrect retained partition fixture pair {i}.");
        }
    }

    public void Dispose() => Context.Dispose();
}
