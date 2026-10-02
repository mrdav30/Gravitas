using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Persistent independent circle contacts through the complete pure-2D step.</summary>
[MemoryDiagnoser]
public class CircleContactSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody2D[] _bodies;
    private LSCircleCollider2D[] _targets;
    private Vector2d[] _positions;
    private Fixed64 _rotation;

    [Params(64, 1024)]
    public int PairCount { get; set; }

    [Params("Axis", "Diagonal", "Rotated")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _rotation = Fixed64.Zero;
        Fixed64 radius = Fixed64.One;
        Vector2d offset = Vector2d.Right;
        Vector2d expectedNormal = Vector2d.Right;
        Fixed64 expectedDepth = Fixed64.One;
        switch (Geometry)
        {
            case "Axis":
                break;
            case "Diagonal":
                radius = (Fixed64)5;
                offset = new Vector2d(3, 4);
                expectedDepth = (Fixed64)5;
                expectedNormal = new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
                break;
            case "Rotated":
                _rotation = Fixed64.One;
                break;
            default:
                throw new InvalidOperationException("Unknown circle contact geometry.");
        }

        _context = BenchmarkEnvironment.PrepareOwnedContext();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        const int columns = 32;
        const int spacing = 16;
        int rows = (PairCount + columns - 1) / columns;
        // Match cells to pair spacing: this isolates sustained contact work instead
        // of measuring large radius-five voxel fanout in a unit-cell grid.
        if (!_context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-16, 0, -16),
            new Vector3d(columns * spacing + 16, 0, rows * spacing + 16),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)16, Fixed64.One, (Fixed64)16)), out _))
            throw new InvalidOperationException("Unable to create the circle contact benchmark grid.");

        _bodies = new SolidBody2D[PairCount];
        _targets = new LSCircleCollider2D[PairCount];
        _positions = new Vector2d[PairCount];
        for (int i = 0; i < PairCount; i++)
        {
            // Sixteen units keep even the radius-five diagonal pairs independent.
            Vector2d position = new(i % columns * spacing, i / columns * spacing);
            _positions[i] = position;
            var first = new LSCircleCollider2D(radius);
            var second = new LSCircleCollider2D(radius);
            var body = new SolidBody2D(new BenchmarkMatterAgent(_context,
                new Vector3d(position.X, Fixed64.Zero, position.Y)), first)
            {
                Mass = Fixed64.One,
                SleepEnabled = false
            };
            body.Initialize(position, _rotation);
            var target = new SolidBody2D(new BenchmarkMatterAgent(_context,
                new Vector3d(position.X + offset.X, Fixed64.Zero, position.Y + offset.Y)), second);
            target.Initialize(position + offset, -_rotation, BodyMotionType.Static);
            _bodies[i] = body;
            _targets[i] = second;

            if (!CollisionDetection2D.TryCollide(first, second, out Contact2D contact)
                || contact.Normal != expectedNormal || contact.Depth != expectedDepth
                || contact.DepthIsClamped)
                throw new InvalidOperationException($"Incorrect {Geometry} circle contact at pair {i}.");
        }

        if (ResetAndDetectContacts() != PairCount)
            throw new InvalidOperationException("The dispatcher workload lost circle contacts.");
        for (int i = 0; i < 16; i++)
            ResetAndFullSimulationStep();

        var expectedPositions = new Vector2d[PairCount];
        var expectedVelocities = new Vector2d[PairCount];
        for (int i = 0; i < PairCount; i++)
        {
            if (_bodies[i].Position == _positions[i])
                throw new InvalidOperationException("The full-step workload skipped positional response.");
            expectedPositions[i] = _bodies[i].Position;
            expectedVelocities[i] = _bodies[i].LinearVelocity;
        }

        // Reset pose and motion every invocation: settling or sleep must never turn
        // a high-contact benchmark into a cheaper separated/resting workload.
        for (int frame = 0; frame < 8; frame++)
        {
            if (ResetAndFullSimulationStep() != PairCount)
                throw new InvalidOperationException("The full-step workload changed its candidate count.");
            for (int i = 0; i < PairCount; i++)
            {
                SolidBody2D body = _bodies[i];
                LSCollider2D first = body.Collider;
                LSCollider2D second = _targets[i];
                bool hasPair = first.TryGetCollisionPair(second.Id, out CollisionPair2D pair)
                    || second.TryGetCollisionPair(first.Id, out pair);
                if (!hasPair || !pair.IsColliding || pair.Manifold.Count != 1
                    || first.CollisionPairCount + first.CollisionPairHolderCount != 1
                    || body.IsSleeping || body.Position != expectedPositions[i]
                    || body.LinearVelocity != expectedVelocities[i]
                    || body.Rotation != _rotation || body.AngularVelocity != Fixed64.Zero)
                    throw new InvalidOperationException($"Unstable full-step circle contact at pair {i}.");
            }
        }
    }

    [Benchmark]
    public int ResetAndDetectContacts()
    {
        ResetBodies();
        int contacts = 0;
        for (int i = 0; i < PairCount; i++)
        {
            if (CollisionDetection2D.TryCollide(_bodies[i].Collider, _targets[i], out _))
                contacts++;
        }
        return contacts;
    }

    [Benchmark]
    public int ResetAndFullSimulationStep()
    {
        ResetBodies();
        _context.Simulate();
        _context.LateSimulate();
        return _context.Physics2D.LastBroadPhaseCandidateCount;
    }

    private void ResetBodies()
    {
        for (int i = 0; i < PairCount; i++)
            _bodies[i].ResetPosition(_positions[i], _rotation);
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
