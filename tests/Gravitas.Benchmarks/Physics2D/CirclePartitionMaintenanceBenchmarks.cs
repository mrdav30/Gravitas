using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;

namespace Gravitas.Benchmarks;

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
        => _scene.ForceAutomaticGroundProbes();

    [GlobalCleanup]
    public void Cleanup() => _scene?.Dispose();
}
