using System;
using BenchmarkDotNet.Attributes;

namespace Gravitas.Benchmarks;

/// <summary>Automatic probes at the corrected poses used by the full-step control.</summary>
[MemoryDiagnoser]
public class CircleAutomaticGroundingBenchmarks
{
    private CirclePartitionBenchmarkScene _scene;

    [Params(64, 1024)]
    public int PairCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _scene = new CirclePartitionBenchmarkScene(PairCount * 2, cellSize: 16);
        _scene.RegisterBodies();
        _scene.ValidateGeometry();
        _scene.Context.Simulate();
        _scene.Context.LateSimulate();
        if (_scene.Context.Physics2D.LastBroadPhaseCandidateCount != PairCount)
            throw new InvalidOperationException("Corrected grounding fixture candidate count changed.");
        _scene.ValidateCorrectedGroundProbes();
        int expectedGrounded = ForceAutomaticGroundProbesAfterResponse();
        // Only the first row lacks a target below it. Each later row can be
        // supported by the previous row's circle after positional correction.
        if (expectedGrounded != PairCount - 32)
            throw new InvalidOperationException($"Incorrect corrected support count: {expectedGrounded}.");
        for (int i = 0; i < 4; i++)
            if (ForceAutomaticGroundProbesAfterResponse() != expectedGrounded)
                throw new InvalidOperationException("Corrected grounding support count changed between probes.");
    }

    [Benchmark]
    public int ForceAutomaticGroundProbesAfterResponse() => _scene.ForceAutomaticGroundProbes();

    [GlobalCleanup]
    public void Cleanup() => _scene?.Dispose();
}
