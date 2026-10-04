using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Paired ground-hit selection at identical corrected circle poses.</summary>
[MemoryDiagnoser]
public class CircleGroundQuerySelectionBenchmarks
{
    private CirclePartitionBenchmarkScene _scene;
    private readonly SwiftList<Physics2DHit> _hits = new(8);

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
            throw new InvalidOperationException("Ground selection fixture candidate count changed.");
        _scene.ValidateCorrectedGroundProbes();

        int supported = 0;
        for (int i = 0; i < _scene.Bodies.Length; i++)
        {
            SolidBody2D body = _scene.Bodies[i];
            Vector2d up = body.UseGravityDerivedGroundUpDirection && body.Gravity.MagnitudeSquared > Fixed64.Epsilon
                ? (-body.Gravity).Normalized
                : body.GroundUpDirection;
            if (body.GroundedDistanceRay != Fixed64.Half
                || body.GroundDownDistanceOnAir != body.GroundedDistanceRay
                || body.Collider.CanonicalGroundProbeRadius != (Fixed64)5
                || body.GroundProbeMode != GroundProbeMode2D.Auto
                || up != Vector2d.Forward)
                throw new InvalidOperationException("The default full-radius downward ground probe changed.");

            Vector2d start = body.Position;
            Vector2d end = start - Vector2d.Forward * body.GroundedDistanceRay;
            Fixed64 radius = body.Collider.CanonicalGroundProbeRadius;
            _scene.Context.Query2D.SweepCircleAgainstStaticAll(start, end, radius,
                _scene.Context.Settings.GroundCheckLayerMask, _hits, body.Collider, includeTriggers: false);
            bool oldFound = false;
            Physics2DHit oldHit = default;
            for (int j = 0; j < _hits.Count; j++)
            {
                if (!body.IsValidGroundHit(_hits[j]))
                    continue;
                oldFound = true;
                oldHit = _hits[j];
                break;
            }

            bool newFound = _scene.Context.Query2D.SweepCircleForGrounding(start, end, radius, body,
                out Physics2DHit newHit);
            // Validate the entire reduced witness, including owner, body, rigid
            // anchor, normal and raw distance, for supported and unsupported rows.
            if (oldFound != newFound || !oldHit.Equals(newHit))
                throw new InvalidOperationException($"Ground selection witness changed for body {i}.");
            if (oldFound)
                supported++;
        }

        // The first 32 bodies have no preceding row to support them. Paired
        // targets overlap the probes but fail the body's support-normal policy.
        if (supported != PairCount - 32 || _scene.ForceAutomaticGroundProbes() != supported)
            throw new InvalidOperationException($"Incorrect corrected support count: {supported}.");
        if (AllHitSortThenFilter() != NearestAcceptedHit())
            throw new InvalidOperationException("Ground selection checksums differ after warmup.");
    }

    [Benchmark(Baseline = true)]
    public long AllHitSortThenFilter()
    {
        int supported = 0;
        long selectedIds = 0;
        for (int i = 0; i < _scene.Bodies.Length; i++)
        {
            SolidBody2D body = _scene.Bodies[i];
            Vector2d start = body.Position;
            Vector2d end = start - Vector2d.Forward * body.GroundedDistanceRay;
            Fixed64 radius = body.Collider.CanonicalGroundProbeRadius;
            _scene.Context.Query2D.SweepCircleAgainstStaticAll(start, end, radius,
                _scene.Context.Settings.GroundCheckLayerMask, _hits, body.Collider, includeTriggers: false);
            for (int j = 0; j < _hits.Count; j++)
            {
                Physics2DHit candidate = _hits[j];
                if (!body.IsValidGroundHit(candidate))
                    continue;
                supported++;
                selectedIds += candidate.Collider.Id + 1L;
                break;
            }
        }
        return (selectedIds << 32) | (uint)supported;
    }

    [Benchmark]
    public long NearestAcceptedHit()
    {
        int supported = 0;
        long selectedIds = 0;
        for (int i = 0; i < _scene.Bodies.Length; i++)
        {
            SolidBody2D body = _scene.Bodies[i];
            Vector2d start = body.Position;
            Vector2d end = start - Vector2d.Forward * body.GroundedDistanceRay;
            Fixed64 radius = body.Collider.CanonicalGroundProbeRadius;
            if (!_scene.Context.Query2D.SweepCircleForGrounding(start, end, radius, body, out Physics2DHit hit))
                continue;
            supported++;
            selectedIds += hit.Collider.Id + 1L;
        }
        return (selectedIds << 32) | (uint)supported;
    }

    [GlobalCleanup]
    public void Cleanup() => _scene?.Dispose();
}
