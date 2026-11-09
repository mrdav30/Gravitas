using System;
using System.Diagnostics;
using System.Linq;
using Chronicler.Hashing;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using GridForge.Configuration;
using GridForge.Grids.Topology;

namespace Gravitas.Benchmarks;

internal static class CurvedContactWorkload
{
    private const int PoseCount = 16;
    private const int WarmupFrames = 16;
    private const int SampleCount = 128;
    private const int FrameCount = WarmupFrames + SampleCount;

    private static readonly Case[] Cases =
    {
        new("cylinder-cap", Cylinder("CapFace")),
        new("cylinder-oblique", Cylinder("ObliqueRim"), Tilt: true),
        new("cylinder-interior", Cylinder("InteriorRim"), Tilt: true),
        new("cylinder-gap", Cylinder("RimGap"), Gap: true, Slide: -Vector3d.Right),
        new("cone-base", Cone("BaseFace")),
        new("cone-oblique", Cone("ObliqueRim"), Tilt: true),
        new("cone-interior", Cone("InteriorRim"), Tilt: true),
        new("cone-gap", Cone("RimGap"), Gap: true, Slide: -Vector3d.Right),
        new("capsule-cylinder-side", Circle("Side")),
        new("capsule-cylinder-oblique", Circle("ObliqueRim"), Tilt: true),
        new("capsule-circle-side", Circle("Side"), Mixed: true),
        new("capsule-circle-oblique", Circle("ObliqueRim"), Mixed: true, Tilt: true),
        new("capsule-stadium-side", Stadium("Side"), Mixed: true),
        new("capsule-stadium-oblique", Stadium("ObliqueInteriorRim"), Mixed: true, Tilt: true),
        new("capsule-stadium-oblique-gap", Stadium("ObliqueInteriorRim"), Mixed: true, Gap: true,
            Slide: new Vector3d(0, 1, -1), Offset: new Vector3d(Fixed64.Zero, Fixed64.Half, -Fixed64.Half)),
        new("capsule-stadium-gap", Stadium("SeparatedRim"), Mixed: true, Gap: true)
    };

    internal static int Verify(string[] args)
    {
        Case[] selected = args.Length == 1 ? Cases
            : args.Length == 2 ? Cases.Where(item => item.Name == args[1]).ToArray()
            : Array.Empty<Case>();
        if (selected.Length == 0)
        {
            Console.Error.WriteLine($"Usage: verify-curved-workload [{string.Join('|', Cases.Select(item => item.Name))}]");
            return 1;
        }

        foreach (Case item in selected)
        {
            var samples = new Sample[SampleCount];
            Observation[] expected = Run(item, Span<Sample>.Empty);
            Observation[] replay = Run(item, Span<Sample>.Empty);
            Observation[] measured = Run(item, samples);
            var hash = new ChronicleHashWriter();
            hash.WriteSection(item.Name, 1);
            int contacts = 0;
            int mixedCandidates = 0;
            for (int i = 0; i < expected.Length; i++)
            {
                StrategyWorkload.Require(expected[i] == replay[i], i + 1, $"{item.Name} replay diverged");
                Observation row = expected[i];
                hash.WriteUInt64(row.State.Low);
                hash.WriteUInt64(row.State.High);
                hash.WriteInt32(row.Contacts);
                hash.WriteInt32(row.MixedCandidates);
                if (i >= WarmupFrames)
                {
                    contacts += row.Contacts;
                    mixedCandidates += row.MixedCandidates;
                }
            }
            StrategyWorkload.Require(expected[^1].State == measured[^1].State, FrameCount, "timed terminal state diverged");
            StrategyWorkload.Require(item.Gap ? contacts == 0 : contacts > 0, FrameCount, $"{item.Name} missed its contact envelope");
            Console.WriteLine($"{item.Name} trace {hash.ToHash()}, contact events {contacts}, mixed candidates {mixedCandidates}");
            WriteTiming("physics", samples.Select(sample => sample.StepTicks).Order().ToArray());
            WriteTiming("scenario", samples.Select(sample => sample.FrameTicks).Order().ToArray());
            Console.WriteLine($"  Physics step managed bytes total: {samples.Sum(sample => sample.StepBytes)}");
        }
        return 0;
    }

    private static Observation[] Run(Case item, Span<Sample> samples)
    {
        Fixture fixture = item.Create();
        using GravitasWorldContext context = fixture.Context;
        Vector3d initialPosition = fixture.Mover.Position;
        FixedQuaternion initialRotation = fixture.Mover.Rotation;
        fixture.Mover.Deactivate();
        if (item.Mixed)
            fixture.SpatialTarget?.Deactivate();
        else
            fixture.PlanarTarget?.Deactivate();
        context.Settings.RuntimeMode = item.Mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD;
        StrategyWorkload.Configure(context);
        StrategyWorkload.Require(context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-64, -64, -64), new Vector3d(64, 64, 64),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)8, (Fixed64)8, (Fixed64)8)), out _),
            0, "unable to create the curved workload grid");
        var mover = new SolidBody(new BenchmarkMatterAgent(context, initialPosition), fixture.Mover)
        {
            Mass = Fixed64.One,
            GravityScale = Fixed64.Zero,
            LinearDragCoefficient = Fixed64.Zero,
            SleepEnabled = false
        };
        mover.UseManualGrounding();
        mover.Initialize(initialPosition, initialRotation);
        if (samples.IsEmpty)
            context.Diagnostics.Enable();

        var poses = new (Vector3d Position, FixedQuaternion Rotation)[PoseCount];
        Vector3d slide = item.Slide == Vector3d.Zero ? Vector3d.Right : item.Slide;
        for (int i = 0; i < poses.Length; i++)
        {
            int distance = Math.Min(i, poses.Length - i);
            Fixed64 travel = Fixed64.FromFraction(distance, 32);
            FixedQuaternion rotation = item.Tilt
                ? initialRotation * FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero,
                    Fixed64.FromFraction(distance, 2))
                : initialRotation;
            poses[i] = (initialPosition + item.Offset + slide * travel, rotation);
        }

        var trace = new Observation[FrameCount];
        for (int i = 0; i < trace.Length; i++)
        {
            var pose = poses[i % PoseCount];
            bool measure = !samples.IsEmpty && i >= WarmupFrames;
            long frameStart = measure ? Stopwatch.GetTimestamp() : 0;
            mover.ResetPosition(pose.Position, pose.Rotation);
            long bytesBefore = measure ? GC.GetAllocatedBytesForCurrentThread() : 0;
            long stepStart = measure ? Stopwatch.GetTimestamp() : 0;
            context.Simulate();
            context.LateSimulate();
            if (measure)
            {
                long end = Stopwatch.GetTimestamp();
                samples[i - WarmupFrames] = new Sample(end - stepStart, end - frameStart,
                    GC.GetAllocatedBytesForCurrentThread() - bytesBefore);
            }

            StrategyWorkload.Require(context.FrameCount == i + 1 && context.LateSimulateToken == i + 1 &&
                mover.Active && mover.Collider.IsPartitioned,
                i + 1, "curved workload lost a frame or its moving collider");
            if (i == 0)
            {
                bool targetPresent = item.Mixed
                    ? fixture.PlanarTarget.IsMixedPartitioned && context.Physics2D.ColliderCount == 1 && context.Physics.ColliderCount == 1
                    : fixture.SpatialTarget.IsPartitioned && context.Physics.ColliderCount == 2 && context.Physics2D.ColliderCount == 0;
                StrategyWorkload.Require(targetPresent, 1, "curved workload lost its target or retained its comparison collider");
            }
            if (samples.IsEmpty)
            {
                int contacts = 0;
                foreach (GravitasDiagnosticEvent entry in context.Diagnostics.Events)
                    if (entry.Kind is GravitasDiagnosticEventKind.Contact or GravitasDiagnosticEventKind.MixedContact)
                        contacts++;
                trace[i] = new Observation(context.ComputeReplayHash(), contacts, context.MixedCollisions.LastBroadPhaseCandidateCount);
                context.Diagnostics.Clear();
            }
            else if (i == trace.Length - 1)
                trace[i] = new Observation(context.ComputeReplayHash(), 0, 0);
        }
        return trace;
    }

    private static void WriteTiming(string window, long[] ticks)
    {
        double Ms(long value) => value * 1000.0 / Stopwatch.Frequency;
        double mean = ticks.Average() * 1000.0 / Stopwatch.Frequency;
        int misses = ticks.Count(value => value > Stopwatch.Frequency / 60);
        Console.WriteLine(FormattableString.Invariant(
            $"  {window} ({ticks.Length}): mean {mean:F3}, p95 {Ms(ticks[(ticks.Length * 95 + 99) / 100 - 1]):F3}, p99 {Ms(ticks[(ticks.Length * 99 + 99) / 100 - 1]):F3}, max {Ms(ticks[^1]):F3} ms, {misses} over the 60 Hz interval"));
    }

    private static Func<Fixture> Cylinder(string geometry) => () =>
    {
        var benchmark = new MeshCylinderContactBenchmarks { Geometry = geometry };
        benchmark.Setup();
        return benchmark.WorldFixture;
    };

    private static Func<Fixture> Cone(string geometry) => () =>
    {
        var benchmark = new MeshConeContactBenchmarks { Geometry = geometry };
        benchmark.Setup();
        return benchmark.WorldFixture;
    };

    private static Func<Fixture> Circle(string geometry) => () =>
    {
        var benchmark = new CapsuleCircleContactBenchmarks { Geometry = geometry };
        benchmark.Setup();
        return benchmark.WorldFixture;
    };

    private static Func<Fixture> Stadium(string geometry) => () =>
    {
        var benchmark = new CapsuleSlabContactBenchmarks { Geometry = geometry };
        benchmark.Setup();
        return benchmark.WorldFixture;
    };

    internal readonly record struct Fixture(GravitasWorldContext Context, LSCollider Mover,
        LSCollider SpatialTarget = null, LSCollider2D PlanarTarget = null);

    private readonly record struct Case(string Name, Func<Fixture> Create, bool Mixed = false, bool Tilt = false,
        bool Gap = false, Vector3d Slide = default, Vector3d Offset = default);
    private readonly record struct Observation(ChronicleHash State, int Contacts, int MixedCandidates);
    private readonly record struct Sample(long StepTicks, long FrameTicks, long StepBytes);
}
