using System;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using Chronicler.Hashing;
using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.Benchmarks;

internal static class StrategyWorkload
{
    internal const int ActorsPerRegion = 256;
    internal const int RegionStride = 192;
    internal const int EncounterActors = 24;
    internal const int SparseActors = 8;
    internal const int ProjectileCount = 2;
    internal const int FirstProjectileSlot = ActorsPerRegion - ProjectileCount;
    internal const int StageFrames = 32;
    internal const int SparseStart = StageFrames + 1;
    internal const int CrowdedStart = 2 * StageFrames + 1;
    internal const int RetireActorsFrame = 40;
    internal const int FireFrame = 44;
    internal const int RetireProjectilesFrame = 57;
    internal const int ProjectileSpeed = 900;
    internal static readonly Fixed64 WallX = Fixed64.FromFraction(31, 4);

    private const int FrameCount = 3 * StageFrames;
    private const int QuietWarmupFrames = 8;
    private const int QueryCadence = 8;

    internal delegate Observation[] Replay(Span<FrameTiming> samples);
    internal delegate void QueryRegion<TRegion>(TRegion region, int frame, ref ChronicleHashWriter query);

    internal static int Verify(string workloadId, int regionCount, string pairMetric, Replay replay)
    {
        Console.WriteLine($"Verifying {workloadId}.");
        Observation[] baseline = replay(Span<FrameTiming>.Empty);
        Observation[] verification = replay(Span<FrameTiming>.Empty);
        var samples = new FrameTiming[FrameCount];
        Observation[] measured = replay(samples);
        var trace = new ChronicleHashWriter();
        trace.WriteSection(workloadId, 1);
        for (int i = 0; i < baseline.Length; i++)
        {
            Observation row = baseline[i];
            if (row != verification[i])
                throw new InvalidOperationException(
                    $"{workloadId}, frame {i + 1}: replay divergence\nExpected: {row}\nObserved: {verification[i]}");
            Require(row.Queries == measured[i].Queries, i + 1, "measured replay changed a query result");
            trace.WriteUInt64(row.State.Low);
            trace.WriteUInt64(row.State.High);
            trace.WriteBool(row.Queries.HasValue);
            if (row.Queries is ChronicleHash queries)
            {
                trace.WriteUInt64(queries.Low);
                trace.WriteUInt64(queries.High);
            }
            trace.WriteInt32(row.Registered);
            trace.WriteInt32(row.Awake);
            trace.WriteInt32(row.Pairs);
            trace.WriteInt32(row.ChainContacts);
            trace.WriteInt32(row.LargestChain);
            trace.WriteInt32(row.ToiIterations);
        }

        Require(baseline[^1] == measured[^1], FrameCount, "measured replay changed the terminal observation");
        int fullContactCount = regionCount * (EncounterActors - 1);
        int firstFullChain = Array.FindIndex(baseline, CrowdedStart - 1, row => row.ChainContacts == fullContactCount) + 1;
        Require(firstFullChain > 0, FrameCount, "the encounters never produced all connected contact chains together");
        Console.WriteLine($"{workloadId} matched all {FrameCount} frames. Trace {trace.ToHash()}");
        Console.WriteLine($"All encounter chains first complete on frame {firstFullChain}.");
        WriteStage("quiet", baseline[..StageFrames], samples[QuietWarmupFrames..StageFrames], regionCount, pairMetric);
        WriteStage("sparse", baseline[StageFrames..(2 * StageFrames)], samples[StageFrames..(2 * StageFrames)],
            regionCount, pairMetric);
        WriteStage("crowded", baseline[(2 * StageFrames)..], samples[(2 * StageFrames)..], regionCount, pairMetric);
        if (Environment.GetEnvironmentVariable("GRAVITAS_STRATEGY_FRAME_CSV") == "1")
            WriteFrames(samples, baseline);
        return 0;
    }

    internal static Observation[] Capture<TRegion>(GravitasWorldContext context, TRegion[] regions,
        Action<TRegion, int> applyCommands, Action<TRegion> runQueries, QueryRegion<TRegion> observeQueries,
        Func<ChronicleHash?, int, Observation> observeWork, Span<FrameTiming> samples)
    {
        var trace = new Observation[FrameCount];
        for (int frame = 1; frame <= FrameCount; frame++)
        {
            bool sample = !samples.IsEmpty && frame > QuietWarmupFrames;
            bool queryFrame = frame % QueryCadence == 0;
            long frameStart = sample ? Stopwatch.GetTimestamp() : 0;
            foreach (TRegion region in regions)
                applyCommands(region, frame);
            long bytesBefore = sample ? GC.GetAllocatedBytesForCurrentThread() : 0;
            long stepStart = sample ? Stopwatch.GetTimestamp() : 0;
            context.Simulate();
            context.LateSimulate();
            long stepTicks = sample ? Stopwatch.GetTimestamp() - stepStart : 0;
            long stepBytes = sample ? GC.GetAllocatedBytesForCurrentThread() - bytesBefore : 0;
            if (queryFrame)
                foreach (TRegion region in regions)
                    runQueries(region);
            if (sample)
                samples[frame - 1] = new FrameTiming(stepTicks, Stopwatch.GetTimestamp() - frameStart, stepBytes);

            Require(context.FrameCount == frame && context.LateSimulateToken == frame,
                frame, "the complete host step did not advance both clocks");
            ChronicleHash? queryHash = null;
            if (queryFrame)
            {
                var query = new ChronicleHashWriter();
                foreach (TRegion region in regions)
                    observeQueries(region, frame, ref query);
                queryHash = query.ToHash();
            }
            // The spatial replay hash probes every collider pair and can disturb the next timed step.
            trace[frame - 1] = samples.IsEmpty || frame == FrameCount
                ? observeWork(queryHash, frame)
                : default(Observation) with { Queries = queryHash };
        }
        return trace;
    }

    internal static void Configure(GravitasWorldContext context)
    {
        context.SetFrameRate(60);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        context.Settings.DiscreteSolverIterations = 6;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
    }

    internal static Vector2d RegionOrigin(int region) => new(region % 2 * RegionStride, region / 2 * RegionStride);

    internal static Vector2d ActorPosition(int slot)
    {
        int gridIndex = slot - EncounterActors;
        return gridIndex < 0
            ? new Vector2d(Fixed64.FromFraction(9 * slot, 4), (Fixed64)144)
            : new Vector2d(32 + 8 * (gridIndex % 16), 8 + 8 * (gridIndex / 16));
    }

    internal static int ProjectileLaneZ(int slot) => -16 - 2 * (slot - FirstProjectileSlot);

    // A constant velocity difference closes all 23 initially equal gaps together.
    internal static int EncounterImpulseX(int slot) => 3 * (EncounterActors - 1 - 2 * slot);

    internal static void CheckPopulation(int registered, int awake, int frame)
    {
        int expectedRegistered = frame is >= RetireActorsFrame and < FireFrame or >= RetireProjectilesFrame and < CrowdedStart
            ? ActorsPerRegion - ProjectileCount : ActorsPerRegion;
        Require(registered == expectedRegistered, frame, "incorrect region registration count");
        if (frame <= StageFrames)
            Require(awake == 0, frame, "quiet actors woke up");
        if (frame == SparseStart)
            Require(awake == SparseActors, frame, "the sparse wake burst changed population");
        if (frame is >= SparseStart and < CrowdedStart)
        {
            int activeProjectiles = frame is >= FireFrame and < RetireProjectilesFrame ? ProjectileCount : 0;
            Require(awake <= SparseActors + activeProjectiles, frame, "the sparse stage woke extra actors");
        }
        if (frame == CrowdedStart)
            Require(awake == SparseActors + EncounterActors, frame, "the crowded wake burst changed population");
    }

    internal static (int Contacts, int Largest) ObserveChain<T>(T[] actors, Func<T, T, bool> hasContact)
    {
        int chain = 1;
        int largest = 0;
        int contacts = 0;
        for (int i = 1; i < EncounterActors; i++)
        {
            if (hasContact(actors[i - 1], actors[i]))
            {
                contacts++;
                largest = Math.Max(largest, ++chain);
            }
            else
                chain = 1;
        }
        return (contacts, largest);
    }

    private static void WriteStage(string stage, Observation[] rows, FrameTiming[] samples,
        int regionCount, string pairMetric)
    {
        int queryBatches = regionCount * rows.Count(row => row.Queries.HasValue);
        Console.WriteLine($"{stage}: registered {rows.Min(row => row.Registered)}–{rows.Max(row => row.Registered)}, " +
            $"peak awake {rows.Max(row => row.Awake)}, {pairMetric} {rows.Sum(row => row.Pairs)}, " +
            $"peak chain contacts {rows.Max(row => row.ChainContacts)}, largest chain {rows.Max(row => row.LargestChain)}, " +
            $"TOI iterations {rows.Sum(row => row.ToiIterations)}, queries {2 * queryBatches}, hits {3 * queryBatches}");
        WriteWindow("Physics step", samples.Select(row => row.StepTicks).Order().ToArray());
        WriteWindow("Scenario frame", samples.Select(row => row.FrameTicks).Order().ToArray());
        Console.WriteLine($"  Physics step managed bytes total: {samples.Sum(row => row.StepBytes)}");
    }

    private static void WriteWindow(string window, long[] ticks)
    {
        double p50 = Milliseconds(ticks[(ticks.Length - 1) / 2] + ticks[ticks.Length / 2]) / 2;
        double p95 = Milliseconds(ticks[(ticks.Length * 95 + 99) / 100 - 1]);
        double p99 = Milliseconds(ticks[(ticks.Length * 99 + 99) / 100 - 1]);
        int overBudget = ticks.Count(value => value > Stopwatch.Frequency / 60);
        Console.WriteLine(FormattableString.Invariant(
            $"  {window} ({ticks.Length} frames): p50 {p50:F3}, p95 {p95:F3}, p99 {p99:F3}, max {Milliseconds(ticks[^1]):F3} ms, {overBudget} over the 60 Hz interval"));
    }

    private static void WriteFrames(FrameTiming[] samples, Observation[] observations)
    {
        Console.WriteLine("frame,physics_ms,scenario_ms,physics_bytes,awake,pairs,chain_contacts,toi_iterations,query_frame");
        for (int i = QuietWarmupFrames; i < samples.Length; i++)
        {
            FrameTiming sample = samples[i];
            Observation row = observations[i];
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{i + 1},{Milliseconds(sample.StepTicks):F6},{Milliseconds(sample.FrameTicks):F6},{sample.StepBytes},{row.Awake},{row.Pairs},{row.ChainContacts},{row.ToiIterations},{(row.Queries.HasValue ? 1 : 0)}"));
        }
    }

    private static double Milliseconds(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;

    internal static void Require(bool condition, int frame, string problem)
    {
        if (!condition)
            throw new InvalidOperationException($"Frame {frame}: {problem}");
    }

    internal sealed class QueryResults<THit>
    {
        public readonly SwiftList<THit> Ray = new(capacity: 4);
        public readonly SwiftList<THit> Overlap = new(capacity: 4);
        public int RayCount;
        public int OverlapCount;
    }

    internal readonly record struct FrameTiming(long StepTicks, long FrameTicks, long StepBytes);

    internal readonly record struct Observation(ChronicleHash State, ChronicleHash? Queries, int Registered,
        int Awake, int Pairs, int ChainContacts, int LargestChain, int ToiIterations);
}
