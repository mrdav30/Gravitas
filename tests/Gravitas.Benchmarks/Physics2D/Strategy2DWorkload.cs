using System;
using System.Diagnostics;
using System.Linq;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Benchmarks;

internal static class Strategy2DWorkload
{
    private const string WorkloadId = "strategy-2d-256-v1";
    private const int ActorCount = 256;
    private const int EncounterActors = 24;
    private const int SparseActors = 8;
    private const int ProjectileCount = 2;
    private const int FirstProjectileSlot = ActorCount - ProjectileCount;
    private const int StageFrames = 32;
    private const int FrameCount = 3 * StageFrames;
    private const int SparseStart = StageFrames + 1;
    private const int CrowdedStart = 2 * StageFrames + 1;
    private const int QuietWarmupFrames = 8;
    private const int RetireActorsFrame = 40;
    private const int FireFrame = 44;
    private const int RetireProjectilesFrame = 57;
    private static readonly Fixed64 WallX = Fixed64.FromFraction(31, 4);
    private static readonly Vector2d ProjectileImpulse = new(900, 0);

    internal static int Verify()
    {
        Observation[] baseline = Run(Span<(long, long)>.Empty);
        Span<(long Ticks, long Bytes)> samples = stackalloc (long, long)[3];
        samples.Clear();
        Observation[] measured = Run(samples);
        var trace = new ChronicleHashWriter();
        trace.WriteSection(WorkloadId, 1);
        for (int i = 0; i < baseline.Length; i++)
        {
            Observation row = baseline[i];
            if (row != measured[i])
                throw new InvalidOperationException(
                    $"{WorkloadId}, frame {i + 1}: replay divergence\nExpected: {row}\nObserved: {measured[i]}");
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
            trace.WriteInt32(row.Candidates);
            trace.WriteInt32(row.ChainContacts);
            trace.WriteInt32(row.LargestChain);
            trace.WriteInt32(row.ToiIterations);
        }

        int firstFullChain = Array.FindIndex(baseline, CrowdedStart - 1, row => row.LargestChain == EncounterActors) + 1;
        Require(firstFullChain > 0, FrameCount, "the encounter never produced a connected contact chain");
        Console.WriteLine($"{WorkloadId} matched all {FrameCount} frames. Trace {trace.ToHash()}");
        Console.WriteLine($"First full contact chain at frame {firstFullChain}.");
        WriteStage("quiet", baseline[..StageFrames], samples[0], StageFrames - QuietWarmupFrames);
        WriteStage("sparse", baseline[StageFrames..(2 * StageFrames)], samples[1], StageFrames);
        WriteStage("crowded", baseline[(2 * StageFrames)..], samples[2], StageFrames);
        return 0;
    }

    private static Observation[] Run(Span<(long Ticks, long Bytes)> samples)
    {
        using GravitasWorldContext context =
            BenchmarkPhysicsScene.CreatePlanarContext(new Vector2d(-16, -32), new Vector2d(176, 160), cellSize: 8);
        context.SetFrameRate(60);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        context.Settings.DiscreteSolverIterations = 6;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        LSCollider2D wall = Register(context, new Vector2d(8, -16),
            new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, (Fixed64)12)), PhysicsMobilityType.Static).Collider;
        LSCollider2D obstacle = Register(context, new Vector2d(14, -16),
            new LSAABBoxCollider2D(new Vector2d(2, 12)), PhysicsMobilityType.Static).Collider;

        var actors = new SolidBody2D[ActorCount];
        for (int slot = 0; slot < actors.Length; slot++)
            actors[slot] = SpawnActor(context, slot);

        var trace = new Observation[FrameCount];
        var hits = new SwiftList<Physics2DHit>(capacity: 4);
        for (int frame = 1; frame <= FrameCount; frame++)
        {
            ApplyCommands(context, actors, frame);
            bool sample = !samples.IsEmpty && frame > QuietWarmupFrames;
            long bytesBefore = sample ? GC.GetAllocatedBytesForCurrentThread() : 0;
            long start = sample ? Stopwatch.GetTimestamp() : 0;
            context.Simulate();
            context.LateSimulate();
            if (sample)
            {
                long elapsed = Stopwatch.GetTimestamp() - start;
                long bytes = GC.GetAllocatedBytesForCurrentThread() - bytesBefore;
                ref var stage = ref samples[(frame - 1) / StageFrames];
                stage.Ticks += elapsed;
                stage.Bytes += bytes;
            }

            Require(context.FrameCount == frame && context.LateSimulateToken == frame,
                frame, "the complete host step did not advance both clocks");
            ChronicleHash? queryHash = frame % 8 == 0
                ? ObserveQueries(context, hits, wall, obstacle, actors[EncounterActors].Collider, frame)
                : null;
            trace[frame - 1] = ObserveWork(context, actors, queryHash, frame);
        }
        return trace;
    }

    private static void ApplyCommands(GravitasWorldContext context, SolidBody2D[] actors, int frame)
    {
        switch (frame)
        {
            case SparseStart:
                for (int slot = EncounterActors; slot < EncounterActors + SparseActors; slot++)
                    actors[slot].AddLinearImpulse(new Vector2d(0, 4));
                break;
            case RetireActorsFrame:
            case RetireProjectilesFrame:
                for (int slot = FirstProjectileSlot; slot < ActorCount; slot++)
                {
                    SolidBody2D retired = actors[slot];
                    retired.Deactivate();
                    Require(!retired.Active && retired.Collider.Id < 0, frame, $"slot {slot} remained registered");
                    actors[slot] = null;
                }
                break;
            case FireFrame:
                for (int slot = FirstProjectileSlot; slot < ActorCount; slot++)
                {
                    SolidBody2D projectile = Register(context, new Vector2d(0, -16 - 2 * (slot - FirstProjectileSlot)),
                        new LSCircleCollider2D(Fixed64.Half), PhysicsMobilityType.Dynamic);
                    projectile.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
                    projectile.AddLinearImpulse(ProjectileImpulse);
                    Require(projectile.Position.X + projectile.LinearVelocity.X * context.DeltaTime > WallX + Fixed64.One,
                        frame, "projectile would not cross the wall without collision response");
                    actors[slot] = projectile;
                }
                break;
            case CrowdedStart:
                for (int slot = FirstProjectileSlot; slot < ActorCount; slot++)
                    actors[slot] = SpawnActor(context, slot);
                // A constant velocity difference closes all 23 initially equal gaps together.
                for (int slot = 0; slot < EncounterActors; slot++)
                    actors[slot].AddLinearImpulse(new Vector2d(3 * (EncounterActors - 1 - 2 * slot), 0));
                break;
        }
    }

    private static SolidBody2D SpawnActor(GravitasWorldContext context, int slot)
    {
        int gridIndex = slot - EncounterActors;
        Vector2d position = gridIndex < 0
            ? new Vector2d(Fixed64.FromFraction(9 * slot, 4), (Fixed64)144)
            : new Vector2d(32 + 8 * (gridIndex % 16), 8 + 8 * (gridIndex / 16));
        SolidBody2D body = Register(context, position, new LSCircleCollider2D(Fixed64.One), PhysicsMobilityType.Dynamic);
        body.Sleep();
        return body;
    }

    private static SolidBody2D Register(GravitasWorldContext context, Vector2d position,
        LSCollider2D collider, PhysicsMobilityType mobility)
    {
        var body = new SolidBody2D(new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y)), collider)
        {
            Mass = Fixed64.One,
            GravityScale = Fixed64.Zero,
            SleepEnabled = true
        };
        body.UseManualGrounding();
        body.Initialize(position, motionType: mobility);
        return body;
    }

    private static ChronicleHash ObserveQueries(GravitasWorldContext context, SwiftList<Physics2DHit> hits,
        LSCollider2D wall, LSCollider2D obstacle, LSCollider2D scout, int frame)
    {
        var query = new ChronicleHashWriter();
        int count = context.Query2D.RaycastAll(new Vector2d(6, -12), new Vector2d(18, -12), hits);
        Require(count == 2 && hits[0].Collider == wall && hits[1].Collider == obstacle,
            frame, "ray did not return both obstacles in travel order");
        WriteHits(ref query, hits);

        count = context.Query2D.OverlapCircleAll(new Vector2d(32, 8), (Fixed64)4, hits);
        Require(count == 1 && hits[0].Collider == scout, frame, "overlap did not return the designated scout");
        WriteHits(ref query, hits);
        return query.ToHash();
    }

    private static void WriteHits(ref ChronicleHashWriter writer, SwiftList<Physics2DHit> hits)
    {
        writer.WriteInt32(hits.Count);
        foreach (Physics2DHit hit in hits.AsReadOnlySpan())
        {
            writer.WriteInt32(hit.Collider.Id);
            writer.WriteFixed64(hit.Distance);
            writer.WriteVector2d(hit.Point);
            writer.WriteVector2d(hit.Normal);
        }
    }

    private static Observation ObserveWork(GravitasWorldContext context, SolidBody2D[] actors,
        ChronicleHash? queries, int frame)
    {
        int expectedRegistered = frame is >= RetireActorsFrame and < FireFrame or >= RetireProjectilesFrame and < CrowdedStart
            ? ActorCount - ProjectileCount : ActorCount;
        int registered = 0;
        int awake = 0;
        int toiIterations = 0;
        foreach (SolidBody2D actor in actors)
        {
            if (actor == null)
                continue;
            Require(actor.Active && actor.Collider.Id >= 0, frame, "a populated actor slot lost registration");
            registered++;
            if (!actor.IsSleeping)
                awake++;
            toiIterations += actor.LastContinuousCollisionToiIterationCount;
            Require(!actor.LastContinuousCollisionToiIterationLimitReached, frame, "projectile exceeded its TOI budget");
        }

        Require(registered == expectedRegistered && context.Physics2D.BodyCount == registered &&
            context.Physics2D.ColliderCount == registered + 2, frame, "incorrect actor or obstacle registration count");
        if (frame <= StageFrames)
            Require(awake == 0 && context.Physics2D.LastBroadPhaseCandidateCount == 0, frame, "quiet actors did work");
        if (frame == SparseStart)
            Require(awake == SparseActors, frame, "the sparse wake burst changed population");
        if (frame is >= SparseStart and < CrowdedStart)
        {
            int activeProjectiles = frame is >= FireFrame and < RetireProjectilesFrame ? ProjectileCount : 0;
            Require(awake <= SparseActors + activeProjectiles, frame, "the sparse stage woke extra actors");
        }
        if (frame == CrowdedStart - 1)
            for (int slot = EncounterActors; slot < EncounterActors + SparseActors; slot++)
                Require(actors[slot].Position.Y > (Fixed64)9, frame, $"sparse actor {slot} did not move");
        if (frame == CrowdedStart)
            Require(awake == SparseActors + EncounterActors, frame, "the crowded wake burst changed population");
        if (frame == FireFrame)
            for (int slot = FirstProjectileSlot; slot < ActorCount; slot++)
            {
                SolidBody2D projectile = actors[slot];
                Fixed64 contactX = WallX - Fixed64.Half;
                Require(projectile.LastContinuousCollisionToiIterationCount > 0 &&
                    (projectile.Position.X - contactX).Abs() <= Fixed64.Epsilon,
                    frame, $"projectile {slot} stopped at {projectile.Position.X}, expected {contactX}");
            }

        int chain = 1;
        int largestChain = 0;
        int chainContacts = 0;
        for (int i = 1; i < EncounterActors; i++)
        {
            LSCollider2D first = actors[i - 1].Collider;
            LSCollider2D second = actors[i].Collider;
            bool hasPair = first.TryGetCollisionPair(second.Id, out CollisionPair2D pair)
                || second.TryGetCollisionPair(first.Id, out pair);
            if (hasPair && pair.IsColliding && pair.Manifold.Count > 0)
            {
                chainContacts++;
                largestChain = Math.Max(largestChain, ++chain);
            }
            else
                chain = 1;
        }
        return new Observation(context.ComputeReplayHash(), queries, registered, awake,
            context.Physics2D.LastBroadPhaseCandidateCount, chainContacts, largestChain, toiIterations);
    }

    private static void WriteStage(string stage, Observation[] rows, (long Ticks, long Bytes) sample, int sampleCount)
    {
        int queryFrames = rows.Count(row => row.Queries.HasValue);
        Console.WriteLine($"{stage}: registered {rows.Min(row => row.Registered)}–{rows.Max(row => row.Registered)}, " +
            $"peak awake {rows.Max(row => row.Awake)}, candidates {rows.Sum(row => row.Candidates)}, " +
            $"peak chain contacts {rows.Max(row => row.ChainContacts)}, largest chain {rows.Max(row => row.LargestChain)}, " +
            $"TOI iterations {rows.Sum(row => row.ToiIterations)}, queries {2 * queryFrames}, hits {3 * queryFrames}");
        Console.WriteLine(FormattableString.Invariant(
            $"  Physics smoke ({sampleCount} frames): {Stopwatch.GetElapsedTime(0, sample.Ticks).TotalMilliseconds / sampleCount:F3} ms/frame, {sample.Bytes} managed bytes total"));
    }

    private static void Require(bool condition, int frame, string problem)
    {
        if (!condition)
            throw new InvalidOperationException($"{WorkloadId}, frame {frame}: {problem}");
    }

    private readonly record struct Observation(ChronicleHash State, ChronicleHash? Queries, int Registered,
        int Awake, int Candidates, int ChainContacts, int LargestChain, int ToiIterations);
}
