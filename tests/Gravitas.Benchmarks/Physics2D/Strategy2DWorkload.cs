using System;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class Strategy2DWorkload
{
    private static readonly Vector2d ProjectileImpulse = new(ProjectileSpeed, 0);

    internal static int Verify(int actorCount)
    {
        int regionCount = actorCount / ActorsPerRegion;
        return StrategyWorkload.Verify($"strategy-2d-{actorCount}-v1", regionCount, "candidates",
            samples => Run(regionCount, samples));
    }

    private static Observation[] Run(int regionCount, Span<FrameTiming> samples)
    {
        int extent = regionCount == 1 ? 0 : RegionStride;
        using GravitasWorldContext context =
            BenchmarkPhysicsScene.CreatePlanarContext(new Vector2d(-16, -32),
                new Vector2d(176 + extent, 160 + extent), cellSize: 8);
        Configure(context);
        var regions = new Region[regionCount];
        for (int i = 0; i < regions.Length; i++)
            regions[i] = CreateRegion(context, RegionOrigin(i));

        return Capture(context, regions, (region, frame) => ApplyCommands(context, region, frame),
            region => RunQueries(context, region), ObserveQueries,
            (queries, frame) => ObserveWork(context, regions, queries, frame), samples);
    }

    private static Region CreateRegion(GravitasWorldContext context, Vector2d origin)
    {
        LSCollider2D wall = Register(context, origin + new Vector2d(8, -16),
            new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, (Fixed64)12)), PhysicsMobilityType.Static).Collider;
        LSCollider2D obstacle = Register(context, origin + new Vector2d(14, -16),
            new LSAABBoxCollider2D(new Vector2d(2, 12)), PhysicsMobilityType.Static).Collider;
        var actors = new SolidBody2D[ActorsPerRegion];
        for (int slot = 0; slot < actors.Length; slot++)
            actors[slot] = SpawnActor(context, origin, slot);
        return new Region(origin, actors, wall, obstacle, new QueryResults<Physics2DHit>());
    }

    private static void ApplyCommands(GravitasWorldContext context, Region region, int frame)
    {
        SolidBody2D[] actors = region.Actors;
        switch (frame)
        {
            case SparseStart:
                for (int slot = EncounterActors; slot < EncounterActors + SparseActors; slot++)
                    actors[slot].AddLinearImpulse(new Vector2d(0, 4));
                break;
            case RetireActorsFrame:
            case RetireProjectilesFrame:
                for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
                {
                    SolidBody2D retired = actors[slot];
                    retired.Deactivate();
                    if (retired.Active || retired.Collider.Id >= 0)
                        Require(false, frame, $"slot {slot} remained registered");
                    actors[slot] = null;
                }
                break;
            case FireFrame:
                for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
                {
                    SolidBody2D projectile = Register(context, region.Origin + new Vector2d(0, ProjectileLaneZ(slot)),
                        new LSCircleCollider2D(Fixed64.Half), PhysicsMobilityType.Dynamic);
                    projectile.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
                    projectile.AddLinearImpulse(ProjectileImpulse);
                    Require(projectile.Position.X + projectile.LinearVelocity.X * context.DeltaTime > region.Origin.X + WallX + Fixed64.One,
                        frame, "projectile would not cross the wall without collision response");
                    actors[slot] = projectile;
                }
                break;
            case CrowdedStart:
                for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
                    actors[slot] = SpawnActor(context, region.Origin, slot);
                for (int slot = 0; slot < EncounterActors; slot++)
                    actors[slot].AddLinearImpulse(new Vector2d(EncounterImpulseX(slot), 0));
                break;
        }
    }

    private static SolidBody2D SpawnActor(GravitasWorldContext context, Vector2d origin, int slot)
    {
        SolidBody2D body = Register(context, origin + ActorPosition(slot),
            new LSCircleCollider2D(Fixed64.One), PhysicsMobilityType.Dynamic);
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

    private static void RunQueries(GravitasWorldContext context, Region region)
    {
        Vector2d origin = region.Origin;
        QueryResults<Physics2DHit> hits = region.Hits;
        hits.RayCount = context.Query2D.RaycastAll(origin + new Vector2d(6, -12),
            origin + new Vector2d(18, -12), hits.Ray);
        hits.OverlapCount = context.Query2D.OverlapCircleAll(origin + new Vector2d(32, 8), (Fixed64)4, hits.Overlap);
    }

    private static void ObserveQueries(Region region, int frame, ref ChronicleHashWriter query)
    {
        QueryResults<Physics2DHit> hits = region.Hits;
        Require(hits.RayCount == 2 && hits.Ray.Count == 2 &&
            hits.Ray[0].Collider == region.Wall && hits.Ray[1].Collider == region.Obstacle,
            frame, "ray did not return both obstacles in travel order");
        WriteHits(ref query, hits.Ray);

        Require(hits.OverlapCount == 1 && hits.Overlap.Count == 1 &&
            hits.Overlap[0].Collider == region.Actors[EncounterActors].Collider,
            frame, "overlap did not return the designated scout");
        WriteHits(ref query, hits.Overlap);
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

    private static Observation ObserveWork(GravitasWorldContext context, Region[] regions,
        ChronicleHash? queries, int frame)
    {
        int registered = 0;
        int awake = 0;
        int chainContacts = 0;
        int largestChain = 0;
        int toiIterations = 0;
        foreach (Region region in regions)
        {
            var work = ObserveRegion(region, frame);
            registered += work.Registered;
            awake += work.Awake;
            chainContacts += work.ChainContacts;
            largestChain = Math.Max(largestChain, work.LargestChain);
            toiIterations += work.ToiIterations;
        }

        Require(context.Physics2D.BodyCount == registered && context.Physics2D.ColliderCount == registered + 2 * regions.Length,
            frame, "incorrect world registration count");
        int candidates = context.Physics2D.LastBroadPhaseCandidateCount;
        if (frame <= StageFrames)
            Require(candidates == 0, frame, "quiet actors dispatched collision candidates");
        return new Observation(context.ComputeReplayHash(), queries, registered, awake,
            candidates, chainContacts, largestChain, toiIterations);
    }

    private static (int Registered, int Awake, int ChainContacts, int LargestChain, int ToiIterations)
        ObserveRegion(Region region, int frame)
    {
        SolidBody2D[] actors = region.Actors;
        int registered = 0;
        int awake = 0;
        int toiIterations = 0;
        foreach (SolidBody2D actor in actors)
        {
            if (actor == null)
                continue;
            Require(actor.Active && actor.Collider.Id >= 0 && actor.Collider.IsPartitioned, frame,
                "a populated actor slot lost registration or grid coverage");
            registered++;
            if (!actor.IsSleeping)
                awake++;
            toiIterations += actor.LastContinuousCollisionToiIterationCount;
            Require(!actor.LastContinuousCollisionToiIterationLimitReached, frame, "projectile exceeded its TOI budget");
        }

        CheckPopulation(registered, awake, frame);
        if (frame == CrowdedStart - 1)
            for (int slot = EncounterActors; slot < EncounterActors + SparseActors; slot++)
                Require(actors[slot].Position.Y > region.Origin.Y + (Fixed64)9, frame, $"sparse actor {slot} did not move");
        if (frame == FireFrame)
            for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
            {
                SolidBody2D projectile = actors[slot];
                Fixed64 contactX = region.Origin.X + WallX - Fixed64.Half;
                Require(projectile.LastContinuousCollisionToiIterationCount > 0 &&
                    (projectile.Position.X - contactX).Abs() <= Fixed64.Epsilon,
                    frame, $"projectile {slot} stopped at {projectile.Position.X}, expected {contactX}");
            }

        var chain = ObserveChain(actors, static (first, second) =>
        {
            LSCollider2D a = first.Collider;
            LSCollider2D b = second.Collider;
            bool hasPair = a.TryGetCollisionPair(b.Id, out CollisionPair2D pair)
                || b.TryGetCollisionPair(a.Id, out pair);
            return hasPair && pair.IsColliding && pair.Manifold.Count > 0;
        });
        return (registered, awake, chain.Contacts, chain.Largest, toiIterations);
    }

    private readonly record struct Region(Vector2d Origin, SolidBody2D[] Actors, LSCollider2D Wall, LSCollider2D Obstacle, QueryResults<Physics2DHit> Hits);
}
