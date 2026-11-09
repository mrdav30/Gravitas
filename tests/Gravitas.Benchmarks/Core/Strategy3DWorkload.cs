using System;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using SwiftCollections;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class Strategy3DWorkload
{
    private static readonly PhysicsLayerMask QueryMask = PhysicsLayerMask.FromLayer(0);
    private static readonly Fixed64 FloorTolerance = Fixed64.FromFraction(1, 16);

    internal static int Verify(int actorCount)
    {
        int regionCount = actorCount / ActorsPerRegion;
        return StrategyWorkload.Verify($"strategy-3d-{actorCount}-v1", regionCount, "touching pairs",
            samples => Run(regionCount, samples));
    }

    private static Observation[] Run(int regionCount, Span<FrameTiming> samples)
    {
        using GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        Configure(context);
        context.Environment.Gravity = Fixed64.FromFraction(49, 5);
        context.Environment.MaxSpeed = (Fixed64)1024;
        int extent = regionCount == 1 ? 0 : RegionStride;
        Require(context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-16, -4, -32), new Vector3d(176 + extent, 12, 160 + extent),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)8, (Fixed64)4, (Fixed64)8)), out _),
            0, "unable to create the spatial strategy grid");

        var regions = new Region[regionCount];
        for (int i = 0; i < regions.Length; i++)
        {
            Vector2d origin = RegionOrigin(i);
            regions[i] = CreateRegion(context, new Vector3d(origin.X, Fixed64.Zero, origin.Y));
        }

        return Capture(context, regions, (region, frame) => ApplyCommands(context, region, frame),
            region => RunQueries(context, region), ObserveQueries,
            (queries, frame) => ObserveWork(context, regions, queries, frame), samples);
    }

    private static Region CreateRegion(GravitasWorldContext context, Vector3d origin)
    {
        LSCollider floor = Box(context, origin + new Vector3d(80, -1, 64),
            new Vector3d(RegionStride, 2, RegionStride), layer: 1);
        LSCollider wall = Box(context, origin + new Vector3d(8, 4, -16),
            new Vector3d(Fixed64.Half, (Fixed64)8, (Fixed64)12));
        LSCollider obstacle = Box(context, origin + new Vector3d(14, 4, -16), new Vector3d(2, 8, 12));
        var actors = new SolidBody[ActorsPerRegion];
        for (int slot = 0; slot < actors.Length; slot++)
            actors[slot] = SpawnActor(context, origin, slot);
        return new Region(origin, actors, floor, wall, obstacle, new QueryResults<Physics3DHit>());
    }

    private static void ApplyCommands(GravitasWorldContext context, Region region, int frame)
    {
        SolidBody[] actors = region.Actors;
        switch (frame)
        {
            case SparseStart:
                for (int slot = EncounterActors; slot < EncounterActors + SparseActors; slot++)
                    actors[slot].AddLinearImpulse(new Vector3d(0, 0, 4));
                break;
            case RetireActorsFrame:
            case RetireProjectilesFrame:
                for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
                {
                    SolidBody retired = actors[slot];
                    retired.Deactivate();
                    if (retired.Active || retired.Collider.Id >= 0)
                        Require(false, frame, $"slot {slot} remained registered");
                    actors[slot] = null;
                }
                break;
            case FireFrame:
                for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
                {
                    SolidBody projectile = Sphere(context, region.Origin + new Vector3d(0, 4, ProjectileLaneZ(slot)),
                        Fixed64.Half);
                    projectile.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
                    projectile.AddLinearImpulse(new Vector3d(ProjectileSpeed, 0, 0));
                    Require(projectile.Position3d.X + projectile.LinearVelocity.X * context.DeltaTime >
                        region.Origin.X + WallX + Fixed64.One,
                        frame, "projectile would not cross the wall without collision response");
                    actors[slot] = projectile;
                }
                break;
            case CrowdedStart:
                for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
                    actors[slot] = SpawnActor(context, region.Origin, slot);
                for (int slot = 0; slot < EncounterActors; slot++)
                    actors[slot].AddLinearImpulse(new Vector3d(EncounterImpulseX(slot), 0, 0));
                break;
        }
    }

    private static SolidBody SpawnActor(GravitasWorldContext context, Vector3d origin, int slot)
    {
        Vector2d position = ActorPosition(slot);
        SolidBody body = Sphere(context, origin + new Vector3d(position.X, Fixed64.One, position.Y), Fixed64.One);
        body.Sleep();
        return body;
    }

    private static SolidBody Sphere(GravitasWorldContext context, Vector3d position, Fixed64 radius) =>
        Register(context, position, new LSSphereCollider(ColliderShapeDefinition.Sphere(radius, PhysicsMaterial.Frictionless)),
            PhysicsMobilityType.Dynamic);

    private static LSCollider Box(GravitasWorldContext context, Vector3d position, Vector3d size, int layer = 0)
    {
        var collider = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(size, PhysicsMaterial.Frictionless))
        {
            Layer = new PhysicsLayer(layer)
        };
        return Register(context, position, collider, PhysicsMobilityType.Static).Collider;
    }

    private static SolidBody Register(GravitasWorldContext context, Vector3d position,
        LSCollider collider, PhysicsMobilityType mobility)
    {
        var body = new SolidBody(new BenchmarkMatterAgent(context, position), collider)
        {
            Mass = Fixed64.One,
            LinearDragCoefficient = Fixed64.Zero,
            SleepEnabled = true
        };
        body.UseManualGrounding();
        body.Initialize(position, FixedQuaternion.Identity, mobility);
        return body;
    }

    private static void RunQueries(GravitasWorldContext context, Region region)
    {
        Vector3d origin = region.Origin;
        QueryResults<Physics3DHit> hits = region.Hits;
        hits.RayCount = context.Query3D.RaycastAll(origin + new Vector3d(6, 4, -12),
            origin + new Vector3d(18, 4, -12), QueryMask, hits.Ray);
        hits.OverlapCount = context.Query3D.OverlapCircleAll(origin + new Vector3d(32, 1, 8),
            (Fixed64)4, QueryMask, hits.Overlap);
    }

    private static void ObserveQueries(Region region, int frame, ref ChronicleHashWriter query)
    {
        QueryResults<Physics3DHit> hits = region.Hits;
        Require(hits.RayCount == 2 && hits.Ray.Count == 2 &&
            hits.Ray[0].Collider == region.Wall && hits.Ray[1].Collider == region.Obstacle,
            frame, "ray did not return both obstacles in travel order");
        WriteHits(ref query, hits.Ray);

        Require(hits.OverlapCount == 1 && hits.Overlap.Count == 1 &&
            hits.Overlap[0].Collider == region.Actors[EncounterActors].Collider,
            frame, "overlap did not return the designated scout");
        WriteHits(ref query, hits.Overlap);
    }

    private static void WriteHits(ref ChronicleHashWriter writer, SwiftList<Physics3DHit> hits)
    {
        writer.WriteInt32(hits.Count);
        foreach (Physics3DHit hit in hits.AsReadOnlySpan())
        {
            writer.WriteInt32(hit.Collider.Id);
            writer.WriteFixed64(hit.Distance);
            writer.WriteVector3d(hit.Point);
            writer.WriteVector3d(hit.Normal);
            writer.WriteVector3d(hit.Direction);
        }
    }

    private static Observation ObserveWork(GravitasWorldContext context, Region[] regions, ChronicleHash? queries, int frame)
    {
        int registered = 0;
        int awake = 0;
        int touchingPairs = 0;
        int chainContacts = 0;
        int largestChain = 0;
        int toiIterations = 0;
        foreach (Region region in regions)
        {
            var work = ObserveRegion(region, frame);
            registered += work.Registered;
            awake += work.Awake;
            touchingPairs += work.TouchingPairs;
            chainContacts += work.ChainContacts;
            largestChain = Math.Max(largestChain, work.LargestChain);
            toiIterations += work.ToiIterations;
        }

        Require(context.Physics.BodyCount == registered && context.Physics.ColliderCount == registered + 3 * regions.Length,
            frame, "incorrect spatial world registration count");
        if (frame <= StageFrames)
            Require(touchingPairs == 0, frame, "quiet actors processed touching pairs");
        return new Observation(context.ComputeReplayHash(), queries, registered, awake,
            touchingPairs, chainContacts, largestChain, toiIterations);
    }

    private static (int Registered, int Awake, int TouchingPairs, int ChainContacts, int LargestChain, int ToiIterations)
        ObserveRegion(Region region, int frame)
    {
        SolidBody[] actors = region.Actors;
        int registered = 0;
        int awake = 0;
        int toiIterations = 0;
        int touchingPairs = CountTouching(region.Floor, frame) + CountTouching(region.Wall, frame)
            + CountTouching(region.Obstacle, frame);
        for (int slot = 0; slot < actors.Length; slot++)
        {
            SolidBody actor = actors[slot];
            if (actor == null)
                continue;
            Require(actor.Active && actor.Collider.Id >= 0 && actor.Collider.IsPartitioned, frame,
                "a populated actor slot lost registration or grid coverage");
            registered++;
            if (!actor.IsSleeping)
                awake++;
            toiIterations += actor.LastContinuousCollisionToiIterationCount;
            Require(!actor.LastContinuousCollisionToiIterationLimitReached, frame, "projectile exceeded its TOI budget");
            touchingPairs += CountTouching(actor.Collider, frame);
            bool projectile = slot >= FirstProjectileSlot && frame is >= FireFrame and < RetireProjectilesFrame;
            if (!projectile)
                Require((actor.Position3d.Y - Fixed64.One).Abs() <= FloorTolerance, frame, "a floor actor left its support band");
        }

        CheckPopulation(registered, awake, frame);
        if (frame == CrowdedStart - 1)
            for (int slot = EncounterActors; slot < EncounterActors + SparseActors; slot++)
            {
                SolidBody scout = actors[slot];
                Require(scout.Position3d.Z > region.Origin.Z + (Fixed64)9 && IsTouching(scout.Collider, region.Floor, frame),
                    frame, $"sparse actor {slot} did not move along the floor");
            }
        if (frame == FireFrame)
            for (int slot = FirstProjectileSlot; slot < actors.Length; slot++)
            {
                SolidBody projectile = actors[slot];
                Fixed64 contactX = region.Origin.X + WallX - Fixed64.Half;
                Require(projectile.LastContinuousCollisionToiIterationCount > 0 &&
                    (projectile.Position3d.X - contactX).Abs() <= Fixed64.Epsilon &&
                    projectile.LinearVelocity.X == Fixed64.Zero &&
                    projectile.Position3d.Y < (Fixed64)4 && projectile.Position3d.Y > Fixed64.One,
                    frame, $"projectile {slot} missed its wall stop at X {contactX} above the floor " +
                           $"(position {projectile.Position3d}, velocity {projectile.LinearVelocity})");
            }

        var chain = ObserveChain(actors,
            static (first, second) => IsTouching(first.Collider, second.Collider, first.Context.FrameCount));
        return (registered, awake, touchingPairs, chain.Contacts, chain.Largest, toiIterations);
    }

    // Each pair is stored on only one collider. Holder references are excluded.
    private static int CountTouching(LSCollider collider, int frame)
    {
        int count = 0;
        if (collider.CollisionPairs is { } pairs)
            foreach (var entry in pairs)
                if (IsTouching(entry.Value, frame))
                    count++;
        return count;
    }

    private static bool IsTouching(LSCollider first, LSCollider second, long frame) =>
        (first.TryGetCollisionPair(second.Id, out CollisionPair pair) || second.TryGetCollisionPair(first.Id, out pair))
        && IsTouching(pair, frame);

    private static bool IsTouching(CollisionPair pair, long frame) =>
        pair.Active && pair.LastFrame == frame && pair.Manifold.HasContact;

    private readonly record struct Region(Vector3d Origin, SolidBody[] Actors, LSCollider Floor,
        LSCollider Wall, LSCollider Obstacle, QueryResults<Physics3DHit> Hits);
}
