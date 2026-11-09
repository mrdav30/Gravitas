using System;
using System.Linq;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using static FixedMathSharp.FixedMath;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class QualityStackWorkload
{
    private const int Frames = 256;
    private static readonly Fixed64 Gravity = Fixed64.FromFraction(49, 5);
    private static readonly Fixed64 TiltRadians = FixedMath.DegToRad((Fixed64)15);
    private static readonly PhysicsMaterial Material = new(Fixed64.Half, Fixed64.FromFraction(1, 3), Fixed64.Zero);

    internal static int Verify()
    {
        foreach (bool planar in new[] { true, false })
        {
            foreach (int ratio in new[] { 1, 10, 100 })
                Verify(planar, ratio, tilted: false);
            Verify(planar, 1, tilted: true);
        }
        return 0;
    }

    private static void Verify(bool planar, int ratio, bool tilted)
    {
        string id = $"quality-{(planar ? "2d" : "3d")}-{(tilted ? "tilted-capsule" : $"stack-m{ratio}")}-v1";
        bool writeFrames = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        Result first = Run(planar, ratio, tilted, observe: true, writeFrames ? id : null);
        Result second = Run(planar, ratio, tilted, observe: true);
        Result control = Run(planar, ratio, tilted, observe: false);
        Require(first == second && first.Terminal == control.Terminal, Frames, $"{id} replay changed");
        Require(first.PeakNeighbors == (tilted ? 1 : 2) && first.PeakSolverDepth > Fixed64.Zero,
            Frames, $"{id} missed its intended support contacts");
        Reading end = first.Final;
        // A failed quality bound is a measured result. Replay and validity are hard gates.
        bool bounded = first.LowestHeight > Fixed64.Zero && end.Residual <= Fixed64.FromFraction(1, 16)
            && end.NeighborPairs == (tilted ? 1 : 2)
            && end.Drift <= (tilted ? (Fixed64)2 : Fixed64.FromFraction(1, 8))
            && (tilted ? first.PeakAngle >= (Fixed64)5 : end.Angle <= (Fixed64)5);
        Console.WriteLine($"{id} trace {first.Trace}, terminal {first.Terminal}, quality {(bounded ? "within" : "outside")} bounds");
        Console.WriteLine(FormattableString.Invariant(
            $"  Solver manifold / residual depth peak {first.PeakSolverDepth:F6} / {first.PeakResidual:F6}, final residual {end.Residual:F6}, final touching pairs {end.ResidualPairs} ({end.NeighborPairs} original neighbors)"));
        Console.WriteLine(FormattableString.Invariant(
            $"  Lateral drift peak/final {first.PeakDrift:F6}/{end.Drift:F6}, angle change peak/final {first.PeakAngle:F4}/{end.Angle:F4} deg, lowest COM {first.LowestHeight:F6}"));
        Console.WriteLine($"  Frames with manifold ID changes {first.ContactChanges}, sleep/wake transitions {first.Sleeps}/{first.Wakes}, asleep at end {end.Sleeping}");
        Console.WriteLine(FormattableString.Invariant(
            $"  Final K linear/angular {end.LinearEnergy:F6}/{end.AngularEnergy:F6}, U {end.Potential:F6}, potential decrease {first.InitialPotential - end.Potential:F6}, mechanical energy change {end.Energy - first.InitialEnergy:F6}"));
    }

    private static Result Run(bool planar, int ratio, bool tilted, bool observe, string frameLabel = null)
    {
        using GravitasWorldContext context = planar
            ? BenchmarkPhysicsScene.CreatePlanarContext(new Vector2d(-16, -8), new Vector2d(16, 16), cellSize: 4)
            : BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        if (!planar)
            Require(context.World.TryAddGrid(new GridConfiguration(new Vector3d(-16, -8, -16), new Vector3d(16, 16, 16),
                topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)4, (Fixed64)4, (Fixed64)4)), out _),
                0, "unable to create the spatial quality grid");
        Configure(context);
        context.Environment.Gravity = Gravity;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        Scene scene = planar ? Planar(context, ratio, tilted) : Spatial(context, ratio, tilted);
        var trace = new ChronicleHashWriter();
        trace.WriteSection("quality-stack-v1", 1);
        var sleeping = new bool[scene.Count];
        Reading initial = observe ? Read(scene, sleeping, 0, ref trace) : default;
        if (frameLabel != null)
        {
            Console.WriteLine("fixture,frame,solver_depth,residual,overlapping_pairs,neighbor_pairs,lowest_com,drift,angle_deg,kinetic_linear,kinetic_angular,potential,sleeping,contact_set");
            WriteFrame(frameLabel, 0, initial);
        }
        Reading row = initial;
        Fixed64 peakSolver = Fixed64.Zero, peakResidual = Fixed64.Zero, peakDrift = Fixed64.Zero, peakAngle = Fixed64.Zero;
        Fixed64 lowest = initial.LowestHeight;
        int changes = 0, sleeps = 0, wakes = 0, peakNeighbors = 0;
        for (int frame = 1; frame <= Frames; frame++)
        {
            context.Simulate();
            context.LateSimulate();
            Require(context.FrameCount == frame && context.LateSimulateToken == frame &&
                (planar ? context.Physics2D.BodyCount : context.Physics.BodyCount) == scene.Count,
                frame, "quality workload lost its fixed step or registered actors");
            if (!observe)
                continue;

            Reading next = Read(scene, sleeping, frame, ref trace);
            changes += next.ContactSet == row.ContactSet ? 0 : 1;
            row = next;
            if (frameLabel != null)
                WriteFrame(frameLabel, frame, row);
            sleeps += row.Sleeps;
            wakes += row.Wakes;
            peakNeighbors = Math.Max(peakNeighbors, row.NeighborPairs);
            peakSolver = Max(peakSolver, row.SolverDepth);
            peakResidual = Max(peakResidual, row.Residual);
            peakDrift = Max(peakDrift, row.Drift);
            peakAngle = Max(peakAngle, row.Angle);
            lowest = Min(lowest, row.LowestHeight);
            ChronicleHash state = context.ComputeReplayHash();
            trace.WriteUInt64(state.Low);
            trace.WriteUInt64(state.High);
        }

        return new Result(context.ComputeReplayHash(), trace.ToHash(), row, peakSolver, peakResidual, peakDrift, peakAngle,
            lowest, changes, sleeps, wakes, peakNeighbors, initial.Potential, initial.Energy);
    }

    private static void WriteFrame(string id, int frame, Reading row) => Console.WriteLine(FormattableString.Invariant(
        $"{id},{frame},{row.SolverDepth:F6},{row.Residual:F6},{row.ResidualPairs},{row.NeighborPairs},{row.LowestHeight:F6},{row.Drift:F6},{row.Angle:F6},{row.LinearEnergy:F6},{row.AngularEnergy:F6},{row.Potential:F6},{row.Sleeping},{row.ContactSet}"));

    private static Reading Read(Scene scene, bool[] sleeping, int frame, ref ChronicleHashWriter trace)
    {
        Fixed64 drift = Fixed64.Zero, angle = Fixed64.Zero, linear = Fixed64.Zero, angular = Fixed64.Zero;
        Fixed64 potential = Fixed64.Zero, lowest = (Fixed64)16;
        int asleep = 0, sleeps = 0, wakes = 0;
        for (int i = 0; i < scene.Count; i++)
        {
            Motion body = scene.ReadBody(i);
            Require(body.Center.MagnitudeSquared < (Fixed64)256 && body.Linear.MagnitudeSquared < (Fixed64)4096,
                frame, "quality sample exceeded the bounded fixture domain");
            drift = Max(drift, body.Drift);
            angle = Max(angle, body.Angle);
            lowest = Min(lowest, body.Center.Y);
            linear += Fixed64.Half * body.Mass * body.Linear.MagnitudeSquared;
            angular += body.AngularEnergy;
            potential += body.Mass * Gravity * body.Center.Y;
            if (body.Sleeping) asleep++;
            if (!sleeping[i] && body.Sleeping) sleeps++;
            if (sleeping[i] && !body.Sleeping) wakes++;
            sleeping[i] = body.Sleeping;
        }

        ContactReading contact = scene.ReadContacts(frame);
        trace.WriteFixed64(drift);
        trace.WriteFixed64(angle);
        trace.WriteFixed64(lowest);
        trace.WriteFixed64(linear);
        trace.WriteFixed64(angular);
        trace.WriteFixed64(potential);
        trace.WriteFixed64(contact.SolverDepth);
        trace.WriteFixed64(contact.Residual);
        trace.WriteInt32(contact.ResidualPairs);
        trace.WriteInt32(contact.NeighborPairs);
        trace.WriteInt32(asleep);
        trace.WriteInt32(sleeps);
        trace.WriteInt32(wakes);
        trace.WriteUInt64(contact.Set.Low);
        trace.WriteUInt64(contact.Set.High);
        return new Reading(drift, angle, lowest, linear, angular, potential, asleep, sleeps, wakes,
            contact.SolverDepth, contact.Residual, contact.ResidualPairs, contact.NeighborPairs, contact.Set);
    }

    private static Scene Spatial(GravitasWorldContext context, int ratio, bool tilted)
    {
        int count = tilted ? 1 : 2;
        var colliders = new LSCollider[count + 1];
        var bodies = new SolidBody[count];
        var inertia = new Fixed3x3[count];
        var floor = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(16, 1, 16), Material));
        floor.InitializeWithNoBody(new BenchmarkMatterAgent(context, new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero)));
        colliders[0] = floor;
        FixedQuaternion initialRotation = tilted
            ? FixedQuaternion.FromAxisAngle(Vector3d.Forward, TiltRadians)
            : FixedQuaternion.Identity;
        for (int i = 0; i < count; i++)
        {
            Vector2d start = Start(i, tilted);
            var position = new Vector3d(start.X, start.Y, Fixed64.Zero);
            LSCollider collider = tilted
                ? new LSCapsuleCollider(ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2, Material))
                : new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(2, 1, 2), Material));
            var body = new SolidBody(new BenchmarkMatterAgent(context, position), collider)
            {
                Mass = (Fixed64)(i == 0 ? 1 : ratio),
                LinearDragCoefficient = Fixed64.Zero,
                SleepEnabled = true
            };
            body.UseManualGrounding();
            body.Initialize(position, initialRotation);
            Require(body.CanTranslate && body.CanRotate, 0, "spatial stack body must move and rotate freely");
            inertia[i] = collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset);
            bodies[i] = body;
            colliders[i + 1] = collider;
        }

        var probes = (from a in Enumerable.Range(0, colliders.Length)
                      from b in Enumerable.Range(a + 1, colliders.Length - a - 1)
                      select (Pair: new CollisionPair(colliders[a], colliders[b]), Neighbors: b == a + 1)).ToArray();
        return new Scene(count, i =>
        {
            SolidBody body = bodies[i];
            Vector3d center = body.WorldCenterOfMass;
            Vector3d localAngular = body.Rotation.Conjugate().Rotate(body.AngularVelocity);
            Require(body.Collider.IsPartitioned && localAngular.MagnitudeSquared < (Fixed64)4096,
                (int)context.FrameCount, "spatial actor left the fixture domain");
            Fixed64 energy = Fixed64.Half * Vector3d.Dot(localAngular, inertia[i] * localAngular);
            return new Motion(center, body.LinearVelocity, body.Mass, energy,
                new Vector2d(center.X - Start(i, tilted).X, center.Z).Magnitude,
                FixedQuaternion.Angle(initialRotation, body.Rotation), body.IsSleeping);
        }, frame => SpatialContacts(probes, frame));
    }

    private static Scene Planar(GravitasWorldContext context, int ratio, bool tilted)
    {
        int count = tilted ? 1 : 2;
        var colliders = new LSCollider2D[count + 1];
        var bodies = new SolidBody2D[count];
        var floor = new LSAABBoxCollider2D(ColliderShapeDefinition2D.AABBox(new Vector2d(16, 1), Material));
        floor.InitializeWithNoBody(new BenchmarkMatterAgent(context, new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half)));
        colliders[0] = floor;
        Fixed64 initialRotation = tilted ? TiltRadians : Fixed64.Zero;
        for (int i = 0; i < count; i++)
        {
            Vector2d start = Start(i, tilted);
            LSCollider2D collider = tilted
                ? new LSCapsuleCollider2D(ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)2, Material))
                : new LSPolygonCollider2D(ColliderShapeDefinition2D.ConvexPolygon(Material,
                    new Vector2d(-Fixed64.One, -Fixed64.Half), new Vector2d(Fixed64.One, -Fixed64.Half),
                    new Vector2d(Fixed64.One, Fixed64.Half), new Vector2d(-Fixed64.One, Fixed64.Half)));
            var body = new SolidBody2D(new BenchmarkMatterAgent(context, new Vector3d(start.X, Fixed64.Zero, start.Y)), collider)
            {
                Mass = (Fixed64)(i == 0 ? 1 : ratio),
                Gravity = new Vector2d(Fixed64.Zero, -Gravity),
                SleepEnabled = true
            };
            body.UseManualGrounding();
            body.Initialize(start, initialRotation);
            Require(body.CanTranslate && body.CanRotate, 0, "planar stack body must move and rotate freely");
            bodies[i] = body;
            colliders[i + 1] = collider;
        }

        return new Scene(count, i =>
        {
            SolidBody2D body = bodies[i];
            Vector2d center = body.WorldCenterOfMass;
            Fixed64 spinSquared = body.AngularVelocity * body.AngularVelocity;
            Require(body.Collider.IsPartitioned && spinSquared < (Fixed64)4096,
                (int)context.FrameCount, "planar actor left the fixture domain");
            Fixed64 angle = FixedMath.RadToDeg(FixedMath.Atan2(FixedMath.Sin(body.Rotation - initialRotation),
                FixedMath.Cos(body.Rotation - initialRotation)).Abs());
            return new Motion(new Vector3d(center.X, center.Y, Fixed64.Zero), body.LinearVelocity.ToVector3d(Fixed64.Zero),
                body.Mass, Fixed64.Half * body.MomentOfInertia * spinSquared,
                (center.X - Start(i, tilted).X).Abs(), angle, body.IsSleeping);
        }, frame => PlanarContacts(colliders, frame));
    }

    private static ContactReading SpatialContacts((CollisionPair Pair, bool Neighbors)[] probes, int frame)
    {
        var ids = new ChronicleHashWriter();
        ids.WriteSection("pairs", 1);
        Fixed64 solver = Fixed64.Zero, residual = Fixed64.Zero;
        int pairs = 0, neighbors = 0;
        Span<ulong> contacts = stackalloc ulong[ContactManifold.MaxContactsPerGroup];
        foreach (var (probe, isNeighbor) in probes)
        {
            LSCollider a = probe.ColliderA;
            LSCollider b = probe.ColliderB;
            int count = 0;
            if ((a.TryGetCollisionPair(b.Id, out CollisionPair pair) || b.TryGetCollisionPair(a.Id, out pair)) && pair.Active)
                foreach (ManifoldContact row in pair.Manifold)
                {
                    Require(!row.DepthIsClamped && count < contacts.Length, frame, "unexpected spatial contact row");
                    solver = Max(solver, row.Depth);
                    contacts[count++] = row.ContactId;
                }
            WriteIds(ref ids, contacts[..count]);
            probe.Manifold.BeginUpdate(frame);
            if (!CollisionDetection.DoCollisionCheck(CollisionWorkItem.Create(probe)))
                continue;
            pairs++;
            if (isNeighbor) neighbors++;
            foreach (ManifoldContact row in probe.Manifold)
            {
                Require(!row.DepthIsClamped, frame, "spatial residual depth clamped");
                residual = Max(residual, row.Depth);
            }
        }
        return new ContactReading(solver, residual, pairs, neighbors, ids.ToHash());
    }

    private static ContactReading PlanarContacts(LSCollider2D[] colliders, int frame)
    {
        var ids = new ChronicleHashWriter();
        ids.WriteSection("pairs", 1);
        Fixed64 solver = Fixed64.Zero, residual = Fixed64.Zero;
        int pairs = 0, neighbors = 0;
        Span<ulong> contacts = stackalloc ulong[2];
        for (int i = 0; i < colliders.Length; i++)
        for (int j = i + 1; j < colliders.Length; j++)
        {
            LSCollider2D a = colliders[i];
            LSCollider2D b = colliders[j];
            int count = 0;
            if ((a.TryGetCollisionPair(b.Id, out CollisionPair2D pair) || b.TryGetCollisionPair(a.Id, out pair)) && pair.IsColliding)
                for (int k = 0; k < pair.Manifold.Count; k++)
                {
                    ManifoldContact2D row = pair.Manifold[k];
                    Require(!row.DepthIsClamped && count < contacts.Length, frame, "unexpected planar contact row");
                    solver = Max(solver, row.Depth);
                    contacts[count++] = row.ContactId;
                }
            WriteIds(ref ids, contacts[..count]);
            if (!CollisionDetection2D.TryCollide(a, b, out Contact2D contact))
                continue;
            Require(!contact.DepthIsClamped, frame, "planar residual depth clamped");
            pairs++;
            if (j == i + 1) neighbors++;
            residual = Max(residual, contact.Depth);
        }
        return new ContactReading(solver, residual, pairs, neighbors, ids.ToHash());
    }

    private static void WriteIds(ref ChronicleHashWriter writer, Span<ulong> ids)
    {
        ids.Sort();
        writer.WriteInt32(ids.Length);
        foreach (ulong id in ids)
            writer.WriteUInt64(id);
    }

    private static Vector2d Start(int index, bool tilted) => tilted
        ? new Vector2d(Fixed64.Zero, Fixed64.FromFraction(9, 8))
        : new Vector2d(Fixed64.FromFraction(index, 64), Fixed64.Half + Fixed64.FromFraction(1 + 33 * index, 32));

    private sealed record Scene(int Count, Func<int, Motion> ReadBody, Func<int, ContactReading> ReadContacts);
    private readonly record struct Motion(Vector3d Center, Vector3d Linear, Fixed64 Mass, Fixed64 AngularEnergy,
        Fixed64 Drift, Fixed64 Angle, bool Sleeping);
    private readonly record struct ContactReading(Fixed64 SolverDepth, Fixed64 Residual, int ResidualPairs,
        int NeighborPairs, ChronicleHash Set);
    private readonly record struct Reading(Fixed64 Drift, Fixed64 Angle, Fixed64 LowestHeight, Fixed64 LinearEnergy,
        Fixed64 AngularEnergy, Fixed64 Potential, int Sleeping, int Sleeps, int Wakes, Fixed64 SolverDepth,
        Fixed64 Residual, int ResidualPairs, int NeighborPairs, ChronicleHash ContactSet)
    {
        internal Fixed64 Energy => LinearEnergy + AngularEnergy + Potential;
    }
    private readonly record struct Result(ChronicleHash Terminal, ChronicleHash Trace, Reading Final, Fixed64 PeakSolverDepth,
        Fixed64 PeakResidual, Fixed64 PeakDrift, Fixed64 PeakAngle, Fixed64 LowestHeight, int ContactChanges,
        int Sleeps, int Wakes, int PeakNeighbors, Fixed64 InitialPotential, Fixed64 InitialEnergy);
}
