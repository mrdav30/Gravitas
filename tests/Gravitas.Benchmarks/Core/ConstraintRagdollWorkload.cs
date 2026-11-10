using System;
using System.Linq;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Constraints;
using Gravitas.Materials;
using static FixedMathSharp.FixedMath;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class ConstraintRagdollWorkload
{
    private const int Links = 6;
    private const int Frames = 256;
    private const int KickFrame = 193;
    private static readonly Fixed64 Gravity = (Fixed64)10;
    private static readonly Fixed64 Limit = Fixed64.Pi / 18;
    private static readonly Fixed64 SpinImpulse = Fixed64.FromFraction(3, 16);
    private static readonly Fixed64 RestSpeed = Fixed64.FromFraction(1, 20);
    private static readonly PhysicsMaterial Material = new(Fixed64.Half, Fixed64.FromFraction(1, 3), Fixed64.Zero);
    private static readonly int[] RestPositions = [0, 3, -2, 2, -1, 1];

    internal static int Verify()
    {
        bool csv = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        if (csv)
            Console.WriteLine("fixture,frame,solver_depth,floor_residual,retained_supports,contact_ids,anchor_gap,prepared_gap,limit_error_deg,lateral_drift,rotation_deg,sleeping,sleeps,wakes,kinetic,potential");

        foreach (bool planar in new[] { true, false })
        foreach (bool rest in new[] { false, true })
        {
            string id = $"quality-{(planar ? "2d" : "3d")}-ragdoll-{(rest ? "rest" : "drop")}-v1";
            Result first = Run(planar, rest, id, observe: true, csv);
            Result repeat = Run(planar, rest, id, observe: true, csv: false);
            Result control = Run(planar, rest, id, observe: false, csv: false);
            Require(first == repeat && first.Terminal == control.Terminal, Frames, $"{id} replay diverged");
            Require(first.FirstSupport > 0 && first.PeakDepth > Fixed64.Zero, Frames, $"{id} missed floor contact");
            if (rest && first.AsleepBeforeKick == Links)
                Require(first.KickWork > Fixed64.Zero, KickFrame, $"{id} received no positive angular impulse work from rest");
            string recovery = rest
                ? $", sleep recovery {(first.AsleepBeforeKick == Links && first.AwakeAfterKick == Links && first.Final.Sleeping == Links ? "within" : "outside")} bounds"
                : "";
            Console.WriteLine($"{id} trace {first.Trace}, terminal {first.Terminal}, geometry {(GeometryWithin(first) ? "within" : "outside")} bounds{recovery}");
            Console.WriteLine(FormattableString.Invariant(
                $"  Solver depth peak {first.PeakDepth:F6}, floor residual peak/final {first.PeakResidual:F6}/{first.Final.Residual:F6}, lowest COM {first.LowestCom:F6}, first support frame {first.FirstSupport}, final retained supports {first.Final.Supports}, supported frames {first.SupportFrames}, contact ID change frames {first.ContactChanges}"));
            Console.WriteLine(FormattableString.Invariant(
                $"  Anchor gap peak/final {first.PeakGap:F6}/{first.Final.Gap:F6}, peak prepared gap {first.PeakPreparedGap:F6}, prepared limit error peak/final {first.PeakLimit:F4}/{first.Final.Limit:F4} deg, lateral drift peak/final {first.PeakDrift:F6}/{first.Final.Drift:F6}, peak rotation {first.PeakRotation:F4} deg"));
            Console.WriteLine(FormattableString.Invariant(
                $"  Sleep/wake transitions {first.Sleeps}/{first.Wakes}, asleep before kick/final {first.AsleepBeforeKick}/{first.Final.Sleeping}, awake after kick {first.AwakeAfterKick}, gravity work {first.GravityWork:F6}, kick work {first.KickWork:F6}, final K/U {first.Final.Kinetic:F6}/{first.Final.Potential:F6}, warm bytes {control.WarmBytes}, peak {control.PeakBytes} on frame {control.PeakBytesFrame}"));
        }
        return 0;
    }

    internal static int VerifyDuration(bool strict = false)
    {
        bool csv = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        if (csv)
            Console.WriteLine("fixture,frame,seconds,solver_depth,floor_residual,retained_supports,contact_ids,anchor_gap,prepared_gap,limit_error_deg,lateral_drift,rotation_deg,sleeping,sleeps,wakes,kinetic,potential");

        bool geometryMet = true;
        foreach (int rate in new[] { 32, 60 })
        foreach (bool planar in new[] { true, false })
        foreach (bool rest in new[] { false, true })
        {
            string id = $"duration-{(planar ? "2d" : "3d")}-ragdoll-{(rest ? "rest" : "drop")}-{rate}hz-v1";
            Result first = Run(planar, rest, id, observe: true, csv, rate);
            Result repeat = Run(planar, rest, id, observe: true, csv: false, rate);
            Result control = Run(planar, rest, id, observe: false, csv: false, rate);
            int frames = rate * QualityDuration.Seconds;
            Require(first == repeat && first.Terminal == control.Terminal, frames, $"{id} replay diverged");
            Require(first.FirstSupport > 0 && first.PeakDepth > Fixed64.Zero && first.Final.Supports > 0,
                frames, $"{id} did not retain intended floor contact");
            if (rest)
                Require(first.Recovery1.AsleepBefore == Links && first.Recovery1.AwakeAfter == Links,
                    2 * rate + 1, $"{id} did not exercise connected wake from supported rest");
            bool within = GeometryWithin(first);
            geometryMet &= within;
            Console.WriteLine($"{id} trace {first.Trace}, terminal {first.Terminal}, geometry {(within ? "within" : "outside")} bounds");
            Console.WriteLine(FormattableString.Invariant(
                $"  Floor residual peak/final {first.PeakResidual:F6}/{first.Final.Residual:F6}, lowest COM {first.LowestCom:F6}, first support frame {first.FirstSupport}, final retained supports {first.Final.Supports}, supported frames {first.SupportFrames}, contact ID change frames {first.ContactChanges}, anchor gap peak/final {first.PeakGap:F6}/{first.Final.Gap:F6}, peak prepared gap {first.PeakPreparedGap:F6}, prepared limit error peak/final {first.PeakLimit:F4}/{first.Final.Limit:F4} deg, lateral drift peak/final {first.PeakDrift:F6}/{first.Final.Drift:F6}, peak rotation {first.PeakRotation:F4} deg"));
            Console.WriteLine(FormattableString.Invariant(
                $"  First pre-pulse all-rest frame {first.FirstRest}, all-rest steps {first.RestFrames}, longest/final rest steps {first.LongestRest}/{first.FinalRest}, sleep/wake transitions {first.Sleeps}/{first.Wakes}, final sleeping {first.Final.Sleeping}, gravity work {first.GravityWork:F6}, final K/U {first.Final.Kinetic:F6}/{first.Final.Potential:F6}, warm bytes {control.WarmBytes}, peak {control.PeakBytes} on frame {control.PeakBytesFrame}"));
            PrintRecovery(2, first.Recovery1);
            PrintRecovery(4, first.Recovery2);
            PrintRecovery(6, first.Recovery3);
        }
        return strict && !geometryMet ? 1 : 0;
    }

    private static void PrintRecovery(int seconds, Recovery recovery) => Console.WriteLine(FormattableString.Invariant(
        $"  Pulse at {seconds}s, asleep before/awake after {recovery.AsleepBefore}/{recovery.AwakeAfter}, first all-rest frame {recovery.RestFrame}, pulse work {recovery.Work:F6}"));

    private static bool GeometryWithin(Result result) => result.Final.Supports > 0 && result.LowestCom > Fixed64.Zero
        && result.Final.Residual <= Fixed64.FromFraction(1, 16) && result.PeakResidual <= Fixed64.FromFraction(1, 8)
        && result.PeakGap <= Fixed64.FromFraction(1, 4) && result.Final.Gap <= Fixed64.FromFraction(1, 16)
        && result.PeakLimit <= (Fixed64)10 && result.Final.Limit <= (Fixed64)3 && result.PeakDrift <= Fixed64.One;

    private static Result Run(bool planar, bool rest, string id, bool observe, bool csv, int durationRate = 0)
    {
        const int extent = 48;
        bool duration = durationRate != 0;
        int rate = duration ? durationRate : 60;
        int frames = duration ? rate * QualityDuration.Seconds : Frames;
        int warmup = duration ? rate : 64;
        using GravitasWorldContext context = planar
            ? Constraint2DBenchmarks.CreateConstraintContext(extent, Gravity, PhysicsSettings.DefaultDiscreteSolverIterations)
            : Constraint3DBenchmarks.CreateConstraintContext(extent, Gravity, PhysicsSettings.DefaultDiscreteSolverIterations);
        context.SetFrameRate(rate);
        ReadOnlySpan<int> supportedX = rest ? RestPositions : default;
        int sleepSteps = durationRate / 2;
        Scene scene = planar ? CreatePlanar(context, supportedX, sleepSteps) : CreateSpatial(context, supportedX, sleepSteps);
        var trace = new ChronicleHashWriter();
        trace.WriteSection(id, 1);
        Result result = new() { PeakBytesFrame = -1 };
        int sleepMask = 0, episode = 0, restSteps = 0;
        Span<Recovery> recoveries = stackalloc Recovery[QualityDuration.Episodes];
        recoveries.Fill(new Recovery(-1, -1, -1, Fixed64.Zero));
        Fixed64 initialPotential = Fixed64.Zero;
        for (int frame = 0; frame <= frames; frame++)
        {
            bool kick = duration ? QualityDuration.IsPulse(frame, rate) : frame == KickFrame;
            if (kick)
            {
                if (duration)
                {
                    episode++;
                    restSteps = 0;
                }
                Fixed64 before = observe ? scene.ReadLink(Links - 1).Kinetic : Fixed64.Zero;
                scene.Kick();
                if (observe)
                {
                    Fixed64 work = scene.ReadLink(Links - 1).Kinetic - before;
                    if (duration)
                        recoveries[episode] = recoveries[episode] with { AsleepBefore = result.Final.Sleeping, Work = work };
                    else
                        result = result with { AsleepBeforeKick = result.Final.Sleeping, KickWork = work };
                }
            }
            if (frame > 0)
            {
                bool measure = !observe && frame > warmup;
                long bytes = measure ? GC.GetAllocatedBytesForCurrentThread() : 0;
                context.Simulate();
                context.LateSimulate();
                if (measure)
                {
                    bytes = GC.GetAllocatedBytesForCurrentThread() - bytes;
                    result = result with { WarmBytes = result.WarmBytes + bytes };
                    if (bytes > result.PeakBytes)
                        result = result with { PeakBytes = bytes, PeakBytesFrame = frame };
                }
                Require(context.FrameCount == frame && context.LateSimulateToken == frame, frame, $"{id} did not complete its fixed step");
                if (duration)
                    Require((planar ? context.Physics2D.BodyCount : context.Physics.BodyCount) == Links
                        && (planar ? context.Physics2D.ColliderCount : context.Physics.ColliderCount) == Links + 1,
                        frame, "ragdoll lost its registered bodies");
            }
            if (!observe)
                continue;

            Frame row = Read(scene, frame, ref sleepMask, ref result);
            if (kick)
            {
                if (duration)
                    recoveries[episode] = recoveries[episode] with { AwakeAfter = Links - row.Sleeping };
                else
                    result = result with { AwakeAfterKick = Links - row.Sleeping };
            }
            if (duration && row.Sleeping == Links && row.Supports > 0)
            {
                if (recoveries[episode].RestFrame < 0)
                    recoveries[episode] = recoveries[episode] with { RestFrame = frame };
                restSteps++;
                result = result with { RestFrames = result.RestFrames + 1, LongestRest = Math.Max(result.LongestRest, restSteps) };
            }
            else
                restSteps = 0;
            if (frame == 0)
            {
                initialPotential = row.Potential;
                Require(row.Gap == Fixed64.Zero && row.Residual == Fixed64.Zero && row.Kinetic == Fixed64.Zero,
                    frame, $"{id} must start aligned, still and above the floor");
            }
            else if (row.Supports > 0)
            {
                if (result.FirstSupport == 0)
                    result = result with { FirstSupport = frame };
                result = result with { SupportFrames = result.SupportFrames + 1 };
            }
            ChronicleHash hash = context.ComputeReplayHash();
            trace.WriteUInt64(hash.Low);
            trace.WriteUInt64(hash.High);
            row.Hash(ref trace);
            if (csv && duration)
                Console.WriteLine(FormattableString.Invariant(
                    $"{id},{frame},{(Fixed64)frame / rate:F6},{row.Depth:F6},{row.Residual:F6},{row.Supports},{row.ContactIds},{row.Gap:F6},{row.PreparedGap:F6},{row.Limit:F4},{row.Drift:F6},{row.Rotation:F4},{row.Sleeping},{row.Sleeps},{row.Wakes},{row.Kinetic:F6},{row.Potential:F6}"));
            else if (csv)
                Console.WriteLine(FormattableString.Invariant(
                    $"{id},{frame},{row.Depth:F6},{row.Residual:F6},{row.Supports},{row.ContactIds},{row.Gap:F6},{row.PreparedGap:F6},{row.Limit:F4},{row.Drift:F6},{row.Rotation:F4},{row.Sleeping},{row.Sleeps},{row.Wakes},{row.Kinetic:F6},{row.Potential:F6}"));
        }
        if (duration)
            Require(episode == QualityDuration.Episodes - 1, frames, "ragdoll missed a scheduled pulse");
        return result with
        {
            Trace = trace.ToHash(), Terminal = context.ComputeReplayHash(), GravityWork = initialPotential - result.Final.Potential,
            FirstRest = recoveries[0].RestFrame, Recovery1 = recoveries[1], Recovery2 = recoveries[2], Recovery3 = recoveries[3],
            FinalRest = restSteps
        };
    }

    private static Frame Read(Scene scene, int frame, ref int sleepMask, ref Result result)
    {
        var contactIds = new ChronicleHashWriter();
        Span<ulong> ids = stackalloc ulong[ContactManifold.MaxContactsPerGroup];
        Fixed64 depth = Fixed64.Zero, residual = Fixed64.Zero, lowest = (Fixed64)8;
        Fixed64 gap = Fixed64.Zero, prepared = Fixed64.Zero, limit = Fixed64.Zero;
        Fixed64 drift = Fixed64.Zero, rotation = Fixed64.Zero, kinetic = Fixed64.Zero, potential = Fixed64.Zero;
        int sleeping = 0, sleeps = 0, wakes = 0, supports = 0;
        for (int i = 0; i < Links; i++)
        {
            LinkReading link = scene.ReadLink(i);
            residual = Max(residual, link.Residual);
            lowest = Min(lowest, link.Height);
            drift = Max(drift, link.Drift);
            rotation = Max(rotation, link.Rotation);
            kinetic += link.Kinetic;
            potential += link.Potential;
            int bit = 1 << i;
            if (link.Sleeping)
            {
                sleeping++;
                if ((sleepMask & bit) == 0)
                    sleeps++;
                sleepMask |= bit;
            }
            else if ((sleepMask & bit) != 0)
            {
                wakes++;
                sleepMask &= ~bit;
            }

            if (i > 0)
            {
                JointReading joint = scene.ReadJoint(i - 1);
                gap = Max(gap, joint.Gap);
                prepared = Max(prepared, joint.PreparedGap);
                limit = Max(limit, joint.Limit);
            }
            for (int j = i + 1; j <= Links; j++)
            {
                PairReading pair = scene.ReadPair(i, j, ids);
                if (pair.Support)
                    supports++;
                depth = Max(depth, pair.Depth);
                Span<ulong> active = ids[..pair.Count];
                active.Sort();
                contactIds.WriteInt32(pair.Count);
                foreach (ulong contactId in active)
                    contactIds.WriteUInt64(contactId);
            }
        }

        ChronicleHash set = contactIds.ToHash();
        var row = new Frame(depth, residual, supports, set, gap, prepared, limit, drift, rotation, sleeping, sleeps, wakes, kinetic, potential);
        result = result with
        {
            Final = row, PeakDepth = Max(result.PeakDepth, depth), PeakResidual = Max(result.PeakResidual, residual),
            LowestCom = frame == 0 ? lowest : Min(result.LowestCom, lowest), PeakGap = Max(result.PeakGap, gap),
            PeakPreparedGap = Max(result.PeakPreparedGap, prepared), PeakLimit = Max(result.PeakLimit, limit),
            PeakDrift = Max(result.PeakDrift, drift), PeakRotation = Max(result.PeakRotation, rotation),
            ContactChanges = result.ContactChanges + (frame > 0 && set != result.Final.ContactIds ? 1 : 0),
            Sleeps = result.Sleeps + sleeps, Wakes = result.Wakes + wakes
        };
        return row;
    }

    private static Scene CreatePlanar(GravitasWorldContext context, ReadOnlySpan<int> supportedX, int sleepSteps)
    {
        LSCollider2D floor = Constraint2DBenchmarks.CreateRagdoll(context, Vector2d.Zero,
            out SolidBody2D[] links, out Joint2D[] joints, sleepEnabled: true, angularLimit: Limit, supportedX);
        floor.Material = Material;
        var colliders = new LSCollider2D[] { floor }.Concat(links.Select(link => link.Collider)).ToArray();
        Vector2d[] starts = links.Select(link => link.Position).ToArray();
        foreach (SolidBody2D link in links)
        {
            link.Collider.Material = Material;
            link.Gravity = new Vector2d(Fixed64.Zero, -Gravity);
            link.UseManualGrounding();
            if (sleepSteps > 0)
                link.SleepFrameThreshold = sleepSteps;
            if (!supportedX.IsEmpty)
            {
                link.SleepLinearSpeedThreshold = RestSpeed;
                link.SleepAngularSpeedThreshold = RestSpeed;
            }
        }
        return new Scene(i =>
        {
            SolidBody2D link = links[i];
            Fixed64 speed = link.LinearVelocity.Magnitude, spin = link.AngularVelocity;
            var bounds = link.Collider.Bounds;
            Require(link.IsDynamic && link.Collider.IsPartitioned && bounds.Min.X > floor.Bounds.Min.X && bounds.Max.X < floor.Bounds.Max.X
                && link.Position.Magnitude < (Fixed64)8 && speed < (Fixed64)16 && spin.Abs() < (Fixed64)16,
                (int)context.FrameCount, "planar ragdoll left the finite measurement domain");
            Fixed64 height = link.Position.Y - floor.Bounds.Max.Y;
            return new LinkReading(height, Max(Fixed64.Zero, floor.Bounds.Max.Y - bounds.Min.Y),
                (link.Position.X - starts[i].X).Abs(), RadToDeg(Atan2(Sin(link.Rotation), Cos(link.Rotation)).Abs()),
                Fixed64.Half * (link.Mass * speed * speed + link.MomentOfInertia * spin * spin),
                Gravity * link.Mass * height, link.IsSleeping);
        }, i =>
        {
            Joint2D joint = joints[i];
            Vector2d a = joint.BodyA.Position + Vector2d.Rotate(joint.LocalFrameA.Anchor, joint.BodyA.Rotation);
            Vector2d b = joint.BodyB.Position + Vector2d.Rotate(joint.LocalFrameB.Anchor, joint.BodyB.Rotation);
            return new JointReading((b - a).Magnitude, joint.LastSolveMetrics.LinearAnchorErrorMagnitude,
                RadToDeg(joint.LastSolveMetrics.LimitErrorMagnitude));
        }, (i, j, ids) =>
        {
            LSCollider2D a = colliders[i], b = colliders[j];
            if ((!a.TryGetCollisionPair(b.Id, out CollisionPair2D pair) && !b.TryGetCollisionPair(a.Id, out pair))
                || !pair.IsColliding)
                return default;

            int count = pair.Manifold.Count;
            Fixed64 depth = Fixed64.Zero;
            bool support = false;
            for (int row = 0; row < count; row++)
            {
                ManifoldContact2D contact = pair.Manifold[row];
                Require(!contact.DepthIsClamped, (int)context.FrameCount, "planar ragdoll depth clamped");
                ids[row] = contact.ContactId;
                depth = Max(depth, contact.Depth);
                support |= i == 0 && (pair.ColliderB == b ? contact.Normal.Y : -contact.Normal.Y) > Fixed64.Half;
            }
            return new PairReading(count, depth, support);
        }, () => links[^1].AddAngularImpulse(SpinImpulse));
    }

    private static Scene CreateSpatial(GravitasWorldContext context, ReadOnlySpan<int> supportedX, int sleepSteps)
    {
        LSCollider floor = Constraint3DBenchmarks.CreateRagdoll(context, Vector3d.Zero,
            out SolidBody[] links, out Joint3D[] joints, sleepEnabled: true, angularLimit: Limit, supportedX);
        floor.Material = Material;
        var colliders = new LSCollider[] { floor }.Concat(links.Select(link => link.Collider)).ToArray();
        Vector3d[] starts = links.Select(link => link.Position3d).ToArray();
        Fixed3x3[] inertia = links.Select(link => link.Collider.CalculateInertiaTensor(link.Mass, link.LocalCenterOfMassOffset)).ToArray();
        foreach (SolidBody link in links)
        {
            link.Collider.Material = Material;
            link.UseManualGrounding();
            if (sleepSteps > 0)
                link.SleepFrameThreshold = sleepSteps;
            if (!supportedX.IsEmpty)
            {
                link.SleepLinearSpeedThreshold = RestSpeed;
                link.SleepAngularSpeedThreshold = RestSpeed;
            }
        }
        return new Scene(i =>
        {
            SolidBody link = links[i];
            Fixed64 speed = link.LinearVelocity.Magnitude;
            Require(link.IsDynamic && link.Collider.IsPartitioned && link.Collider.BoundsMin.X > floor.BoundsMin.X && link.Collider.BoundsMax.X < floor.BoundsMax.X
                && link.Collider.BoundsMin.Z > floor.BoundsMin.Z && link.Collider.BoundsMax.Z < floor.BoundsMax.Z
                && link.Position3d.Magnitude < (Fixed64)8 && speed < (Fixed64)16 && link.AngularVelocity.Magnitude < (Fixed64)16,
                (int)context.FrameCount, "spatial ragdoll left the finite measurement domain");
            Fixed64 height = link.Position3d.Y - floor.BoundsMax.Y;
            Vector3d spin = link.Rotation.Conjugate().Rotate(link.AngularVelocity);
            Vector3d drift = link.Position3d - starts[i];
            return new LinkReading(height, Max(Fixed64.Zero, floor.BoundsMax.Y - link.Collider.BoundsMin.Y),
                new Vector2d(drift.X, drift.Z).Magnitude, FixedQuaternion.Angle(FixedQuaternion.Identity, link.Rotation),
                Fixed64.Half * (link.Mass * speed * speed + Vector3d.Dot(spin, inertia[i] * spin)),
                Gravity * link.Mass * height, link.IsSleeping);
        }, i =>
        {
            Joint3D joint = joints[i];
            Vector3d a = joint.BodyA.Position3d + joint.BodyA.Rotation.Rotate(joint.LocalFrameA.Position);
            Vector3d b = joint.BodyB.Position3d + joint.BodyB.Rotation.Rotate(joint.LocalFrameB.Position);
            return new JointReading((b - a).Magnitude, joint.LastSolveMetrics.LinearAnchorErrorMagnitude,
                RadToDeg(joint.LastSolveMetrics.AngularLimitErrorMagnitude));
        }, (i, j, ids) =>
        {
            LSCollider a = colliders[i], b = colliders[j];
            if ((!a.TryGetCollisionPair(b.Id, out CollisionPair pair) && !b.TryGetCollisionPair(a.Id, out pair)) || !pair.Active)
                return default;

            int count = 0;
            Fixed64 depth = Fixed64.Zero;
            bool support = false;
            foreach (ManifoldContact contact in pair.Manifold)
            {
                Require(!contact.DepthIsClamped && count < ids.Length, (int)context.FrameCount, "unexpected spatial ragdoll contact row");
                ids[count++] = contact.ContactId;
                depth = Max(depth, contact.Depth);
                support |= i == 0 && (pair.ColliderB == b ? contact.Normal.Y : -contact.Normal.Y) > Fixed64.Half;
            }
            return new PairReading(count, depth, support);
        }, () => links[^1].AddAngularImpulse(Vector3d.Forward * SpinImpulse));
    }

    private delegate PairReading ReadPair(int first, int second, Span<ulong> ids);
    private sealed record Scene(Func<int, LinkReading> ReadLink, Func<int, JointReading> ReadJoint, ReadPair ReadPair, Action Kick);
    private readonly record struct LinkReading(Fixed64 Height, Fixed64 Residual, Fixed64 Drift, Fixed64 Rotation,
        Fixed64 Kinetic, Fixed64 Potential, bool Sleeping);
    private readonly record struct JointReading(Fixed64 Gap, Fixed64 PreparedGap, Fixed64 Limit);
    private readonly record struct PairReading(int Count, Fixed64 Depth, bool Support);
    private readonly record struct Frame(Fixed64 Depth, Fixed64 Residual, int Supports, ChronicleHash ContactIds,
        Fixed64 Gap, Fixed64 PreparedGap, Fixed64 Limit, Fixed64 Drift, Fixed64 Rotation, int Sleeping, int Sleeps, int Wakes,
        Fixed64 Kinetic, Fixed64 Potential)
    {
        internal void Hash(ref ChronicleHashWriter writer)
        {
            writer.WriteFixed64(Depth);
            writer.WriteFixed64(Residual);
            writer.WriteInt32(Supports);
            writer.WriteUInt64(ContactIds.Low);
            writer.WriteUInt64(ContactIds.High);
            writer.WriteFixed64(Gap);
            writer.WriteFixed64(PreparedGap);
            writer.WriteFixed64(Limit);
            writer.WriteFixed64(Drift);
            writer.WriteFixed64(Rotation);
            writer.WriteInt32(Sleeping);
            writer.WriteInt32(Sleeps);
            writer.WriteInt32(Wakes);
            writer.WriteFixed64(Kinetic);
            writer.WriteFixed64(Potential);
        }
    }
    private readonly record struct Recovery(int AsleepBefore, int AwakeAfter, int RestFrame, Fixed64 Work);

    private readonly record struct Result(ChronicleHash Trace, ChronicleHash Terminal, Frame Final, Fixed64 PeakDepth,
        Fixed64 PeakResidual, Fixed64 LowestCom, Fixed64 PeakGap, Fixed64 PeakPreparedGap, Fixed64 PeakLimit, Fixed64 PeakDrift,
        Fixed64 PeakRotation, Fixed64 GravityWork, Fixed64 KickWork, int FirstSupport, int SupportFrames, int ContactChanges,
        int Sleeps, int Wakes, int AsleepBeforeKick, int AwakeAfterKick, long WarmBytes, long PeakBytes, int PeakBytesFrame,
        int FirstRest, int RestFrames, int LongestRest, int FinalRest, Recovery Recovery1, Recovery Recovery2, Recovery Recovery3);
}
