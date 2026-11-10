using System;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Constraints;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class ConstraintChainWorkload
{
    private const int Links = 32;
    private const int Frames = 256;
    private const int Warmup = 64;
    private static readonly Vector2d UnitImpulse = new(Fixed64.FromFraction(3, 16), Fixed64.FromFraction(1, 16));

    internal static int Verify()
    {
        bool csv = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        if (csv)
            Console.WriteLine("fixture,frame,anchor_gap,prepared_anchor_gap,tip_drift,peak_rotation_degrees,linear_energy,angular_energy,prepared_rows");

        (string Name, int TipMass, Vector2d Impulse)[] cases =
        [
            ("rest", 1, Vector2d.Zero),
            ("1", 1, UnitImpulse),
            ("10", 10, UnitImpulse * (Fixed64)10),
            ("100", 100, UnitImpulse * (Fixed64)100),
            ("100-matched-impulse", 100, UnitImpulse)
        ];
        foreach (bool planar in new[] { true, false })
        foreach (var (name, mass, impulse) in cases)
        {
            string id = $"quality-{(planar ? "2d" : "3d")}-chain-{name}-v1";
            RunResult first = Run(planar, mass, impulse, id, observe: true, csv);
            RunResult repeat = Run(planar, mass, impulse, id, observe: true, csv: false);
            RunResult control = Run(planar, mass, impulse, id, observe: false, csv: false);
            Require(first == repeat && first.Terminal == control.Terminal, Frames, $"{id} replay diverged");
            bool withinBounds = impulse == Vector2d.Zero
                ? first.PeakGap == Fixed64.Zero && first.PeakDrift == Fixed64.Zero && first.PeakRotation == Fixed64.Zero
                    && first.PeakEnergy == Fixed64.Zero
                : first.PeakGap <= Fixed64.FromFraction(1, 16) && first.Final.Gap <= Fixed64.FromFraction(1, 64)
                    && first.PeakDrift <= Fixed64.One && first.PeakEnergy <= first.InputEnergy * 2;
            Console.WriteLine(FormattableString.Invariant(
                $"{id} trace {first.Trace}, terminal {first.Terminal}, peak/final gap {first.PeakGap:F6}/{first.Final.Gap:F6}, peak prepared gap {first.PeakPreparedGap:F6}, peak/final tip drift {first.PeakDrift:F6}/{first.Final.TipDrift:F6}, peak rotation {first.PeakRotation:F4} deg, impulse/peak/final energy {first.InputEnergy:F6}/{first.PeakEnergy:F6}/{first.Final.Energy:F6}, prepared rows {first.PreparedRows}, warm bytes {control.WarmBytes}, peak {control.PeakBytes} on frame {control.PeakBytesFrame}, bounds {(withinBounds ? "within" : "outside")}"));
        }

        return 0;
    }

    private static RunResult Run(bool planar, int mass, Vector2d impulse, string id, bool observe, bool csv)
    {
        const int gridExtent = Links + 4;
        using GravitasWorldContext context = planar
            ? Constraint2DBenchmarks.CreateConstraintContext(gridExtent, Fixed64.Zero, PhysicsSettings.DefaultDiscreteSolverIterations)
            : Constraint3DBenchmarks.CreateConstraintContext(gridExtent, Fixed64.Zero, PhysicsSettings.DefaultDiscreteSolverIterations);
        context.SetFrameRate(60);
        Scene scene = planar ? CreatePlanar(context, mass, impulse) : CreateSpatial(context, mass, impulse);
        var trace = new ChronicleHashWriter();
        trace.WriteSection(id, 1);
        Sample sample = default;
        Fixed64 peakGap = Fixed64.Zero, peakPreparedGap = Fixed64.Zero, peakDrift = Fixed64.Zero;
        Fixed64 peakRotation = Fixed64.Zero, peakEnergy = Fixed64.Zero, inputEnergy = Fixed64.Zero;
        int preparedRows = 0, peakBytesFrame = -1;
        long warmBytes = 0, peakBytes = 0;
        for (int frame = 0; frame <= Frames; frame++)
        {
            if (frame > 0)
            {
                bool measure = !observe && frame > Warmup;
                long startBytes = measure ? GC.GetAllocatedBytesForCurrentThread() : 0;
                context.Simulate();
                context.LateSimulate();
                if (measure)
                {
                    long bytes = GC.GetAllocatedBytesForCurrentThread() - startBytes;
                    warmBytes += bytes;
                    if (bytes > peakBytes)
                    {
                        peakBytes = bytes;
                        peakBytesFrame = frame;
                    }
                }
            }

            if (!observe)
                continue;

            sample = Read(scene, planar ? 2 : 3, id, frame);
            if (frame == 0)
            {
                inputEnergy = sample.Energy;
                Fixed64 expectedEnergy = Fixed64.Half * (impulse.MagnitudeSquared / mass);
                // Applying the impulse and measuring kinetic energy round separately.
                Require(sample.Gap == Fixed64.Zero && sample.TipDrift == Fixed64.Zero
                    && (inputEnergy - expectedEnergy).Abs() <= Fixed64.FromRaw(4096), frame,
                    $"{id} must start aligned with only the commanded tip motion");
            }

            peakGap = FixedMath.Max(peakGap, sample.Gap);
            peakPreparedGap = FixedMath.Max(peakPreparedGap, sample.PreparedGap);
            peakDrift = FixedMath.Max(peakDrift, sample.TipDrift);
            peakRotation = FixedMath.Max(peakRotation, sample.RotationDegrees);
            peakEnergy = FixedMath.Max(peakEnergy, sample.Energy);
            preparedRows += sample.Rows;
            ChronicleHash hash = context.ComputeReplayHash();
            trace.WriteUInt64(hash.Low);
            trace.WriteUInt64(hash.High);
            trace.WriteFixed64(sample.Gap);
            trace.WriteFixed64(sample.PreparedGap);
            trace.WriteFixed64(sample.TipDrift);
            trace.WriteFixed64(sample.RotationDegrees);
            trace.WriteFixed64(sample.LinearEnergy);
            trace.WriteFixed64(sample.AngularEnergy);
            trace.WriteInt32(sample.Rows);
            if (csv)
                Console.WriteLine(FormattableString.Invariant(
                    $"{id},{frame},{sample.Gap:F6},{sample.PreparedGap:F6},{sample.TipDrift:F6},{sample.RotationDegrees:F4},{sample.LinearEnergy:F6},{sample.AngularEnergy:F6},{sample.Rows}"));
        }

        return new RunResult(trace.ToHash(), context.ComputeReplayHash(), sample, inputEnergy,
            peakGap, peakPreparedGap, peakDrift, peakRotation, peakEnergy, preparedRows, warmBytes, peakBytes, peakBytesFrame);
    }

    private static Sample Read(Scene scene, int rowsPerJoint, string id, int frame)
    {
        Fixed64 gap = Fixed64.Zero, preparedGap = Fixed64.Zero, tipDrift = Fixed64.Zero;
        Fixed64 rotation = Fixed64.Zero, linear = Fixed64.Zero, angular = Fixed64.Zero;
        int rows = 0;
        for (int i = 0; i < Links; i++)
        {
            BodySample body = scene.ReadBody(i);
            if (!body.Partitioned || body.Touching || body.Center.Magnitude >= (Fixed64)64
                || body.Speed >= (Fixed64)8 || body.AngularEnergy >= (Fixed64)1024)
                throw new InvalidOperationException($"{id} left the contact-free chain sample domain on frame {frame}, link {i}");
            if (i == Links - 1)
                tipDrift = (body.Center - Vector3d.Right * i).Magnitude;
            rotation = FixedMath.Max(rotation, body.RotationDegrees);
            linear += KineticEnergy(body.Mass, body.Speed * body.Speed);
            angular += body.AngularEnergy;
            if (i == Links - 1)
                continue;

            JointSample joint = scene.ReadJoint(i);
            if (frame > 0 && joint.Rows != rowsPerJoint)
                throw new InvalidOperationException($"{id} prepared {joint.Rows} of {rowsPerJoint} rows on frame {frame}, joint {i}");
            gap = FixedMath.Max(gap, joint.Gap);
            preparedGap = FixedMath.Max(preparedGap, joint.PreparedGap);
            rows += joint.Rows;
        }

        return new Sample(gap, preparedGap, tipDrift, rotation, linear, angular, rows);
    }

    private static Scene CreatePlanar(GravitasWorldContext context, int mass, Vector2d impulse)
    {
        Constraint2DBenchmarks.CreatePinLine(context, Links, out SolidBody2D[] bodies, out Joint2D[] joints,
            seedForce: false, spacing: 1);
        foreach (SolidBody2D body in bodies)
            body.UseManualGrounding();
        bodies[^1].Mass = (Fixed64)mass;
        bodies[^1].AddLinearImpulse(impulse);

        return new Scene(i =>
        {
            SolidBody2D body = bodies[i];
            Require(body.AngularVelocity.Abs() < (Fixed64)8, (int)context.FrameCount, "Planar chain spin exceeded the sample domain.");
            bool touching = false;
            if (body.Collider.CollisionPairs is { } pairs)
                foreach (var entry in pairs)
                    touching |= entry.Value.IsColliding;
            return new BodySample(new Vector3d(body.Position.X, body.Position.Y, Fixed64.Zero),
                body.Mass, body.LinearVelocity.Magnitude,
                KineticEnergy(body.MomentOfInertia, body.AngularVelocity * body.AngularVelocity),
                FixedMath.RadToDeg(FixedMath.Atan2(FixedMath.Sin(body.Rotation), FixedMath.Cos(body.Rotation)).Abs()),
                body.Collider.IsPartitioned, touching);
        }, i =>
        {
            Joint2D joint = joints[i];
            Vector2d a = joint.BodyA.Position + Vector2d.Rotate(joint.LocalFrameA.Anchor, joint.BodyA.Rotation);
            Vector2d b = joint.BodyB.Position + Vector2d.Rotate(joint.LocalFrameB.Anchor, joint.BodyB.Rotation);
            return new JointSample((b - a).Magnitude, joint.LastSolveMetrics.LinearAnchorErrorMagnitude,
                joint.LastSolveMetrics.PreparedRowCount);
        });
    }

    private static Scene CreateSpatial(GravitasWorldContext context, int mass, Vector2d impulse)
    {
        Constraint3DBenchmarks.CreateBallSocketLine(context, Links, out SolidBody[] bodies, out Joint3D[] joints,
            seedImpulse: false, spacing: 1);
        foreach (SolidBody body in bodies)
            body.UseManualGrounding();
        bodies[^1].Mass = (Fixed64)mass;
        bodies[^1].AddLinearImpulse(new Vector3d(impulse.X, impulse.Y, Fixed64.Zero));

        var inertia = new Fixed3x3[Links];
        for (int i = 0; i < inertia.Length; i++)
            inertia[i] = bodies[i].Collider.CalculateInertiaTensor(bodies[i].Mass, bodies[i].LocalCenterOfMassOffset);

        return new Scene(i =>
        {
            SolidBody body = bodies[i];
            Require(body.AngularVelocity.Magnitude < (Fixed64)8, (int)context.FrameCount, "Spatial chain spin exceeded the sample domain.");
            Vector3d localSpin = body.Rotation.Conjugate().Rotate(body.AngularVelocity);
            bool touching = false;
            if (body.Collider.CollisionPairs is { } pairs)
                foreach (var entry in pairs)
                    touching |= entry.Value.Manifold.HasContact;
            return new BodySample(body.Position3d, body.Mass, body.LinearVelocity.Magnitude,
                Fixed64.Half * Vector3d.Dot(localSpin, inertia[i] * localSpin),
                FixedQuaternion.Angle(FixedQuaternion.Identity, body.Rotation), body.Collider.IsPartitioned, touching);
        }, i =>
        {
            Joint3D joint = joints[i];
            Vector3d a = joint.BodyA.Position3d + joint.BodyA.Rotation.Rotate(joint.LocalFrameA.Position);
            Vector3d b = joint.BodyB.Position3d + joint.BodyB.Rotation.Rotate(joint.LocalFrameB.Position);
            return new JointSample((b - a).Magnitude, joint.LastSolveMetrics.LinearAnchorErrorMagnitude,
                joint.LastSolveMetrics.PreparedRowCount);
        });
    }

    private static Fixed64 KineticEnergy(Fixed64 mass, Fixed64 speedSquared) => Fixed64.Half * mass * speedSquared;

    private sealed record Scene(Func<int, BodySample> ReadBody, Func<int, JointSample> ReadJoint);

    private readonly record struct BodySample(Vector3d Center, Fixed64 Mass, Fixed64 Speed,
        Fixed64 AngularEnergy, Fixed64 RotationDegrees, bool Partitioned, bool Touching);

    private readonly record struct JointSample(Fixed64 Gap, Fixed64 PreparedGap, int Rows);

    private readonly record struct Sample(Fixed64 Gap, Fixed64 PreparedGap, Fixed64 TipDrift, Fixed64 RotationDegrees,
        Fixed64 LinearEnergy, Fixed64 AngularEnergy, int Rows)
    {
        internal Fixed64 Energy => LinearEnergy + AngularEnergy;
    }

    private readonly record struct RunResult(ChronicleHash Trace, ChronicleHash Terminal, Sample Final, Fixed64 InputEnergy,
        Fixed64 PeakGap, Fixed64 PeakPreparedGap, Fixed64 PeakDrift, Fixed64 PeakRotation, Fixed64 PeakEnergy,
        int PreparedRows, long WarmBytes, long PeakBytes, int PeakBytesFrame);
}
