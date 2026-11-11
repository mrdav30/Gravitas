using System;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Materials;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class QualityWorkEnergyWorkload
{
    private const int Frames = 120;
    private static readonly Fixed64 Mass = Fixed64.Two;
    private static readonly Fixed64 InitialHeight = Fixed64.Two;
    private static readonly Fixed64 AngleBound = Fixed64.Half;
    private static readonly Fixed64 InitialLimitAngle = Fixed64.FromFraction(3, 4);
    private static readonly Fixed64 MotorImpulseCap = Fixed64.FromFraction(1, 64);
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 4096);

    internal static int Verify()
    {
        bool csv = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        if (csv)
            Console.WriteLine("fixture,frame,kinetic,potential,gravity_work,impulse_work,motor_work,angle,limit_error,prepared_limit_error");

        foreach (bool planar in new[] { true, false })
        {
            string dimension = planar ? "2d" : "3d";
            foreach (bool falling in new[] { false, true })
            {
                string id = $"quality-{dimension}-{(falling ? "gravity" : "coast")}-v1";
                TranslationResult result = RunTranslation(planar, falling, id, csv);
                Require(result == RunTranslation(planar, falling, id, csv: false), Frames, $"{id} replay diverged");
                Console.WriteLine(FormattableString.Invariant(
                    $"{id} trace {result.Trace}, K {result.Kinetic:F6}, U {result.Potential:F6}, impulse work {result.ImpulseWork:F6}, gravity work {result.GravityWork:F6}, peak budget residual {result.PeakResidual:F6}, integrator energy change {result.IntegratorEnergyChange:F6}"));
            }

            foreach (bool motor in new[] { true, false })
            {
                string id = $"quality-{dimension}-{(motor ? "motor-work" : "angular-limit")}-v1";
                RotationResult result = RunRotation(planar, motor, id, csv);
                Require(result == RunRotation(planar, motor, id, csv: false), Frames, $"{id} replay diverged");
                Console.WriteLine(FormattableString.Invariant(
                    $"{id} trace {result.Trace}, K {result.Kinetic:F6}, motor work +{result.PositiveWork:F6}/{result.NegativeWork:F6}, peak budget residual {result.PeakResidual:F6}, final angle {result.FinalAngle:F6} rad, peak/final limit error {result.PeakLimitError:F6}/{result.FinalLimitError:F6} rad, peak prepared limit error {result.PeakPreparedError:F6} rad"));
            }
        }

        return 0;
    }

    private static TranslationResult RunTranslation(bool planar, bool falling, string id, bool csv)
    {
        Fixed64 gravity = falling ? Fixed64.One : Fixed64.Zero;
        using GravitasWorldContext context = CreateContext(planar, gravity, PhysicsSettings.DefaultDiscreteSolverIterations);
        Func<Translation> read;
        if (planar)
        {
            SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, InitialHeight), Fixed64.Zero);
            body.Gravity = new Vector2d(Fixed64.Zero, -gravity);
            body.AddLinearImpulse(Vector2d.Right);
            read = () => new Translation(InPlane(body.WorldCenterOfMass), InPlane(body.LinearVelocity),
                body.AngularVelocity.Abs(), body.Collider.IsPartitioned, HasContact(body));
        }
        else
        {
            SolidBody body = CreateSphere(context, Vector3d.Up * InitialHeight, Fixed64.Zero);
            body.AddLinearImpulse(Vector3d.Right);
            read = () => new Translation(body.WorldCenterOfMass, body.LinearVelocity, body.AngularVelocity.Magnitude,
                body.Collider.IsPartitioned, HasContact(body));
        }

        var trace = new ChronicleHashWriter();
        trace.WriteSection(id, 1);
        Fixed64 impulseWork = Fixed64.Half / Mass;
        Fixed64 kinetic = Fixed64.Zero, potential = Fixed64.Zero, gravityWork = Fixed64.Zero;
        Fixed64 peakResidual = Fixed64.Zero, integratorChange = Fixed64.Zero;
        Fixed64 deltaTime = context.DeltaTime;
        Fixed64 velocityStep = gravity * deltaTime;
        for (int frame = 0; frame <= Frames; frame++)
        {
            if (frame > 0)
                Step(context);
            Translation sample = read();
            Require(sample.Partitioned && !sample.Touching && sample.Position.Magnitude < (Fixed64)4
                && sample.Velocity.Magnitude < (Fixed64)4 && sample.Spin == Fixed64.Zero, frame, $"{id} left the free-motion domain");
            Fixed64 expectedY = InitialHeight - velocityStep * deltaTime * (frame * (frame + 1) / 2);
            Require((sample.Position.Y - expectedY).Abs() <= Tolerance
                && (sample.Velocity.Y + velocityStep * frame).Abs() <= Tolerance
                && (sample.Position.X - Fixed64.Half * deltaTime * frame).Abs() <= Tolerance
                && sample.Velocity.X == Fixed64.Half && sample.Position.Z == Fixed64.Zero
                && sample.Velocity.Z == Fixed64.Zero, frame, $"{id} free motion missed the fixed-step trajectory");

            kinetic = Fixed64.Half * Mass * sample.Velocity.MagnitudeSquared;
            potential = Mass * gravity * sample.Position.Y;
            gravityWork = Mass * gravity * (InitialHeight - sample.Position.Y);
            // Semi-implicit Euler uses the updated velocity for displacement.
            integratorChange = -Fixed64.Half * Mass * velocityStep * velocityStep * frame;
            Fixed64 residual = (kinetic - impulseWork - gravityWork - integratorChange).Abs();
            Require(falling ? residual <= Tolerance : residual == Fixed64.Zero, frame, $"{id} energy budget missed its bound");
            peakResidual = FixedMath.Max(peakResidual, residual);
            Record(ref trace, context, new WorkSample(kinetic, potential, gravityWork, impulseWork), csv ? id : null, frame);
        }

        Require(!falling || gravityWork > Fixed64.One, Frames, $"{id} did not exercise gravity work");
        return new TranslationResult(trace.ToHash(), kinetic, potential, impulseWork, gravityWork, peakResidual, integratorChange);
    }

    private static RotationResult RunRotation(bool planar, bool motor, string id, bool csv)
    {
        // One solver pass makes the accumulated row impulse the net impulse of this step.
        using GravitasWorldContext context = CreateContext(planar, Fixed64.Zero, iterations: 1);
        Fixed64 initialAngle = motor ? Fixed64.Zero : InitialLimitAngle;
        (Fixed64 inertia, Func<Rotor> read) = planar
            ? CreatePlanarRotor(context, motor, initialAngle)
            : CreateSpatialRotor(context, motor, initialAngle);
        var trace = new ChronicleHashWriter();
        trace.WriteSection(id, 1);
        Fixed64 priorSpin = Fixed64.Zero, positive = Fixed64.Zero, negative = Fixed64.Zero;
        Fixed64 kinetic = Fixed64.Zero, peakResidual = Fixed64.Zero, peakLimit = Fixed64.Zero, peakPrepared = Fixed64.Zero;
        Fixed64 angle = Fixed64.Zero, limitError = Fixed64.Zero;
        Fixed64 correctionSpeed = (InitialLimitAngle - AngleBound) * Fixed64.FromFraction(2, 5) * context.InvDeltaTime;
        for (int frame = 0; frame <= Frames; frame++)
        {
            if (frame > 0)
                Step(context);
            Rotor sample = read();
            angle = sample.Angle;
            bool spinWithinDomain = motor ? sample.Spin.Abs() < (Fixed64)4
                : sample.Spin.Abs() <= correctionSpeed + Tolerance;
            Require(sample.Centered && spinWithinDomain && angle.Abs() < Fixed64.One,
                frame, $"{id} left its contact-free centered rotor domain");
            kinetic = Fixed64.Half * inertia * sample.Spin * sample.Spin;
            if (motor && frame > 0)
            {
                Fixed64 momentumChange = inertia * (sample.Spin - priorSpin);
                Require((sample.ImpulseMagnitude - momentumChange.Abs()).Abs() <= Tolerance
                    && sample.ImpulseMagnitude <= MotorImpulseCap, frame, $"{id} motor impulse disagreed with rotor momentum");
                // Centered anchors exert no torque. The signed motor impulse is the sole angular work input.
                Fixed64 signedImpulse = momentumChange < Fixed64.Zero ? -sample.ImpulseMagnitude : sample.ImpulseMagnitude;
                Fixed64 work = signedImpulse * Fixed64.Half * (priorSpin + sample.Spin);
                if (work > Fixed64.Zero)
                    positive += work;
                else
                    negative += work;

                Fixed64 residual = (kinetic - positive - negative).Abs();
                Require(residual <= Tolerance, frame, $"{id} kinetic energy disagreed with cumulative signed motor work");
                peakResidual = FixedMath.Max(peakResidual, residual);
                if (frame == 1)
                    Require(sample.Spin > Fixed64.Zero && sample.ImpulseMagnitude == MotorImpulseCap,
                        frame, $"{id} did not accelerate with the authored motor cap");
            }

            priorSpin = sample.Spin;
            limitError = motor ? Fixed64.Zero : FixedMath.Max(Fixed64.Zero, angle.Abs() - AngleBound);
            peakLimit = FixedMath.Max(peakLimit, limitError);
            peakPrepared = FixedMath.Max(peakPrepared, sample.PreparedLimitError);
            if (!motor && frame == 1)
                Require((sample.Spin + correctionSpeed).Abs() <= Tolerance
                    && (sample.PreparedLimitError - (InitialLimitAngle - AngleBound)).Abs() <= Tolerance,
                    frame, $"{id} did not prepare the authored angular limit violation");

            Record(ref trace, context, new WorkSample(kinetic, MotorWork: positive + negative, Angle: angle,
                LimitError: limitError, PreparedLimitError: sample.PreparedLimitError), csv ? id : null, frame);
        }

        Require(motor
            ? positive > Fixed64.Zero && negative < Fixed64.Zero && (angle - AngleBound).Abs() <= Fixed64.FromFraction(1, 16)
            : peakLimit <= Fixed64.Quarter + Tolerance && limitError <= Fixed64.FromFraction(3, 16),
            Frames, $"{id} missed its target or limit response bound");
        return new RotationResult(trace.ToHash(), kinetic, positive, negative, peakResidual, angle, peakLimit, limitError, peakPrepared);
    }

    private static (Fixed64, Func<Rotor>) CreatePlanarRotor(GravitasWorldContext context, bool motor, Fixed64 angle)
    {
        SolidBody2D anchor = CreateCircle(context, Vector2d.Zero, Fixed64.Zero, PhysicsMobilityType.Static);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, angle);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor, body, JointFrame2D.Identity, JointFrame2D.Identity, JointType2D.Pin,
            motor ? JointLimit2D.Unrestricted : JointLimit2D.Angular(-AngleBound, AngleBound),
            motor ? JointMotor2D.Angular(AngleBound, (Fixed64)4, Fixed64.Half, MotorImpulseCap) : JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        return (body.MomentOfInertia, () => new Rotor(body.Rotation, body.AngularVelocity,
            joint.LastSolveMetrics.AccumulatedImpulseMagnitude, joint.LastSolveMetrics.LimitErrorMagnitude,
            body.Collider.IsPartitioned && !HasContact(body) && !HasContact(anchor)
            && body.WorldCenterOfMass == Vector2d.Zero && body.LinearVelocity == Vector2d.Zero));
    }

    private static (Fixed64, Func<Rotor>) CreateSpatialRotor(GravitasWorldContext context, bool motor, Fixed64 angle)
    {
        SolidBody anchor = CreateSphere(context, Vector3d.Zero, Fixed64.Zero, PhysicsMobilityType.Static);
        SolidBody body = CreateSphere(context, Vector3d.Zero, angle);
        var centerFrame = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Joint3D joint = context.Constraints3D.RegisterJoint(new JointDefinition3D(
            anchor, body, centerFrame, centerFrame, JointType3D.Hinge,
            motor ? JointLimit3D.Unrestricted : JointLimit3D.Hinge(AngleBound),
            motor ? new JointMotor3D(FixedQuaternion.FromAxisAngle(Vector3d.Right, AngleBound), (Fixed64)4, Fixed64.Half, MotorImpulseCap) : JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Vector3d inertiaAlongAxis = body.Collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset) * Vector3d.Right;
        return (inertiaAlongAxis.X, () => new Rotor(Fixed64.Two * FixedMath.Atan2(body.Rotation.X, body.Rotation.W), body.AngularVelocity.X,
            joint.LastSolveMetrics.AccumulatedImpulseMagnitude, joint.LastSolveMetrics.AngularLimitErrorMagnitude,
            body.Collider.IsPartitioned && !HasContact(body) && !HasContact(anchor)
            && body.WorldCenterOfMass == Vector3d.Zero && body.LinearVelocity == Vector3d.Zero
            && body.AngularVelocity.Y == Fixed64.Zero && body.AngularVelocity.Z == Fixed64.Zero));
    }

    private static GravitasWorldContext CreateContext(bool planar, Fixed64 gravity, int iterations)
    {
        GravitasWorldContext context = planar
            ? Constraint2DBenchmarks.CreateConstraintContext(8, Fixed64.Zero, iterations)
            : Constraint3DBenchmarks.CreateConstraintContext(8, gravity, iterations);
        context.SetFrameRate(60);
        return context;
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position, Fixed64 angle,
        PhysicsMobilityType mobility = PhysicsMobilityType.Dynamic)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero)), collider)
            { Mass = Mass, SleepEnabled = false };
        body.UseManualGrounding();
        body.Initialize(position, angle, mobility);
        return body;
    }

    private static SolidBody CreateSphere(GravitasWorldContext context, Vector3d position, Fixed64 angle,
        PhysicsMobilityType mobility = PhysicsMobilityType.Dynamic)
    {
        var collider = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody(new BenchmarkMatterAgent(context, position), collider) { Mass = Mass, SleepEnabled = false };
        body.UseManualGrounding();
        body.Initialize(position, FixedQuaternion.FromAxisAngle(Vector3d.Right, angle), mobility);
        return body;
    }

    private static bool HasContact(SolidBody2D body)
    {
        if (body.Collider.CollisionPairs is { } pairs)
            foreach (var entry in pairs)
                if (entry.Value.IsColliding)
                    return true;
        return false;
    }

    private static bool HasContact(SolidBody body)
    {
        if (body.Collider.CollisionPairs is { } pairs)
            foreach (var entry in pairs)
                if (entry.Value.Manifold.HasContact)
                    return true;
        return false;
    }

    private static Vector3d InPlane(Vector2d vector) => new(vector.X, vector.Y, Fixed64.Zero);

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static void Record(ref ChronicleHashWriter trace, GravitasWorldContext context, WorkSample sample, string csvId, int frame)
    {
        ChronicleHash state = context.ComputeReplayHash();
        trace.WriteUInt64(state.Low);
        trace.WriteUInt64(state.High);
        trace.WriteFixed64(sample.Kinetic);
        trace.WriteFixed64(sample.Potential);
        trace.WriteFixed64(sample.GravityWork);
        trace.WriteFixed64(sample.ImpulseWork);
        trace.WriteFixed64(sample.MotorWork);
        trace.WriteFixed64(sample.Angle);
        trace.WriteFixed64(sample.LimitError);
        trace.WriteFixed64(sample.PreparedLimitError);
        if (csvId != null)
            Console.WriteLine(FormattableString.Invariant($"{csvId},{frame},{sample.Kinetic:F9},{sample.Potential:F9},{sample.GravityWork:F9},{sample.ImpulseWork:F9},{sample.MotorWork:F9},{sample.Angle:F9},{sample.LimitError:F9},{sample.PreparedLimitError:F9}"));
    }

    private readonly record struct Translation(Vector3d Position, Vector3d Velocity, Fixed64 Spin, bool Partitioned, bool Touching);
    private readonly record struct Rotor(Fixed64 Angle, Fixed64 Spin, Fixed64 ImpulseMagnitude, Fixed64 PreparedLimitError, bool Centered);
    private readonly record struct WorkSample(Fixed64 Kinetic, Fixed64 Potential = default, Fixed64 GravityWork = default,
        Fixed64 ImpulseWork = default, Fixed64 MotorWork = default, Fixed64 Angle = default, Fixed64 LimitError = default,
        Fixed64 PreparedLimitError = default);
    private readonly record struct TranslationResult(ChronicleHash Trace, Fixed64 Kinetic, Fixed64 Potential, Fixed64 ImpulseWork,
        Fixed64 GravityWork, Fixed64 PeakResidual, Fixed64 IntegratorEnergyChange);
    private readonly record struct RotationResult(ChronicleHash Trace, Fixed64 Kinetic, Fixed64 PositiveWork, Fixed64 NegativeWork,
        Fixed64 PeakResidual, Fixed64 FinalAngle, Fixed64 PeakLimitError, Fixed64 FinalLimitError, Fixed64 PeakPreparedError);
}
