using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Diagnostics;
using Gravitas.Tests.Determinism;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Constraints;

public sealed class Constraint3DStressTests
{
    [Fact]
    public void LongBallSocketChain_ShouldRemainBoundedAndDeterministic()
    {
        ReplayConformanceHarness.AssertRepeatedRunsMatch(
            () => CreateLongBallSocketChain(out _, out _),
            frameCount: 32,
            beforeFrame: static (context, _) => context.Simulate(),
            mode: GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        using GravitasWorldContext context = CreateLongBallSocketChain(out SolidBody[] bodies, out Joint3D[] joints);
        Fixed64 initialMaxAnchorError = MaxAnchorError(joints);
        Step(context, 32);

        MaxAnchorError(joints).Should().BeLessThan(initialMaxAnchorError);
        MaxPositionMagnitude(bodies).Should().BeLessThan((Fixed64)64);
        SumSolvedRows(joints).Should().BeGreaterThan(0);
        SumLinearAnchorError(joints).Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void HingeChain_WithAlternatingAxes_ShouldStayDeterministicAndReportAngularRows()
    {
        ReplayConformanceHarness.AssertRepeatedRunsMatch(
            () => CreateAlternatingHingeChain(out _, out _),
            frameCount: 32,
            beforeFrame: static (context, _) => context.Simulate(),
            mode: GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        using GravitasWorldContext context = CreateAlternatingHingeChain(out _, out Joint3D[] joints);
        Step(context, 32);

        SumSolvedRows(joints).Should().BeGreaterThan(0);
        MaxAnchorError(joints).Should().BeLessThan((Fixed64)2);
    }

    [Fact]
    public void HumanoidRagdollRestingOnGeometry_ShouldRemainBoundedAndWakeAsOneIsland()
    {
        using GravitasWorldContext context = CreateHumanoidRagdollOnFloor(out SolidBody[] bodies, out Joint3D[] joints);
        bodies[0].Sleep();
        bodies[^1].AddLinearImpulse(Vector3d.Right * (Fixed64)12);

        context.Simulate();
        context.LateSimulate();

        for (int i = 0; i < bodies.Length; i++)
            bodies[i].IsSleeping.Should().BeFalse();

        Step(context, 48);

        MaxAnchorError(joints).Should().BeLessThan((Fixed64)3);
        MaxPositionMagnitude(bodies).Should().BeLessThan((Fixed64)96);
    }

    [Fact]
    public void MotorDrivenChainDiagnostics_ShouldReportMotorImpulseAndClamping()
    {
        using GravitasWorldContext context = CreateMotorDrivenChain(out _, out Joint3D[] joints);
        context.Diagnostics.Enable(eventCapacity: 256, drawCommandCapacity: 0);

        Step(context, 8);

        bool foundMotorImpulse = false;
        bool foundClampedRow = false;
        bool foundMotorError = false;
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
        {
            if (!events[i].TryAsJoint(out GravitasJointDiagnosticView view)
                || view.Kind != GravitasDiagnosticEventKind.JointImpulse)
            {
                continue;
            }

            foundMotorImpulse |= view.MotorImpulseMagnitude > Fixed64.Zero;
            foundClampedRow |= view.ClampedRowCount > 0;
            foundMotorError |= view.MotorErrorMagnitude > Fixed64.Zero;
        }

        foundMotorImpulse.Should().BeTrue();
        foundClampedRow.Should().BeTrue();
        foundMotorError.Should().BeTrue();
        SumMotorImpulse(joints).Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintStressSolve_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateHumanoidRagdollOnFloor(out _, out Joint3D[] joints, sleepEnabled: false);
        Step(context, 80);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                context.Simulate();
                context.LateSimulate();
            },
            warmupIterations: 8,
            stabilizationIterations: 4,
            measurementIterations: 128);

        SumSolvedRows(joints).Should().BeGreaterThan(0);
        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void MotorDrivenConstraintStress_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMotorDrivenChain(out _, out Joint3D[] joints);
        Step(context, 32);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                context.Simulate();
                context.LateSimulate();
            },
            warmupIterations: 8,
            stabilizationIterations: 4,
            measurementIterations: 8);

        SumMotorImpulse(joints).Should().BeGreaterThan(Fixed64.Zero);
        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void SolverMetrics_ShouldSeparateStableUnderSolvedAndMotorizedStress()
    {
        using GravitasWorldContext stable = CreateHumanoidRagdollOnFloor(
            out _,
            out Joint3D[] stableJoints,
            sleepEnabled: false);
        Step(stable, 80);
        SumLinearAnchorError(stableJoints).Should().BeLessThan((Fixed64)8);
        SumMotorImpulse(stableJoints).Should().Be(Fixed64.Zero);
        SumClampedRows(stableJoints).Should().Be(0);

        using GravitasWorldContext weakIterations = CreateLongBallSocketChain(
            out _,
            out Joint3D[] weakJoints,
            solverIterations: 1);
        Step(weakIterations, 32);

        using GravitasWorldContext strongerIterations = CreateLongBallSocketChain(
            out _,
            out Joint3D[] strongerJoints,
            solverIterations: 12);
        Step(strongerIterations, 32);

        SumLinearAnchorError(weakJoints).Should().BeGreaterThan(SumLinearAnchorError(strongerJoints));

        using GravitasWorldContext motorized = CreateMotorDrivenChain(out _, out Joint3D[] motorJoints);
        Step(motorized, 8);

        SumMotorImpulse(motorJoints).Should().BeGreaterThan(Fixed64.Zero);
        SumMotorError(motorJoints).Should().BeGreaterThan(Fixed64.Zero);
        SumClampedRows(motorJoints).Should().BeGreaterThan(0);
    }

    private static GravitasWorldContext CreateLongBallSocketChain(
        out SolidBody[] bodies,
        out Joint3D[] joints,
        int solverIterations = 10)
    {
        PhysicsScenarioBuilder scenario = CreateConstraintScenario(gravity: Fixed64.Zero, solverIterations);
        bodies = CreateSphereLine(scenario, 12, spacing: (Fixed64)3, firstImmovable: true);
        joints = new Joint3D[bodies.Length - 1];
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i] = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                bodies[i],
                bodies[i + 1],
                LocalFrame(Vector3d.Right * Fixed64.Half),
                LocalFrame(-Vector3d.Right * Fixed64.Half),
                JointType3D.BallSocket,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));
        }

        bodies[^1].AddLinearImpulse(new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)3));
        return scenario.Context;
    }

    private static GravitasWorldContext CreateAlternatingHingeChain(out SolidBody[] bodies, out Joint3D[] joints)
    {
        PhysicsScenarioBuilder scenario = CreateConstraintScenario(gravity: Fixed64.Zero, solverIterations: 10);
        bodies = CreateSphereLine(scenario, 10, spacing: (Fixed64)2, firstImmovable: true);
        joints = new Joint3D[bodies.Length - 1];
        for (int i = 0; i < joints.Length; i++)
        {
            FixedQuaternion frameRotation = i % 2 == 0
                ? FixedQuaternion.Identity
                : FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi);
            joints[i] = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                bodies[i],
                bodies[i + 1],
                LocalFrame(Vector3d.Right * Fixed64.Half, frameRotation),
                LocalFrame(-Vector3d.Right * Fixed64.Half, frameRotation),
                JointType3D.Hinge,
                JointLimit3D.Hinge(Fixed64.HalfPi),
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));
        }

        bodies[^1].AddAngularImpulse(Vector3d.Forward * (Fixed64)5);
        return scenario.Context;
    }

    private static GravitasWorldContext CreateHumanoidRagdollOnFloor(
        out SolidBody[] bodies,
        out Joint3D[] joints,
        bool sleepEnabled = true)
    {
        PhysicsScenarioBuilder scenario = CreateConstraintScenario(gravity: (Fixed64)10, solverIterations: 12);
        var floor = new LSCuboidCollider { Size = new Vector3d((Fixed64)24, Fixed64.One, (Fixed64)24) };
        scenario.InitializeStaticCollider(floor, new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));

        bodies = new[]
        {
            scenario.CreateCuboid(new Vector3d(Fixed64.Zero, (Fixed64)5, Fixed64.Zero), mass: (Fixed64)3).Body,
            scenario.CreateSphere(new Vector3d(Fixed64.Zero, (Fixed64)7, Fixed64.Zero), mass: Fixed64.One).Body,
            scenario.CreateSphere(new Vector3d((Fixed64)(-2), (Fixed64)5, Fixed64.Zero), mass: Fixed64.One).Body,
            scenario.CreateSphere(new Vector3d((Fixed64)2, (Fixed64)5, Fixed64.Zero), mass: Fixed64.One).Body,
            scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(-3, 4), (Fixed64)3, Fixed64.Zero), mass: Fixed64.One).Body,
            scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), (Fixed64)3, Fixed64.Zero), mass: Fixed64.One).Body
        };
        for (int i = 0; i < bodies.Length; i++)
            bodies[i].SleepEnabled = sleepEnabled;

        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[]
            {
                Link(0, bodies[0]),
                Link(1, bodies[1]),
                Link(2, bodies[2]),
                Link(3, bodies[3]),
                Link(4, bodies[4]),
                Link(5, bodies[5])
            },
            new[]
            {
                RagdollJoint(0, 1, Vector3d.Up, -Vector3d.Up),
                RagdollJoint(0, 2, -Vector3d.Right, Vector3d.Right),
                RagdollJoint(0, 3, Vector3d.Right, -Vector3d.Right),
                RagdollJoint(0, 4, new Vector3d(-Fixed64.Half, -Fixed64.One, Fixed64.Zero), Vector3d.Up),
                RagdollJoint(0, 5, new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.Zero), Vector3d.Up)
            },
            RagdollSelfCollisionPolicy.SuppressAdjacentLinks));
        joints = new Joint3D[runtime.JointCount];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = runtime.GetJoint(i);

        return scenario.Context;
    }

    private static GravitasWorldContext CreateMotorDrivenChain(out SolidBody[] bodies, out Joint3D[] joints)
    {
        PhysicsScenarioBuilder scenario = CreateConstraintScenario(gravity: Fixed64.Zero, solverIterations: 8);
        bodies = CreateSphereLine(scenario, 7, spacing: Fixed64.One, firstImmovable: true);
        var motor = new JointMotor3D(
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi),
            (Fixed64)12,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 8));
        var links = new RagdollLinkDefinition3D[bodies.Length];
        var definitions = new RagdollJointDefinition3D[bodies.Length - 1];
        for (int i = 0; i < bodies.Length; i++)
        {
            links[i] = Link(i, bodies[i]);
            if (i > 0)
                bodies[i].SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.FromFraction(1, 4)));
        }

        for (int i = 0; i < definitions.Length; i++)
        {
            definitions[i] = new RagdollJointDefinition3D(
                i,
                i + 1,
                JointType3D.BallSocket,
                LocalFrame(Vector3d.Right * Fixed64.Half),
                LocalFrame(-Vector3d.Right * Fixed64.Half),
                JointLimit3D.Unrestricted,
                motor,
                JointCollisionPolicy.SuppressLinked);
        }

        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            links,
            definitions,
            RagdollSelfCollisionPolicy.SuppressAllLinks));
        joints = new Joint3D[runtime.JointCount];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = runtime.GetJoint(i);

        return scenario.Context;
    }

    private static PhysicsScenarioBuilder CreateConstraintScenario(Fixed64 gravity, int solverIterations)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.EnsureGrid(96);
        scenario.Context.Environment.Gravity = gravity;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Settings.DiscreteSolverIterations = solverIterations;
        return scenario;
    }

    private static SolidBody[] CreateSphereLine(
        PhysicsScenarioBuilder scenario,
        int count,
        Fixed64 spacing,
        bool firstImmovable)
    {
        var bodies = new SolidBody[count];
        for (int i = 0; i < count; i++)
        {
            bool immovable = firstImmovable && i == 0;
            bodies[i] = scenario.CreateSphere(new Vector3d(spacing * i, Fixed64.Zero, Fixed64.Zero), immovable: immovable).Body;
        }

        return bodies;
    }

    private static RagdollLinkDefinition3D Link(int id, SolidBody body) =>
        new(id, body);

    private static RagdollJointDefinition3D RagdollJoint(
        int first,
        int second,
        Vector3d frameA,
        Vector3d frameB) =>
        new(first, second, JointType3D.BallSocket, LocalFrame(frameA), LocalFrame(frameB));

    private static FixedTransform LocalFrame(Vector3d position) =>
        LocalFrame(position, FixedQuaternion.Identity);

    private static FixedTransform LocalFrame(Vector3d position, FixedQuaternion rotation) =>
        new(position, rotation, Vector3d.One);

    private static void Step(GravitasWorldContext context, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private static Fixed64 MaxAnchorError(ReadOnlySpan<Joint3D> joints)
    {
        Fixed64 max = Fixed64.Zero;
        for (int i = 0; i < joints.Length; i++)
        {
            Joint3D joint = joints[i];
            Vector3d anchorA = joint.BodyA.Position3d + joint.BodyA.Rotation * joint.LocalFrameA.Position;
            Vector3d anchorB = joint.BodyB.Position3d + joint.BodyB.Rotation * joint.LocalFrameB.Position;
            Fixed64 error = (anchorB - anchorA).Magnitude;
            if (error > max)
                max = error;
        }

        return max;
    }

    private static Fixed64 MaxPositionMagnitude(ReadOnlySpan<SolidBody> bodies)
    {
        Fixed64 max = Fixed64.Zero;
        for (int i = 0; i < bodies.Length; i++)
        {
            Fixed64 magnitude = bodies[i].Position3d.Magnitude;
            if (magnitude > max)
                max = magnitude;
        }

        return max;
    }

    private static int SumSolvedRows(ReadOnlySpan<Joint3D> joints)
    {
        int rows = 0;
        for (int i = 0; i < joints.Length; i++)
            rows += joints[i].LastSolveMetrics.PreparedRowCount;

        return rows;
    }

    private static Fixed64 SumLinearAnchorError(ReadOnlySpan<Joint3D> joints)
    {
        Fixed64 sum = Fixed64.Zero;
        for (int i = 0; i < joints.Length; i++)
            sum += joints[i].LastSolveMetrics.LinearAnchorErrorMagnitude;

        return sum;
    }

    private static Fixed64 SumMotorImpulse(ReadOnlySpan<Joint3D> joints)
    {
        Fixed64 sum = Fixed64.Zero;
        for (int i = 0; i < joints.Length; i++)
            sum += joints[i].LastSolveMetrics.MotorImpulseMagnitude;

        return sum;
    }

    private static Fixed64 SumMotorError(ReadOnlySpan<Joint3D> joints)
    {
        Fixed64 sum = Fixed64.Zero;
        for (int i = 0; i < joints.Length; i++)
            sum += joints[i].LastSolveMetrics.MotorErrorMagnitude;

        return sum;
    }

    private static int SumClampedRows(ReadOnlySpan<Joint3D> joints)
    {
        int rows = 0;
        for (int i = 0; i < joints.Length; i++)
            rows += joints[i].LastSolveMetrics.ClampedRowCount;

        return rows;
    }
}
