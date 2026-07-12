using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyMotionHardeningTests
{
    [Fact]
    public void Initialize_AfterDeactivate_ShouldDiscardPriorAndQueuedMotion()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);

        body.Body.AddTorque(Vector3d.Up);
        scenario.Context.LateSimulate();
        body.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Right);
        body.Body.AngularAcceleration.Should().NotBe(Vector3d.Zero);

        body.Body.AddForce(Vector3d.Right * (Fixed64)8);
        body.Body.AddTorque(Vector3d.Up);
        body.Body.Deactivate();
        body.Body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        body.Body.LinearAcceleration.Should().Be(Vector3d.Zero);
        body.Body.AngularAcceleration.Should().Be(Vector3d.Zero);

        using PhysicsScenarioBuilder freshScenario = CreateMotionScenario();
        ScenarioBody<LSCuboidCollider> fresh = freshScenario.CreateCuboid(Vector3d.Zero);
        HashBody(body.Body).Should().Be(HashBody(fresh.Body));

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        body.Body.Position3d.Should().Be(Vector3d.Zero);
        body.Body.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    [Fact]
    public void ResetPosition_ShouldDiscardPriorAndQueuedMotion()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);

        body.Body.AddForce(Vector3d.Right * (Fixed64)4);
        body.Body.AddTorque(Vector3d.Up);
        scenario.Context.LateSimulate();
        body.Body.AddForce(Vector3d.Forward * (Fixed64)8);
        body.Body.AddTorque(Vector3d.Right);
        body.Body.AddLinearImpulse(Vector3d.Up);
        Vector3d resetPosition = new((Fixed64)3, (Fixed64)4, (Fixed64)5);

        body.Body.ResetPosition(resetPosition, FixedQuaternion.Identity);
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        body.Body.LinearAcceleration.Should().Be(Vector3d.Zero);
        body.Body.AngularAcceleration.Should().Be(Vector3d.Zero);
        body.Body.Position3d.Should().Be(resetPosition);
        body.Body.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    [Fact]
    public void LateSimulate_WhenGroundedAndRotating_ShouldApplyAngularFriction()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.Gravity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.AddAngularImpulse(Vector3d.Up);
        Fixed64 speedBeforeFriction = body.Body.AngularSpeed;

        scenario.Context.LateSimulate();

        body.Body.AngularSpeed.Should().BeLessThan(speedBeforeFriction);
    }

    [Fact]
    public void LateSimulate_WithGroundedVerticalMotion_ShouldNotInventHorizontalFriction()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.Gravity = Fixed64.One;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.AddForce(Vector3d.Up * (Fixed64)4);

        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        body.Body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
        body.Body.LinearVelocity.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithAnisotropicTorqueFreeRotation_ShouldUseEulerSignAndRefreshMotionStateDeterministically()
    {
        Vector3d initialVelocity = new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        AngularStepResult first = RunAnisotropicAngularStep(Vector3d.Zero);
        AngularStepResult second = RunAnisotropicAngularStep(Vector3d.Zero);
        Fixed3x3 localInertia = first.LocalInertia;
        Fixed3x3 inverseLocalInertia = InertiaTensorMath.InvertForSolver(localInertia);
        Fixed3x3 orientation = first.Rotation.ToMatrix3x3();
        Fixed3x3 inverseOrientation = first.Rotation.Conjugate().ToMatrix3x3();
        Fixed3x3 worldInertia = orientation * localInertia * inverseOrientation;
        Fixed3x3 inverseWorldInertia = orientation * inverseLocalInertia * inverseOrientation;
        Vector3d expectedVelocity = initialVelocity
            - inverseWorldInertia
            * Vector3d.Cross(initialVelocity, worldInertia * initialVelocity)
            * first.DeltaTime;

        second.Should().Be(first);
        first.Velocity.Should().Be(expectedVelocity);
        first.Speed.Should().Be(first.Velocity.Magnitude);
        first.Acceleration.Should().Be(
            (first.Velocity - initialVelocity) / first.DeltaTime);
    }

    [Fact]
    public void LateSimulate_WithTorqueAndGyroscopicPrecession_ShouldMeasureAccelerationFromStepStart()
    {
        Vector3d initialVelocity = new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        AngularStepResult result = RunAnisotropicAngularStep(Vector3d.Forward);

        result.Acceleration.Should().Be(
            (result.Velocity - initialVelocity) / result.DeltaTime);
    }

    [Fact]
    public void QueuedLinearCcdHandoff_AfterNormalAngularStep_ShouldPreserveAngularState()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3)
        };
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        body.Body.ApplyCollisionAngularVelocityDelta(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));
        body.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);

        scenario.Context.Physics.BeginLateSimulateBodies(
            continuousCollisionFramePrepared: false).Should().BeTrue();
        Vector3d angularVelocity = body.Body.AngularVelocity;
        Fixed64 angularSpeed = body.Body.AngularSpeed;
        Vector3d angularAcceleration = body.Body.AngularAcceleration;
        FixedQuaternion rotation = body.Body.Rotation;

        body.Body.ApplyContinuousCollisionHandoff(
            body.Body.Position3d,
            Vector3d.Right,
            scenario.Context.DeltaTime * Fixed64.Half);

        scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(
            iterationBudget: 1).Should().Be(1);
        body.Body.AngularVelocity.Should().Be(angularVelocity);
        body.Body.AngularSpeed.Should().Be(angularSpeed);
        body.Body.AngularAcceleration.Should().Be(angularAcceleration);
        body.Body.Rotation.Should().Be(rotation);
    }

    [Fact]
    public void LateSimulate_WithZeroSleepFrameThreshold_ShouldSleepImmediately()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.SleepFrameThreshold = 0;

        scenario.Context.LateSimulate();

        body.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void SetRotation_WithCurrentRotation_ShouldNotWakeSleepingBody()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.Sleep();

        body.Body.SetRotation(body.Body.Rotation);

        body.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithFrozenSolverRotation_ShouldPreserveDirectAuthoredRotationWithoutGyro()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        FixedQuaternion authoredRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);

        body.Body.SetRotation(authoredRotation);
        scenario.Context.LateSimulate();

        body.Body.Rotation.Should().Be(authoredRotation);
        body.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        body.Body.AngularSpeed.Should().Be(Fixed64.Zero);
        body.Body.CanRotate.Should().BeFalse();
    }

    [Fact]
    public void Wake_AfterDeactivate_ShouldClearSleepWithoutRefreshingRemovedPartition()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.Sleep();
        body.Body.Deactivate();

        body.Body.Wake();

        body.Body.IsSleeping.Should().BeFalse();
        body.Collider.IsPartitioned.Should().BeFalse();
    }

    private static PhysicsScenarioBuilder CreateMotionScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        scenario.Context.Environment.MaxFallSpeed = (Fixed64)100;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        return scenario;
    }

    private static AngularStepResult RunAnisotropicAngularStep(Vector3d torque)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3)
        };
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        body.Body.ApplyCollisionAngularVelocityDelta(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));
        body.Body.AddTorque(torque);

        scenario.Context.LateSimulate();

        return new AngularStepResult(
            body.Collider.CalculateInertiaTensor(body.Body.Mass, body.Body.LocalCenterOfMassOffset),
            body.Body.AngularVelocity,
            body.Body.AngularSpeed,
            body.Body.AngularAcceleration,
            body.Body.Rotation,
            scenario.Context.DeltaTime);
    }

    private static ChronicleHash HashBody(SolidBody body)
    {
        var writer = new ChronicleHashWriter();
        body.ContributeReplayHash(ref writer, GravitasReplayHashMode.Authoritative);
        return writer.ToHash();
    }

    private readonly record struct AngularStepResult(
        Fixed3x3 LocalInertia,
        Vector3d Velocity,
        Fixed64 Speed,
        Vector3d Acceleration,
        FixedQuaternion Rotation,
        Fixed64 DeltaTime);
}
