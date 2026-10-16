using Chronicler.Hashing;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyMotionHardeningTests
{
    [Fact]
    public void CollisionVelocityAdmission_ShouldRejectAnyUnrepresentableComponent()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        body.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Up * Fixed64.MaxValue);

        body.Body.CanApplyCollisionVelocityDeltas(Vector3d.Right, Vector3d.Zero)
            .Should()
            .BeFalse();
        body.Body.CanApplyCollisionVelocityDeltas(Vector3d.Zero, Vector3d.Up)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void LateSimulate_WithAngularSpeedAboveLimit_ShouldClampToEnvironmentMaximum()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.MaxSpeed = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.AddTorque(Vector3d.Up * (Fixed64)16);

        scenario.Context.LateSimulate();

        body.Body.AngularSpeed.Should().Be(Fixed64.One);
    }

    [Fact]
    public void LateSimulate_WithAngularSpeedBelowLimit_ShouldPreserveVelocity()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        Vector3d requestedVelocity = Vector3d.Up * (Fixed64)4;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.ApplyCollisionAngularVelocityDelta(requestedVelocity);

        scenario.Context.LateSimulate();

        body.Body.AngularVelocity.Should().Be(requestedVelocity);
        body.Body.AngularSpeed.Should().Be(requestedVelocity.Magnitude);
    }

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

    [Theory]
    [InlineData(1)]
    [InlineData(64)]
    public void LateSimulate_WhenGroundedAndRotating_ShouldApplyAngularFriction(int impulseDenominator)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        scenario.Context.Environment.Gravity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.AddAngularImpulse(Vector3d.Up / (Fixed64)impulseDenominator);
        Fixed64 speedBeforeFriction = body.Body.AngularSpeed;

        scenario.Context.LateSimulate();

        body.Body.AngularSpeed.Should().BeLessThan(speedBeforeFriction);
    }

    [Theory]
    [InlineData(4, 1, 16, PhysicsRuntimeMode.ThreeD)]
    [InlineData(64, 8, 1024, PhysicsRuntimeMode.ThreeD)]
    [InlineData(4, 1, 16, PhysicsRuntimeMode.Mixed)]
    [InlineData(64, 8, 1024, PhysicsRuntimeMode.Mixed)]
    public void LateSimulate_GroundAngularFriction_ShouldStopWithoutReversing(
        int frameRate, int sizeDenominator, int impulseDenominator, PhysicsRuntimeMode mode)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Settings.RuntimeMode = mode;
        scenario.Context.SetFrameRate(frameRate);
        scenario.Context.Environment.Gravity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider { Size = Vector3d.One / (Fixed64)sizeDenominator },
            Vector3d.Zero, FixedQuaternion.Identity);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.AddAngularImpulse(Vector3d.Up / (Fixed64)impulseDenominator);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(4, PhysicsRuntimeMode.ThreeD)]
    [InlineData(64, PhysicsRuntimeMode.ThreeD)]
    [InlineData(4, PhysicsRuntimeMode.Mixed)]
    [InlineData(64, PhysicsRuntimeMode.Mixed)]
    public void LateSimulate_GroundLinearFriction_ShouldStopWithoutReversing(int frameRate, PhysicsRuntimeMode mode)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Settings.RuntimeMode = mode;
        scenario.Context.SetFrameRate(frameRate);
        scenario.Context.Environment.Gravity = (Fixed64)64;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.AddLinearImpulse(Vector3d.Right / (Fixed64)8);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(Vector3d.Zero);
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

    [Theory]
    [InlineData(-4)]
    [InlineData(-1)]
    [InlineData(0)]
    public void GroundFriction_AfterDrivingMotionReversesOrStops_ShouldNotAddMotion(int drive)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.Gravity = (Fixed64)64;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.AddLinearImpulse(Vector3d.Right / (Fixed64)4);
        body.Body.AddAngularImpulse(Vector3d.Up / (Fixed64)16);
        body.Body.AddForce(Vector3d.Right * (Fixed64)drive);
        body.Body.AddTorque(Vector3d.Up * Fixed64.FromFraction(drive, 4));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GroundLinearFriction_ShouldPreserveSeparatingNormalMotion(bool freezeVertical)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.Gravity = (Fixed64)64;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.FreezeAxes = freezeVertical ? BodyFreezeAxes3D.PositionY : BodyFreezeAxes3D.None;
        body.Body.SetManualGrounding(Vector3d.Zero,
            freezeVertical ? new Vector3d(0, 3, 4) : Vector3d.Up);
        Vector3d separatingVelocity = freezeVertical ? Vector3d.Forward : Vector3d.Up;
        body.Body.AddLinearImpulse(Vector3d.Right + separatingVelocity);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(separatingVelocity);
    }

    [Fact]
    public void GroundLinearFriction_WithTinyPermittedNormal_ShouldNotInventATangent()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.Environment.Gravity = (Fixed64)64;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.PositionZ;
        body.Body.SetManualGrounding(Vector3d.Zero,
            new Vector3d(Fixed64.FromFraction(1, 8192), Fixed64.One, Fixed64.Zero));
        body.Body.AddLinearImpulse(Vector3d.Right);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(Vector3d.Right);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GroundAngularFriction_WithAnisotropicInertia_ShouldReduceEnergy(bool freezeVerticalRotation)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        scenario.Context.Environment.Gravity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider { Size = new Vector3d(1, 2, 3) },
            Vector3d.Zero, FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45));
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.FreezeAxes = freezeVerticalRotation ? BodyFreezeAxes3D.RotationY : BodyFreezeAxes3D.None;
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.AddAngularImpulse(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        Vector3d before = body.Body.AngularVelocity;
        Fixed3x3 inertia = GetWorldInertia(body.Body);
        Fixed64 energyBefore = Vector3d.Dot(before, inertia * before);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        Vector3d after = body.Body.AngularVelocity;
        inertia = GetWorldInertia(body.Body);
        Vector3d.CompareMagnitudeSquared(after, Vector3d.Zero).Should().BeGreaterThan(0);
        Vector3d.Dot(after, inertia * after).Should().BeLessThan(energyBefore);
        if (freezeVerticalRotation)
            after.Y.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(10, 1, false)]
    [InlineData(100, 100, true)]
    [InlineData(1000, 1000, true)]
    public void GroundAngularFriction_WithFrozenAxisAndRotatedLongBody_ShouldNotAddEnergy(
        int length, int thicknessDenominator, bool isolateFriction)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        scenario.Context.Environment.Gravity = (Fixed64)64;
        Vector3d longAxis = new Vector3d(1, 2, 1).Normalized;
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Cross(Vector3d.Right, longAxis).Normalized, FixedMath.Acos(longAxis.X));
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider { Size = new Vector3d((Fixed64)length, Fixed64.FromFraction(1, thicknessDenominator), Fixed64.FromFraction(1, thicknessDenominator)) }, Vector3d.Zero, rotation);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.RotationZ;
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Right);
        Fixed3x3 inertia = GetWorldInertia(body.Body);
        Fixed64 before = Vector3d.Dot(body.Body.AngularVelocity, inertia * body.Body.AngularVelocity);

        if (isolateFriction)
        {
            // Separate friction from the existing explicit gyroscopic integration at extreme aspect ratios.
            body.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken).Should().BeTrue();
        }
        else
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        inertia = GetWorldInertia(body.Body);
        Vector3d after = body.Body.AngularVelocity;
        Vector3d.Dot(after, inertia * after).Should().BeLessThan(before);
        after.Z.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GroundAngularFriction_WhenShapeQuantizationRemovesAnInertiaAxis_ShouldNotInventTorque(bool freezeOtherAxis)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        scenario.Context.Environment.Gravity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        body.Body.FreezeAxes = freezeOtherAxis ? BodyFreezeAxes3D.RotationZ : BodyFreezeAxes3D.None;
        body.Body.AddAngularImpulse(Vector3d.Right / (Fixed64)16);
        Vector3d velocity = body.Body.AngularVelocity;
        body.Collider.Size = new Vector3d(Fixed64.One, Fixed64.FromRaw(2), Fixed64.FromRaw(2));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.Body.CanRotate.Should().BeTrue();
        body.Body.InverseInertiaTensor.M11.Should().Be(Fixed64.Zero);
        body.Body.AngularVelocity.Should().Be(velocity);
    }

    [Theory]
    [InlineData(300000000, false)]
    [InlineData(1000000000, false)]
    [InlineData(1000000000, true)]
    public void GroundAngularFriction_WithUnrepresentableCompletedVelocity_ShouldRejectNumericExhaustion(int friction, bool freezeOtherAxis)
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        SolidBody body = CreateFastAnisotropicBody(scenario, 1000000000, friction);
        body.FreezeAxes = freezeOtherAxis ? BodyFreezeAxes3D.RotationY : BodyFreezeAxes3D.None;
        Vector3d before = body.AngularVelocity;

        Action prepare = () => body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);

        prepare.Should().Throw<OverflowException>().WithMessage("Dissipation produces an unrepresentable velocity.");
        body.AngularVelocity.Should().Be(before);
    }

    [Fact]
    public void GroundAngularFriction_WithOversizedCorrectionButFiniteVelocity_ShouldMaterializeOnlyTheResult()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        SolidBody body = CreateFastAnisotropicBody(scenario, 620000000, 1000000000);
        Vector3d before = body.AngularVelocity;

        body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken).Should().BeTrue();

        body.AngularVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body.AngularVelocity.Z.Should().BeLessThan(Fixed64.Zero);
        Fixed64.TrySubtract(before.Z, body.AngularVelocity.Z, out _).Should().BeFalse();
    }

    [Fact]
    public void ConstrainedGroundFriction_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateMotionScenario();
        scenario.Context.SetFrameRate(64);
        scenario.Context.Environment.Gravity = (Fixed64)64;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45);
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider { Size = new Vector3d(10, 1, 1) }, Vector3d.Zero, rotation);
        body.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.One, Fixed64.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.RotationZ;
        body.Body.SleepEnabled = false;

        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.Body.ResetPosition(Vector3d.Zero, rotation);
            body.Body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
            body.Body.AddAngularImpulse(Vector3d.Right);
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }, measurementIterations: 128);

        allocated.Should().Be(0);
        body.Body.AngularVelocity.Z.Should().Be(Fixed64.Zero);
        body.Body.AngularSpeed.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithAnisotropicTorqueFreeRotation_ShouldUseEulerSignAndRefreshMotionStateDeterministically()
    {
        Vector3d initialVelocity = new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        AngularStepResult first = RunAnisotropicAngularStep(Vector3d.Zero);
        AngularStepResult second = RunAnisotropicAngularStep(Vector3d.Zero);
        Fixed3x3 localInertia = first.LocalInertia;
        Fixed3x3 inverseLocalInertia = InertiaTensorMath.InvertForSolver(localInertia);
        Fixed3x3 orientation = first.Rotation.Conjugate().ToMatrix3x3();
        Fixed3x3 inverseOrientation = first.Rotation.ToMatrix3x3();
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
    public void QueuedLinearCcdHandoff_AfterNormalAngularStep_ShouldContinueAngularState()
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
        Fixed64 remainingTime = scenario.Context.DeltaTime * Fixed64.Half;
        FixedQuaternion angularVelocityQuaternion = new(
            angularVelocity.X,
            angularVelocity.Y,
            angularVelocity.Z,
            Fixed64.Zero);
        FixedQuaternion expectedRotation = (rotation
            + angularVelocityQuaternion * rotation * Fixed64.Half * remainingTime).Normalized;

        body.Body.ApplyContinuousCollisionHandoff(
            body.Body.Position3d,
            Vector3d.Right,
            remainingTime);

        scenario.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(
            iterationBudget: 1).Should().Be(1);
        body.Body.AngularVelocity.Should().Be(angularVelocity);
        body.Body.AngularSpeed.Should().Be(angularSpeed);
        body.Body.AngularAcceleration.Should().Be(angularAcceleration);
        body.Body.Rotation.Should().Be(expectedRotation);
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

    private static Fixed3x3 GetWorldInertia(SolidBody body) =>
        body.Rotation.Conjugate().ToMatrix3x3()
        * body.Collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset)
        * body.Rotation.ToMatrix3x3();

    private static SolidBody CreateFastAnisotropicBody(PhysicsScenarioBuilder scenario, int speed, int friction)
    {
        scenario.Context.Environment.Gravity = (Fixed64)64;
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d(1, 1, 10),
            Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution((Fixed64)friction, Fixed64.Zero)
        };
        SolidBody body = scenario.CreateBody(collider, Vector3d.Zero, FixedQuaternion.Identity,
            mass: Fixed64.FromFraction(1, 1024)).Body;
        body.SetManualGrounding(Vector3d.Zero, Vector3d.Up);
        Fixed3x3 inertia = collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset);
        body.AddAngularImpulse(inertia * new Vector3d((Fixed64)speed, Fixed64.Zero, Fixed64.FromFraction(speed, 8)));
        return body;
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
