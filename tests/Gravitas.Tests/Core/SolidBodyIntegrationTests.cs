using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyIntegrationTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 1_000_000);

    [Fact]
    public void LateSimulate_ShouldIntegrateForceVelocityAndPosition()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);

        body.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        body.Body.Position3d.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        body.Body.IsAtRest.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_ShouldApplyLinearDragWithFixedExpectedValue()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.AirDensity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.LinearDragCoefficient = Fixed64.One;

        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.X.Should().Be(Fixed64.FromFraction(3, 4));
        body.Body.Position3d.X.Should().Be(Fixed64.FromFraction(7, 16));
    }

    [Fact]
    public void LateSimulate_ShouldApplyGroundFrictionWithFixedExpectedValue()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = Fixed64.One;
        CreateGround(scenario);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.LinearVelocity.X.Should().Be(Fixed64.FromFraction(3, 4));
        body.Body.Position3d.X.Should().Be(Fixed64.FromFraction(7, 16));
    }

    [Fact]
    public void GravityScale_WhenChangedWhileGrounded_ShouldRefreshGroundFrictionWeight()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = Fixed64.One;
        CreateGround(scenario);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.GravityScale = Fixed64.Zero;
        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.LinearVelocity.X.Should().Be(Fixed64.One);
        body.Body.Position3d.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void LateSimulate_WithDefaultGravityScale_ShouldApplyEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Y.Should().Be(-Fixed64.One);
        body.Body.Position3d.Y.Should().Be(-Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void LateSimulate_WithHalfGravityScale_ShouldApplyScaledEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GravityScale = Fixed64.Half;

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Y.Should().Be(-Fixed64.Half);
        body.Body.Position3d.Y.Should().Be(-Fixed64.FromFraction(1, 8));
    }

    [Fact]
    public void LateSimulate_WithZeroGravityScale_ShouldIgnoreEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GravityScale = Fixed64.Zero;

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.Body.Position3d.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void GravityScale_ShouldRejectNegativeValues()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        Action action = () => body.Body.GravityScale = -Fixed64.Epsilon;

        action.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void ContinuousCollisionPrediction_WithGravityScale_ShouldUseScaledEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GravityScale = Fixed64.Half;

        body.Body.EnsureContinuousCollisionFramePrepared(123);

        body.Body.ContinuousCollisionFrameDisplacement.Should().Be(new Vector3d(
            Fixed64.Zero,
            -Fixed64.FromFraction(1, 8),
            Fixed64.Zero));
    }

    [Fact]
    public void LateSimulate_ShouldIntegrateTorqueAndAngularDamping()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);

        body.Body.AddTorque(Vector3d.Up);
        scenario.Context.LateSimulate();

        AssertNear(body.Body.AngularVelocity.Y, Fixed64.FromFraction(3, 2));

        scenario.Context.Environment.DampingFactor = Fixed64.Half;
        scenario.Context.LateSimulate();

        AssertNear(body.Body.AngularVelocity.Y, Fixed64.FromFraction(3, 8));
    }

    [Fact]
    public void ResetPosition_ShouldReturnMovingBodyToRest()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();

        body.Body.IsAtRest.Should().BeFalse();

        body.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);

        body.Body.IsAtRest.Should().BeTrue();
        body.Body.Position3d.Should().Be(Vector3d.Zero);
        body.Body.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    [Fact]
    public void SetHeightAndFreezeAxes_ShouldExposeLifecycleStateWithoutWakingOnNoOp()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.Sleep();

        body.Body.SetHeight(body.Body.HeightPos);

        body.Body.IsSleeping.Should().BeTrue();
        body.Body.AngularForcesHalted.Should().BeFalse();

        body.Body.SetHeight(Fixed64.One);

        body.Body.HeightPos.Should().Be(Fixed64.One);
        body.Body.IsSleeping.Should().BeFalse();

        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.Body.AngularForcesHalted.Should().BeTrue();

        body.Body.FreezeAxes = BodyFreezeAxes3D.None;
        body.Body.AngularForcesHalted.Should().BeFalse();

        body.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        body.Body.AngularForcesHalted.Should().BeTrue();
    }

    [Fact]
    public void Visualize_WithResetAccumulation_ShouldPublishAuthoritativeRotationTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.CanSetVisualRotation = true;
        body.Body.DefaultRotationSpeed = Fixed64.One;
        FixedQuaternion target = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);

        body.Body.UpdateRotation(target, Fixed64.Zero);
        scenario.Context.LateSimulate();
        scenario.Context.Visualize();

        FixedQuaternion.Angle(body.Body.RotationTransform.Rotation, target).Should().BeLessThan(Tolerance);
        body.Body.LastVisualRotation.Should().Be(FixedQuaternion.Identity);
        body.Body.VisualRotation.Should().Be(target);
    }

    [Fact]
    public void Visualize_WithSpeedLimitedRotation_ShouldAdvanceAcrossSteadyFrames()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.CanSetVisualRotation = true;
        body.Body.DefaultRotationSpeed = Fixed64.One;
        FixedQuaternion target = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);

        body.Body.UpdateRotation(target, Fixed64.One);
        scenario.Context.LateSimulate();
        scenario.Context.Visualize();
        FixedQuaternion firstVisual = body.Body.RotationTransform.Rotation;
        scenario.Context.Visualize();
        FixedQuaternion secondVisual = body.Body.RotationTransform.Rotation;

        firstVisual.Should().NotBe(FixedQuaternion.Identity);
        firstVisual.Should().NotBe(target);
        secondVisual.Should().NotBe(firstVisual);
        FixedQuaternion.Angle(secondVisual, target).Should().BeLessThan(FixedQuaternion.Angle(firstVisual, target));
    }

    private static PhysicsScenarioBuilder CreateIntegrationScenario(int frameRate)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var settings = new PhysicsSettings(frameRate, null, PhysicsLayerMask.FromLayer(1));

        scenario.Context.ApplySettings(settings);

        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        scenario.Context.Environment.MaxFallSpeed = (Fixed64)100;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;

        return scenario;
    }

    private static void CreateGround(PhysicsScenarioBuilder scenario)
    {
        var transform = new FixedTransform(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var collider = new LSCuboidCollider
        {
            Layer = new PhysicsLayer(1),
            Size = new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8)
        };

        collider.InitializeWithNoBody(agent);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected)
    {
        (actual - expected).Abs().Should().BeLessThan(Tolerance);
    }
}
