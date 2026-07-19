using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void KinematicRotationalSweep_ShouldSelectNearestStaticTargetIndependentlyOfRegistrationOrder()
    {
        FixedQuaternion nearFirst = RunKinematicRotationalStaticArbitration(registerNearFirst: true);
        FixedQuaternion farFirst = RunKinematicRotationalStaticArbitration(registerNearFirst: false);

        nearFirst.Should().Be(farFirst);
        FixedQuaternion.Angle(FixedQuaternion.Identity, nearFirst)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        FixedQuaternion.Angle(nearFirst, RotationalMovingPairQuarterTurn3D)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void KinematicRotationalSweep_DuplicateStaticTargets_ShouldMatchSingleTargetClamp()
    {
        FixedQuaternion singleTarget = RunKinematicRotationalDuplicateStaticTarget(
            includeDuplicate: false);
        FixedQuaternion duplicateTargets = RunKinematicRotationalDuplicateStaticTarget(
            includeDuplicate: true);

        duplicateTargets.Should().Be(singleTarget);
    }

    [Fact]
    public void DynamicRotationalSweep_ShouldIgnoreUnsupportedDynamicCandidate()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> blade = CreateDynamicRotationalBlade(scenario);
        Vector3d targetPosition = PositionOnRotationalArc(45);
        ScenarioBody<UnsupportedTestCollider3D> unsupported = scenario.CreateBody(
            new UnsupportedTestCollider3D(),
            targetPosition,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        unsupported.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        unsupported.Body.Sleep();
        Vector3d requestedAngularVelocity =
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90);
        blade.Body.ApplyCollisionAngularVelocityDelta(requestedAngularVelocity);

        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        blade.Body.AngularVelocity.Should().Be(requestedAngularVelocity);
        unsupported.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void DynamicRotationalSweep_EqualTimeTargets_ShouldUseStableColliderIdentity()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = CreateDynamicRotationalBlade(scenario);
        Vector3d targetPosition = PositionOnRotationalArc(45);
        ScenarioBody<LSSphereCollider> first = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            targetPosition,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> second = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            targetPosition,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        first.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        second.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        first.Body.Sleep();
        second.Body.Sleep();
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));

        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        first.Body.IsSleeping.Should().BeFalse();
        second.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void DynamicRotationalResponse_ShouldPreserveNonClosingLinearMotion()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> blade = CreateDynamicRotationalBlade(scenario);
        ScenarioBody<LSSphereCollider> target = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) },
            PositionOnRotationalArc(45),
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();
        blade.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(1, 10));
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));

        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        blade.Body.LinearVelocity.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.IsSleeping.Should().BeFalse();
    }

    private static FixedQuaternion RunKinematicRotationalStaticArbitration(bool registerNearFirst)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        Vector3d nearPosition = PositionOnRotationalArc(30);
        Vector3d farPosition = PositionOnRotationalArc(60);
        if (registerNearFirst)
        {
            _ = scenario.CreateStaticSphere(nearPosition);
            _ = scenario.CreateStaticSphere(farPosition);
        }
        else
        {
            _ = scenario.CreateStaticSphere(farPosition);
            _ = scenario.CreateStaticSphere(nearPosition);
        }

        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;

        scenario.Context.LateSimulate();

        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        return blade.Body.Rotation;
    }

    private static FixedQuaternion RunKinematicRotationalDuplicateStaticTarget(
        bool includeDuplicate)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        Vector3d targetPosition = PositionOnRotationalArc(45);
        _ = scenario.CreateStaticSphere(targetPosition);
        if (includeDuplicate)
            _ = scenario.CreateStaticSphere(targetPosition);

        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;

        scenario.Context.LateSimulate();

        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        return blade.Body.Rotation;
    }

    private static ScenarioBody<LSCuboidCollider> CreateDynamicRotationalBlade(
        PhysicsScenarioBuilder scenario) =>
        scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d(
                    (Fixed64)6,
                    Fixed64.One,
                    Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);

    private static Vector3d PositionOnRotationalArc(int degrees) =>
        PhysicsScenarioBuilder.Yaw(degrees)
            * (Vector3d.Right * Fixed64.FromFraction(16, 5));
}
