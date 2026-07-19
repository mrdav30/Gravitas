using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void PreparedKinematicTrajectory3D_ShouldConsumeCapturedHostEndpoint()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        Vector3d capturedPosition = new(Fixed64.One, Fixed64.Two, (Fixed64)3);
        FixedQuaternion capturedRotation = PhysicsScenarioBuilder.Yaw(45);
        mover.Body.Agent.Transform.LocalPosition = capturedPosition;
        mover.Body.Agent.Transform.LocalRotation = capturedRotation;

        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        mover.Body.Agent.Transform.LocalPosition = new Vector3d((Fixed64)4, (Fixed64)5, (Fixed64)6);
        mover.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(90);

        scenario.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: true)
            .Should()
            .BeTrue();

        mover.Body.Position3d.Should().Be(capturedPosition);
        mover.Body.Rotation.Should().Be(capturedRotation);
        mover.Body.Agent.Transform.LocalPosition.Should().Be(capturedPosition);
        mover.Body.Agent.Transform.LocalRotation.Should().Be(capturedRotation);
    }

    [Fact]
    public void HandoffTrajectory3D_ShouldReplaceSupersededTailAndRetainIncreasingHistory()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        mover.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);

        mover.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.FromFraction(1, 4),
            Vector3d.Right,
            Fixed64.FromFraction(3, 4));
        mover.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            Vector3d.Right,
            Fixed64.FromFraction(1, 4));
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);

        mover.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.Half,
            Vector3d.Left,
            Fixed64.Half);
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);
        mover.Body.SampleContinuousCollisionPosition(Fixed64.Half)
            .Should()
            .Be(Vector3d.Right * Fixed64.Half);

        mover.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.Half,
            Vector3d.Right,
            Fixed64.Half);
        mover.Body.ContinuousCollisionTrajectoryCount.Should().Be(3);
    }

    [Fact]
    public void SubthresholdHandoffVelocity3D_ShouldPublishStationaryTrajectory()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        mover.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);
        Vector3d impactPosition = Vector3d.Right;

        mover.Body.ApplyContinuousCollisionHandoff(
            impactPosition,
            Vector3d.Right * Fixed64.FromRaw(2),
            Fixed64.Half);

        mover.Body.SampleContinuousCollisionPosition(Fixed64.One)
            .Should()
            .Be(impactPosition);
    }
}
