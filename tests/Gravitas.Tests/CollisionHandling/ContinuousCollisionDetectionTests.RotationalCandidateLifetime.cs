using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalCandidates_WithOneRegisteredSource_ShouldTrackTargetLifetimes(bool movingTarget)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> source = CreateKinematicRotationalCcdBlade(scenario);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, (Fixed64)4).Should().Be(0);

        ScenarioBody<LSSphereCollider> target = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.Half }, Vector3d.Right * (Fixed64)3,
            FixedQuaternion.Identity, immovable: !movingTarget);
        scenario.Context.AdvanceLateSimulateToken();
        source.Body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, (Fixed64)4).Should().BeGreaterThan(0);

        target.Body.Deactivate();
        source.Body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, (Fixed64)4).Should().Be(0);

        source.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(90);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        source.Body.Rotation.Should().Be(PhysicsScenarioBuilder.Yaw(90));
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);

        target.Body.Initialize(Vector3d.Right * (Fixed64)3, FixedQuaternion.Identity,
            movingTarget ? BodyMotionType.Dynamic : BodyMotionType.Static);
        scenario.Context.AdvanceLateSimulateToken();
        source.Body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, (Fixed64)4).Should().BeGreaterThan(0);

        // A lone target is still eligible when the source itself has been unregistered.
        source.Body.Deactivate();
        source.Body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, (Fixed64)4).Should().BeGreaterThan(0);
        target.Body.Deactivate();
        source.Body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, (Fixed64)4).Should().Be(0);
    }

    [Fact]
    public void RotationalCandidateSnapshot_WhenContactCallbackReuses3DDynamicId_ShouldRejectNewLifetime()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> source = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        int dynamicId = target.Body.DynamicId;
        long lifetimeVersion = target.Collider.LifetimeVersion;
        scenario.Context.Physics.PrepareContinuousCollisionFrame();

        source.Collider.OnContactEnter += _ =>
        {
            target.Body.Deactivate();
            target.Body.Initialize(Vector3d.Right * (Fixed64)16, FixedQuaternion.Identity);
            target.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Up);
        };

        source.Collider.NotifyContact(target.Collider, isColliding: true, isChanged: true);

        target.Body.DynamicId.Should().Be(dynamicId);
        target.Collider.LifetimeVersion.Should().BeGreaterThan(lifetimeVersion);
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                (Fixed64)4)
            .Should()
            .BeFalse();
    }
}
