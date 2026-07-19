using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
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
