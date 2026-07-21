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
    public void ContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldBlockTranslationalSource()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 4;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * (Fixed64)6);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up * (Fixed64)6,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Body.Position3d.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void ContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldReceiveKinematicHandoff()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * (Fixed64)6);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up * (Fixed64)6,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_TargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * (Fixed64)6);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Up,
                FixedQuaternion.Identity,
                Vector3d.Up * (Fixed64)6,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Body.Position3d.X.Should().Be((Fixed64)5);
    }
}
