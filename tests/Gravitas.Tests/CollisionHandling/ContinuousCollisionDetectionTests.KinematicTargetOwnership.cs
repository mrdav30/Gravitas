using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void ContinuousSource_MovingDiscreteKinematicTargetShouldNotLeaveStaleStaticHit3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        source.Body.AddLinearImpulse(Vector3d.Right * (Fixed64)6);
        target.Body.Agent.Transform.LocalPosition = Vector3d.Right * (Fixed64)10;

        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(Vector3d.Right * (Fixed64)3);
        source.Body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)6);
    }

    [Fact]
    public void ContinuousSource_BelowThresholdAutoKinematicTargetShouldUseMovingHitTime3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        source.Body.AddLinearImpulse(Vector3d.Right * (Fixed64)6);
        target.Body.Agent.Transform.LocalPosition =
            Vector3d.Right * Fixed64.FromFraction(1, 4);

        scenario.Context.LateSimulate();

        source.Body.Position3d.X.Should().NotBe(-Fixed64.One);
        source.Body.LinearVelocity.X.Should().BeLessThan(Fixed64.Zero);
    }
}
