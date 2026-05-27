using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class StiffBodySleepTests
{
    [Fact]
    public void LateSimulate_AtRestForSleepWindow_ShouldPutBodyToSleep()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        body.Body.SleepFrameThreshold = 2;

        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void SleepingBody_ShouldWakeFromDeterministicStimuli()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        body.Body.Sleep();
        body.Body.IsSleeping.Should().BeTrue();

        body.Body.AddForce(Vector3d.Right);

        body.Body.IsSleeping.Should().BeFalse();

        body.Body.Sleep();
        body.Body.SetPosition(Vector3d.Right);

        body.Body.IsSleeping.Should().BeFalse();

        body.Body.Sleep();
        body.Body.AddLinearImpulse(Vector3d.Right);

        body.Body.IsSleeping.Should().BeFalse();

        body.Body.Sleep();
        body.Body.AddAngularImpulse(Vector3d.Up);

        body.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ShapeMutation_ShouldWakeSleepingBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        body.Body.Sleep();

        body.Collider.Radius = Fixed64.One;

        body.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_StackedRestingBodies_ShouldSleepDeterministically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> lower = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> upper = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 1, 0));
        lower.Body.SleepFrameThreshold = 1;
        upper.Body.SleepFrameThreshold = 1;

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        lower.Body.IsSleeping.Should().BeTrue();
        upper.Body.IsSleeping.Should().BeTrue();
    }
}
