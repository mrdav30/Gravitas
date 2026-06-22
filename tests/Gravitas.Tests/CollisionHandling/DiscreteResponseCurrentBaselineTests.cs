using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DiscreteResponseCurrentBaselineTests
{
    [Fact]
    public void CurrentBaseline_RestingStackUnderGravity_ShouldRemainAwakeOverShortWindow()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0), immovable: true);
        ScenarioBody<LSCuboidCollider> lower = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 1, 0));
        ScenarioBody<LSCuboidCollider> upper = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 2, 0));
        lower.Body.SleepFrameThreshold = 100;
        upper.Body.SleepFrameThreshold = 100;

        for (int i = 0; i < 8; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        lower.Body.IsSleeping.Should().BeFalse();
        upper.Body.IsSleeping.Should().BeFalse();
        lower.Body.Position3d.Y.Should().NotBe((Fixed64)1);
        upper.Body.Position3d.Y.Should().NotBe((Fixed64)2);
    }
}
