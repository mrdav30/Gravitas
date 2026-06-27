using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DiscreteIslandSolverTests
{
    [Fact]
    public void Simulate_WithRestingBodyUnderGravity_ShouldSettleAfterPostSolve()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0), immovable: true);
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        floor.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Body.UseManualGrounding();
        body.Body.SleepFrameThreshold = 4;

        for (int i = 0; i < 16; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        CollisionPair pair = GetPair(floor.Collider, body.Collider);
        body.Body.IsSleeping.Should().BeTrue(
            $"body should settle; position={body.Body.Position3d}, velocity={body.Body.LinearVelocity}, speed={body.Body.LinearSpeed}, contacts={pair.Manifold.Count}, depth={pair.Manifold.PrimaryContact.Depth}");
        body.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.FromFraction(63, 64));
    }

    [Fact]
    public void Simulate_WithRestingStackUnderGravity_ShouldSettleAsIsland()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0), immovable: true);
        ScenarioBody<LSCuboidCollider> lower = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        ScenarioBody<LSCuboidCollider> upper = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 2, 0),
            preventAngularForces: true);
        floor.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        lower.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        upper.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        lower.Body.UseManualGrounding();
        upper.Body.UseManualGrounding();
        lower.Body.SleepFrameThreshold = 4;
        upper.Body.SleepFrameThreshold = 4;
        lower.Body.SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 100);
        upper.Body.SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 100);

        for (int i = 0; i < 64; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        lower.Body.IsSleeping.Should().BeTrue(
            $"lower stack body should settle; position={lower.Body.Position3d}, velocity={lower.Body.LinearVelocity}, speed={lower.Body.LinearSpeed}");
        upper.Body.IsSleeping.Should().BeTrue(
            $"upper stack body should settle; position={upper.Body.Position3d}, velocity={upper.Body.LinearVelocity}, speed={upper.Body.LinearSpeed}");
        lower.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.FromFraction(31, 32));
        upper.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.FromFraction(125, 64));
    }

    [Fact]
    public void Simulate_WithConnectedSleepingContactIsland_ShouldWakeWholeIsland()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;

        ScenarioBody<LSCuboidCollider> far = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> middle = scenario.CreateCuboid(new Vector3d(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> driver = scenario.CreateCuboid(new Vector3d(
            Fixed64.FromFraction(3, 2),
            Fixed64.Zero,
            Fixed64.Zero));
        far.Body.Sleep();
        middle.Body.Sleep();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        driver.Body.IsSleeping.Should().BeFalse();
        middle.Body.IsSleeping.Should().BeFalse();
        far.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithUnconnectedSleepingIslandInAwakePartition_ShouldNotSolveSleepingIsland()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;

        ScenarioBody<LSSphereCollider> sleepingA = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> sleepingB = scenario.CreateSphere(new Vector3d(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        ScenarioBody<LSSphereCollider> awake = scenario.CreateSphere(new Vector3d(
            (Fixed64)2,
            Fixed64.Zero,
            Fixed64.Zero));
        Vector3d sleepingAPosition = sleepingA.Body.Position3d;
        Vector3d sleepingBPosition = sleepingB.Body.Position3d;

        sleepingA.Body.Sleep();
        sleepingB.Body.Sleep();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        awake.Body.IsSleeping.Should().BeFalse();
        GetPair(sleepingA.Collider, sleepingB.Collider).Manifold.HasContact.Should().BeTrue();
        sleepingA.Body.IsSleeping.Should().BeTrue();
        sleepingB.Body.IsSleeping.Should().BeTrue();
        sleepingA.Body.Position3d.Should().Be(sleepingAPosition);
        sleepingB.Body.Position3d.Should().Be(sleepingBPosition);
    }

    private static CollisionPair GetPair(LSCollider first, LSCollider second)
    {
        if (first.TryGetCollisionPair(second.Id, out CollisionPair? firstPair) && firstPair != null)
            return firstPair;

        second.TryGetCollisionPair(first.Id, out CollisionPair? secondPair).Should().BeTrue();
        return secondPair!;
    }
}
