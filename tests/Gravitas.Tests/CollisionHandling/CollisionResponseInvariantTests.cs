using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionResponseInvariantTests
{
    [Fact]
    public void CalculateImpulse_ForDynamicBodies_ShouldApplyOpposingLinearImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.x;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.x.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.x.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithImmovableBody_ShouldNotMoveImmovableBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> immovable = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(movable.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, immovable.Collider, movable.Collider);
        Vector3d immovableVelocityBefore = immovable.Body.LinearVelocity;
        Fixed64 movableVelocityBefore = movable.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        immovable.Body.LinearVelocity.Should().Be(immovableVelocityBefore);
        movable.Body.LinearVelocity.x.Should().BeGreaterThan(movableVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithTriggerCollider_ShouldNotApplyPhysicalImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> trigger = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> solid = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        PhysicsScenarioBuilder.SetTrigger(trigger.Collider);
        Push(trigger.Body, 60);
        Push(solid.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, trigger.Collider, solid.Collider);
        Vector3d triggerVelocityBefore = trigger.Body.LinearVelocity;
        Vector3d solidVelocityBefore = solid.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        trigger.Body.LinearVelocity.Should().Be(triggerVelocityBefore);
        solid.Body.LinearVelocity.Should().Be(solidVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithZeroRestitution_ShouldDampenWithoutReversingLinearVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.RestitutionCoefficient = Fixed64.Zero;
        right.Body.RestitutionCoefficient = Fixed64.Zero;
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.x;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.x.Should().BeGreaterThan(Fixed64.Zero);
        right.Body.LinearVelocity.x.Should().BeLessThan(Fixed64.Zero);
        left.Body.LinearVelocity.x.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.x.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithOffCenterContact_ShouldApplyAngularImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(
            Fixed64.Fraction(3, 4),
            Fixed64.Fraction(1, 4),
            Fixed64.Zero));
        Push(sphere.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, cuboid.Collider, sphere.Collider);

        CollisionResponse.CalculateImpulse(pair);

        cuboid.Body.AngularVelocity.Should().NotBe(Vector3d.Zero);
    }

    private static CollisionPair CreateDetectedPair(
        PhysicsScenarioBuilder scenario,
        LSCollider colliderA,
        LSCollider colliderB)
    {
        CollisionPair pair = scenario.CreatePair(colliderA, colliderB);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        return pair;
    }

    private static void Push(StiffBody body, int xImpulse)
    {
        body.AddLinearImpulse(new Vector3d((Fixed64)xImpulse, Fixed64.Zero, Fixed64.Zero));
    }
}
