using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionPairPriorityTests
{
    [Fact]
    public void AssignPriority_WithSamePriorityBodies_ShouldUseLinearSpeedBeforeOriginalOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> slow = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> fast = scenario.CreateSphere(Vector3d.Right * Fixed64.Fraction(3, 4));
        fast.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Right * (Fixed64)4);

        CollisionPair pair = scenario.CreatePair(slow.Collider, fast.Collider);

        pair.ColliderA.Should().BeSameAs(fast.Collider);
        pair.ColliderB.Should().BeSameAs(slow.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void AssignPriority_WithDifferentPriorities_ShouldKeepShapePriorityAboveLinearSpeed()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> fastSphere = scenario.CreateSphere(Vector3d.Right * Fixed64.Fraction(3, 4));
        ScenarioBody<LSCuboidCollider> slowCuboid = scenario.CreateCuboid(Vector3d.Zero);
        fastSphere.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Right * (Fixed64)8);

        CollisionPair pair = scenario.CreatePair(fastSphere.Collider, slowCuboid.Collider);

        pair.ColliderA.Should().BeSameAs(slowCuboid.Collider);
        pair.ColliderB.Should().BeSameAs(fastSphere.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Right);
    }
}
