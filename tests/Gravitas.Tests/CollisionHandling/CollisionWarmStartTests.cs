using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionWarmStartTests
{
    [Fact]
    public void CalculateImpulse_ShouldStoreWarmStartImpulseByStableContactIdentity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.AddLinearImpulse(new Vector3d((Fixed64)60, Fixed64.Zero, Fixed64.Zero));
        right.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;

        CollisionResponse.CalculateImpulse(pair);

        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Reset_ShouldClearWarmStartState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.AddLinearImpulse(new Vector3d((Fixed64)60, Fixed64.Zero, Fixed64.Zero));
        right.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        CollisionResponse.CalculateImpulse(pair);

        pair.Reset();

        pair.TryGetWarmStartImpulse(contactId, out _).Should().BeFalse();
    }
}
