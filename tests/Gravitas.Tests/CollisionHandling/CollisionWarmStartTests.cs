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
        left.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(15, 8) * left.Body.Mass);
        right.Body.AddLinearImpulse(Vector3d.Left * Fixed64.FromFraction(15, 8) * right.Body.Mass);
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
        left.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(15, 8) * left.Body.Mass);
        right.Body.AddLinearImpulse(Vector3d.Left * Fixed64.FromFraction(15, 8) * right.Body.Mass);
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        CollisionResponse.CalculateImpulse(pair);

        pair.Reset();

        pair.TryGetWarmStartImpulse(contactId, out _).Should().BeFalse();
    }

    [Fact]
    public void CalculateImpulse_WithRestingCachedNormalLoad_ShouldApplyStaticFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.FromFraction(1, 4)));
        Vector3d velocityBefore = box.Body.LinearVelocity;
        StoreWarmStartNormalLoad(pair, Fixed64.One);

        CollisionResponse.CalculateImpulse(pair);

        box.Body.LinearVelocity.X.Abs().Should().BeLessThan(velocityBefore.X.Abs());
        box.Body.LinearVelocity.Z.Abs().Should().BeLessThan(velocityBefore.Z.Abs());
        box.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        pair.TryGetWarmStartImpulse(pair.Manifold[0].ContactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithStaleCached3DImpulse_ShouldUnwindWithoutInjectingEnergy()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold[0];
        ulong contactId = contact.ContactId;
        pair.StoreWarmStartImpulse(contactId, contact.Normal, Fixed64.One, Fixed64.One);

        CollisionResponse.CalculateImpulse(pair);

        box.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        box.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().Be(Fixed64.Zero);
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithChanged3DContactNormal_ShouldIgnoreCachedImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact original = pair.Manifold[0];
        pair.StoreWarmStartImpulse(original.ContactId, Vector3d.Up, Fixed64.One, Fixed64.Zero);
        pair.Manifold.SetContact(original.PointA, original.PointB, Fixed64.Zero, Vector3d.Right);
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 4)));
        Vector3d velocityBefore = box.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        box.Body.LinearVelocity.Should().Be(velocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithWarmStartedRestingFriction_ShouldReplayDeterministically()
    {
        WarmStartedState first = RunWarmStartedRestingFrictionSequence();
        WarmStartedState second = RunWarmStartedRestingFrictionSequence();

        second.Should().Be(first);
    }

    private static void StoreWarmStartNormalLoad(CollisionPair pair, Fixed64 normalImpulse)
    {
        for (int i = 0; i < pair.Manifold.Count; i++)
            pair.StoreWarmStartImpulse(pair.Manifold[i].ContactId, pair.Manifold[i].Normal, normalImpulse, Fixed64.Zero);
    }

    private static WarmStartedState RunWarmStartedRestingFrictionSequence()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        StoreWarmStartNormalLoad(pair, Fixed64.One);

        for (int i = 0; i < 4; i++)
        {
            box.Body.AddLinearImpulse(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.FromFraction(1, 8)));
            CollisionResponse.CalculateImpulse(pair);
        }

        pair.TryGetWarmStartImpulse(pair.Manifold[0].ContactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        return new WarmStartedState(
            box.Body.LinearVelocity,
            box.Body.AngularVelocity,
            impulse.NormalImpulse,
            impulse.TangentImpulse,
            impulse.SecondaryTangentImpulse);
    }

    private readonly record struct WarmStartedState(
        Vector3d LinearVelocity,
        Vector3d AngularVelocity,
        Fixed64 NormalImpulse,
        Fixed64 TangentImpulse,
        Fixed64 SecondaryTangentImpulse);
}
