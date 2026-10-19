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
        floor.Collider.Material = box.Collider.Material =
            new Gravitas.Materials.PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactsPerGroup);
        // A cached load represents sustained support only when a real incoming
        // normal load replenishes it; stale impulses alone cannot grant friction.
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.FromFraction(1, 4)));
        Vector3d velocityBefore = box.Body.LinearVelocity;
        StoreWarmStartNormalLoad(pair, Fixed64.Quarter);

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
        first.TangentImpulse.Should().NotBe(Fixed64.Zero);
        first.SecondaryTangentImpulse.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void WarmStartCache_Remove_ShouldPreserveRemainingStableOrder()
    {
        ContactWarmStartCache cache = default;
        cache.Set(11UL, Vector3d.Right, Fixed64.One, Fixed64.Zero);
        cache.Set(22UL, Vector3d.Up, Fixed64.Two, Fixed64.Zero);
        cache.Set(33UL, Vector3d.Forward, (Fixed64)3, Fixed64.Zero);

        cache.Remove(22UL).Should().BeTrue();

        cache.Count.Should().Be(2);
        cache.GetGroup(0).GetContactId(0).Should().Be(11UL);
        cache.GetGroup(0).GetContactId(1).Should().Be(33UL);
        cache.TryGet(22UL, out _).Should().BeFalse();
        cache.Remove(44UL).Should().BeFalse();

        ContactWarmStartCache2D cache2D = default;
        cache2D.Set(
            default, 11UL,
            Fixed64.One,
            Fixed64.Zero);
        cache2D.Set(
            default, 22UL,
            Fixed64.Two,
            Fixed64.Zero);

        cache2D.Remove(default, 11UL).Should().BeTrue();

        cache2D.Count.Should().Be(1);
        cache2D.GetGroup(0).GetContactId(0).Should().Be(22UL);
        cache2D.TryGet(default, 11UL, out _).Should().BeFalse();
        cache2D.Remove(default, 33UL).Should().BeFalse();
    }

    [Fact]
    public void WarmStartCache2D_ShouldRetainIndependentGroupImpulsesAndPruneVanishedRows()
    {
        ContactWarmStartCache2D cache = default;
        var manifold = new ContactManifold2D();
        for (int g = 3; g >= 0; g--)
        {
            var key = new ContactGroupKey(0, 0, region: g);
            cache.Set(key, 11UL, (Fixed64)(g + 1), Fixed64.Half);
            cache.Set(key, 22UL, (Fixed64)(g + 2), Fixed64.Zero);
            if (g == 2) continue;
            manifold.AddContact(key, new ManifoldContact2D(11UL, Vector2d.Zero,
                Vector2d.Right, Fixed64.One, Vector2d.Right));
        }
        cache.Count.Should().Be(8);
        cache.Retain(manifold);
        cache.GroupCount.Should().Be(3);
        cache.Count.Should().Be(3);
        for (int g = 0; g < 4; g++)
        {
            var key = new ContactGroupKey(0, 0, region: g);
            cache.TryGet(key, 11UL, out var impulse).Should().Be(g != 2);
            if (g != 2) impulse.NormalImpulse.Should().Be((Fixed64)(g + 1));
            cache.TryGet(key, 22UL, out _).Should().BeFalse();
        }
        cache.GetGroup(2).GetContactId(0).Should().Be(11UL);
        cache.GetGroup(2).GetImpulseUnchecked(0).NormalImpulse.Should().Be((Fixed64)4);
        cache.Remove(new ContactGroupKey(0, 0, region: 1), 11UL).Should().BeTrue();
        cache.Remove(new ContactGroupKey(0, 0, region: 1), 11UL).Should().BeFalse();
        manifold.Reset();
        cache.Retain(manifold);
        cache.Count.Should().Be(0);
        cache.GroupCount.Should().Be(0);
        cache.Set(default, 11UL, Fixed64.One, Fixed64.Zero);
        cache.Clear();
        cache.Count.Should().Be(0);
    }

    [Fact]
    public void WarmStartCache2D_WhenAContactIsReplaced_ShouldRetainTheOtherRowAndOtherGroups()
    {
        ContactWarmStartCache2D cache = default;
        var key = new ContactGroupKey(0, 0, region: 1);
        cache.Set(key, 11UL, Fixed64.One, Fixed64.Zero);
        cache.Set(key, 22UL, Fixed64.Two, Fixed64.Half);
        cache.Set(default, 22UL, (Fixed64)7, -Fixed64.One);

        // Capacity belongs to each structural region; replacing its second
        // row must preserve its first row and every independent region.
        cache.Set(key, 33UL, (Fixed64)3, -Fixed64.Half);

        cache.Count.Should().Be(3);
        cache.GroupCount.Should().Be(2);
        cache.TryGet(key, 11UL, out var first).Should().BeTrue();
        first.NormalImpulse.Should().Be(Fixed64.One);
        cache.TryGet(key, 22UL, out _).Should().BeFalse();
        cache.TryGet(key, 33UL, out var replacement).Should().BeTrue();
        replacement.NormalImpulse.Should().Be((Fixed64)3);
        replacement.TangentImpulse.Should().Be(-Fixed64.Half);
        cache.TryGet(default, 22UL, out var independent).Should().BeTrue();
        independent.NormalImpulse.Should().Be((Fixed64)7);
        independent.TangentImpulse.Should().Be(-Fixed64.One);
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
        pair.Manifold.SetContact(
            floor.Body.WorldCenterOfMass,
            box.Body.WorldCenterOfMass,
            Fixed64.Zero,
            Vector3d.Up);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Half);
        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
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
