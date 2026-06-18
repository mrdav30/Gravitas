using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CompoundColliderCollisionTests
{
    [Fact]
    public void CompoundSphere_ShouldDetectContactThroughMatchingPart()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            CreateTwoSphereCompound(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(compound.Collider, sphere.Collider);

        pair.CollisionType.Should().Be(CollisionType.Compound);
        pair.UpdateCollision();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CompoundInternalOverlap_ShouldReduceDuplicateContactsDeterministically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compoundCollider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            compoundCollider,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(compound.Collider, sphere.Collider);

        pair.UpdateCollision();
        ulong firstContactId = pair.Manifold.PrimaryContact.ContactId;

        pair.UpdateCollision();

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.ContactId.Should().Be(firstContactId);
        pair.Manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
    }

    [Fact]
    public void CompoundCollision_ShouldNotifyOwningColliderOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compoundCollider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            compoundCollider,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        int contactEnterCount = 0;
        compound.Collider.OnContactEnter += _ => contactEnterCount++;
        CollisionPair pair = scenario.CreatePair(compound.Collider, sphere.Collider);

        pair.UpdateCollision();
        pair.NotifyCollidersOfContact();

        contactEnterCount.Should().Be(1);
    }

    [Fact]
    public void CompoundCollider_ShouldRespectParentChildFilteringAsOneCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            CreateTwoSphereCompound(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));

        child.Collider.SetParent(compound.Collider);

        scenario.Context.Physics.GetCollisionPair(compound.Collider.Id, child.Collider.Id)
            .Should().BeNull();
    }

    private static LSCompoundCollider CreateTwoSphereCompound()
    {
        return new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero)));
    }
}
