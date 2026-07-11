using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
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
    public void CompoundSphere_WithAllPartsSeparated_ShouldReturnFalseWithoutContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(compound.Collider, sphere.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();

        pair.Manifold.HasContact.Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    [Fact]
    public void SphereCompound_ShouldReportContactInSphereOwnerOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            CreateTwoSphereCompound(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            preventAngularForces: true);
        CollisionPair pair = scenario.CreatePair(sphere.Collider, compound.Collider);

        pair.CollisionType.Should().Be(CollisionType.Compound);
        pair.UpdateCollision();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Right);
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
    public void CompoundCompound_ShouldDetectMatchingPartsInOwnerOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> first = scenario.CreateBody(
            CreateTwoSphereCompound(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            preventAngularForces: true);
        ScenarioBody<LSCompoundCollider> second = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero))),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            preventAngularForces: true);
        CollisionPair forward = scenario.CreatePair(first.Collider, second.Collider);
        CollisionPair reversed = scenario.CreatePair(second.Collider, first.Collider);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();

        forward.CollisionType.Should().Be(CollisionType.Compound);
        reversed.CollisionType.Should().Be(CollisionType.Compound);
        forward.Manifold.Count.Should().Be(1);
        reversed.Manifold.Count.Should().Be(1);
        forward.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        forward.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Right);
        reversed.Manifold.PrimaryContact.Depth.Should().Be(forward.Manifold.PrimaryContact.Depth);
        reversed.Manifold.PrimaryContact.Normal.Should().Be(-Vector3d.Right);
        reversed.Manifold.PrimaryContact.PointA.Should().Be(forward.Manifold.PrimaryContact.PointB);
        reversed.Manifold.PrimaryContact.PointB.Should().Be(forward.Manifold.PrimaryContact.PointA);
    }

    [Fact]
    public void CompoundCompound_WithSeparatedParts_ShouldReturnFalseWithoutContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> first = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCompoundCollider> second = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero))),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();

        pair.Manifold.HasContact.Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    [Fact]
    public void CompoundSpherePart_WithOverlappingCuboidBoundsButSeparatedCorner_ShouldRejectNarrowPhase()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.FromFraction(9, 10)));
        CollisionPair pair = scenario.CreatePair(compound.Collider, cuboid.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();

        pair.Manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CompoundSpherePart_AgainstHigherPriorityCuboid_ShouldPreserveOwnerContactOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        PhysicsMaterial partMaterial = PhysicsMaterial.Frictionless;
        PhysicsMaterial cuboidMaterial = PhysicsMaterial.Default;
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, partMaterial)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        cuboid.Collider.Material = cuboidMaterial;
        CollisionPair pair = scenario.CreatePair(compound.Collider, cuboid.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.PrimaryContact.MaterialA.Should().Be(partMaterial);
        pair.Manifold.PrimaryContact.MaterialB.Should().Be(cuboidMaterial);
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
