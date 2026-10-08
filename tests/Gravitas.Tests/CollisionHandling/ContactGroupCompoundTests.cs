using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactGroupCompoundTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NineSpherePartsAgainstCuboid_ShouldPreserveEveryOwnerMaterialThroughPartReversal(bool reverseArguments)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = scenario.CreateBody(CreateNineSphereCompound(), Vector3d.Zero,
            FixedQuaternion.Identity, immovable: true);
        var cuboid = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(1, 1, 18) },
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, immovable: true);
        cuboid.Collider.Material = PhysicsMaterial.Bouncy;
        CollisionPair pair = reverseArguments
            ? scenario.CreatePair(cuboid.Collider, compound.Collider)
            : scenario.CreatePair(compound.Collider, cuboid.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        pair.ColliderA.Should().BeSameAs(compound.Collider);
        pair.Manifold.Count.Should().Be(9);
        pair.Manifold.GroupCount.Should().Be(9);
        for (int index = 0; index < 9; index++)
        {
            ManifoldContact contact = pair.Manifold[index];
            contact.FeatureNamespaceA.Should().Be(index + 1);
            contact.FeatureNamespaceB.Should().Be(0);
            contact.MaterialA.Should().Be(PartMaterial(index));
            contact.MaterialB.Should().Be(PhysicsMaterial.Bouncy);
            contact.Normal.Should().Be(Vector3d.Right);
            contact.Depth.Should().Be(Fixed64.Quarter);
            contact.PointA.X.Should().Be(Fixed64.Half);
            contact.PointB.X.Should().Be(Fixed64.Quarter);
            pair.Manifold.GetGroupContactCount(index).Should().Be(1);
            pair.Manifold.GetGroup(index).Key.NamespaceA.Should().Be(index + 1);
            pair.Manifold.GetGroup(index).Key.NamespaceB.Should().Be(0);
        }
    }

    [Fact]
    public void NineMatchingCompoundParts_ShouldReverseAnchorsNormalsAndMaterialsWithoutDroppingGroups()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateBody(CreateNineSphereCompound(), Vector3d.Zero,
            FixedQuaternion.Identity, immovable: true);
        var second = scenario.CreateBody(CreateNineSphereCompound(PhysicsMaterial.Bouncy),
            Vector3d.Right * Fixed64.Half, FixedQuaternion.Identity, immovable: true);
        CollisionPair forward = scenario.CreatePair(first.Collider, second.Collider);
        CollisionPair reversed = scenario.CreatePair(second.Collider, first.Collider);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();

        forward.Manifold.Count.Should().Be(9);
        reversed.Manifold.Count.Should().Be(9);
        forward.Manifold.GroupCount.Should().Be(9);
        reversed.Manifold.GroupCount.Should().Be(9);
        for (int index = 0; index < 9; index++)
        {
            ManifoldContact contact = forward.Manifold[index];
            ManifoldContact reverseContact = reversed.Manifold[index];
            contact.FeatureNamespaceA.Should().Be(index + 1);
            contact.FeatureNamespaceB.Should().Be(-(index + 1));
            contact.MaterialA.Should().Be(PartMaterial(index));
            contact.MaterialB.Should().Be(PhysicsMaterial.Bouncy);
            contact.Normal.Should().Be(Vector3d.Right);
            contact.Depth.Should().Be(Fixed64.Half);
            reverseContact.PointA.Should().Be(contact.PointB);
            reverseContact.PointB.Should().Be(contact.PointA);
            reverseContact.Normal.Should().Be(-contact.Normal);
            reverseContact.Depth.Should().Be(contact.Depth);
            reverseContact.MaterialA.Should().Be(contact.MaterialB);
            reverseContact.MaterialB.Should().Be(contact.MaterialA);
            ContactGroupKey forwardKey = forward.Manifold.GetGroup(index).Key;
            ContactGroupKey reverseKey = reversed.Manifold.GetGroup(index).Key;
            forwardKey.NamespaceA.Should().Be(index + 1);
            forwardKey.NamespaceB.Should().Be(-(index + 1));
            reverseKey.NamespaceA.Should().Be(index + 1);
            reverseKey.NamespaceB.Should().Be(-(index + 1));
        }
        forward.Manifold.Select(contact => contact.ContactId).Should().OnlyHaveUniqueItems();
        reversed.Manifold.Select(contact => contact.ContactId).Should().OnlyHaveUniqueItems();
    }

    [Fact]
    public void NineCompoundContacts_ShouldNotifyEachOwnerOncePerPairAndNeverNotifyInternalParts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateBody(CreateNineSphereCompound(), Vector3d.Zero,
            FixedQuaternion.Identity, immovable: true);
        var second = scenario.CreateBody(CreateNineSphereCompound(), Vector3d.Right * Fixed64.Half,
            FixedQuaternion.Identity, immovable: true);
        int firstEnter = 0, secondEnter = 0, firstContact = 0, secondContact = 0;
        int firstExit = 0, secondExit = 0, partContact = 0;
        first.Collider.OnContactEnter += body => { body.Should().BeSameAs(second.Body); firstEnter++; };
        second.Collider.OnContactEnter += body => { body.Should().BeSameAs(first.Body); secondEnter++; };
        first.Collider.OnContact += _ => firstContact++;
        second.Collider.OnContact += _ => secondContact++;
        first.Collider.OnContactExit += _ => firstExit++;
        second.Collider.OnContactExit += _ => secondExit++;
        for (int index = 0; index < 9; index++)
        {
            first.Collider.GetPartCollider(index).OnContact += _ => partContact++;
            second.Collider.GetPartCollider(index).OnContact += _ => partContact++;
        }
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        pair.UpdateCollision();
        pair.NotifyCollidersOfContact();
        pair.Manifold.GroupCount.Should().Be(9);
        pair.UpdateCollision();
        pair.NotifyCollidersOfContact();
        pair.Deactivate();

        firstEnter.Should().Be(1);
        secondEnter.Should().Be(1);
        firstContact.Should().Be(2);
        secondContact.Should().Be(2);
        firstExit.Should().Be(1);
        secondExit.Should().Be(1);
        partContact.Should().Be(0);
    }

    private static PhysicsMaterial PartMaterial(int index) =>
        new((Fixed64)(index + 1), Fixed64.Half, Fixed64.Zero);

    private static LSCompoundCollider CreateNineSphereCompound(PhysicsMaterial? material = null) =>
        new(Enumerable.Range(0, 9).Select(index => CompoundColliderPart.Sphere(
            Fixed64.Half, new Vector3d(0, 0, index * 2 - 8), material ?? PartMaterial(index))).ToArray());
}
