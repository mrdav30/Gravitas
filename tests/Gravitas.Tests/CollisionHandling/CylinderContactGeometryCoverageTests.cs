using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CylinderContactGeometryCoverageTests
{
    [Fact]
    public void HorizontalParallelCylinders_WithCapOverlap_ShouldGenerateFourStableContacts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion horizontalRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)(-90));
        ScenarioBody<LSCylinderCollider> first = scenario.CreateCylinder(Vector3d.Zero, horizontalRotation);
        ScenarioBody<LSCylinderCollider> second = scenario.CreateCylinder(
            Fixed64.FromFraction(3, 4) * Vector3d.Right,
            horizontalRotation);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        Vector3d firstCapCenter = Fixed64.Half * Vector3d.Right;
        Vector3d secondCapCenter = Fixed64.FromFraction(1, 4) * Vector3d.Right;
        Vector3d[] expectedFirstPoints =
        {
            firstCapCenter + Fixed64.Half * Vector3d.Forward,
            firstCapCenter - Fixed64.Half * Vector3d.Forward,
            firstCapCenter + Fixed64.Half * Vector3d.Up,
            firstCapCenter - Fixed64.Half * Vector3d.Up
        };
        Vector3d[] expectedSecondPoints =
        {
            secondCapCenter + Fixed64.Half * Vector3d.Forward,
            secondCapCenter - Fixed64.Half * Vector3d.Forward,
            secondCapCenter + Fixed64.Half * Vector3d.Up,
            secondCapCenter - Fixed64.Half * Vector3d.Up
        };

        first.Collider.LineDirection.Should().Be(Vector3d.Right);
        second.Collider.LineDirection.Should().Be(Vector3d.Right);
        pair.CollisionType.Should().Be(CollisionType.Cylinder_Cylinder);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        AssertHorizontalCapManifold(pair.Manifold, expectedFirstPoints, expectedSecondPoints);
        ulong[] firstContactIds = pair.Manifold.Select(contact => contact.ContactId).ToArray();
        Vector3d[] firstPointOrder = pair.Manifold.Select(contact => contact.PointA).ToArray();

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        AssertHorizontalCapManifold(pair.Manifold, expectedFirstPoints, expectedSecondPoints);
        pair.Manifold.Select(contact => contact.ContactId).Should().Equal(firstContactIds);
        pair.Manifold.Select(contact => contact.PointA).Should().Equal(firstPointOrder);
    }

    private static void AssertHorizontalCapManifold(
        ContactManifold manifold,
        Vector3d[] expectedFirstPoints,
        Vector3d[] expectedSecondPoints)
    {
        manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        manifold.Select(contact => contact.ContactId).Should().BeInAscendingOrder();
        manifold.Select(contact => contact.PointA).Should().BeEquivalentTo(expectedFirstPoints);
        manifold.Select(contact => contact.PointB).Should().BeEquivalentTo(expectedSecondPoints);
        for (int i = 0; i < manifold.Count; i++)
        {
            manifold[i].Depth.Should().Be(Fixed64.FromFraction(1, 4));
            manifold[i].Normal.Should().Be(Vector3d.Right);
        }
    }
}
