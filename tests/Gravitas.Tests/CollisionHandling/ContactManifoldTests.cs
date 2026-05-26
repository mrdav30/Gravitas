using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactManifoldTests
{
    [Fact]
    public void AddContact_ShouldKeepDeepestFourContactsAndExposeStableOrder()
    {
        var manifold = new ContactManifold();
        manifold.BeginUpdate(7);

        manifold.AddContact(new Vector3d(0, 0, 0), new Vector3d(0, 0, 1), Fixed64.Fraction(1, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(1, 0, 0), new Vector3d(1, 0, 1), Fixed64.Fraction(2, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(2, 0, 0), new Vector3d(2, 0, 1), Fixed64.Fraction(3, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(3, 0, 0), new Vector3d(3, 0, 1), Fixed64.Fraction(4, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(4, 0, 0), new Vector3d(4, 0, 1), Fixed64.Fraction(5, 10), Vector3d.Right);

        manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        manifold.LastUpdatedFrame.Should().Be(7);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.Fraction(5, 10));
        manifold.Select(contact => contact.Depth)
            .Should()
            .BeEquivalentTo(new[]
            {
                Fixed64.Fraction(2, 10),
                Fixed64.Fraction(3, 10),
                Fixed64.Fraction(4, 10),
                Fixed64.Fraction(5, 10)
            });
        manifold.Select(contact => contact.ContactId)
            .Should()
            .BeInAscendingOrder();
    }

    [Fact]
    public void AddContact_ShouldIgnoreDuplicateContactIdentity()
    {
        var manifold = new ContactManifold();
        manifold.BeginUpdate(3);

        Vector3d pointA = new(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Vector3d pointB = new(Fixed64.One, Fixed64.Zero, Fixed64.Half);
        manifold.AddContact(pointA, pointB, Fixed64.Fraction(1, 4), Vector3d.Forward);
        manifold.AddContact(pointA, pointB, Fixed64.Fraction(1, 2), Vector3d.Forward);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.Fraction(1, 2));
    }
}
