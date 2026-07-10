using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactManifold2DTests
{
    [Fact]
    public void NewManifold_ShouldBeEmpty()
    {
        var manifold = new ContactManifold2D();

        manifold.Count.Should().Be(0);
        manifold.HasContact.Should().BeFalse();
        manifold.LastUpdatedFrame.Should().Be(-1);
    }

    [Fact]
    public void BeginUpdate_ShouldClearContactsAndRecordFrame()
    {
        var manifold = new ContactManifold2D();
        manifold.AddContact(new Vector2d(0, 0), new Vector2d(1, 0), Fixed64.One, Vector2d.Right);

        manifold.BeginUpdate(42);

        manifold.Count.Should().Be(0);
        manifold.HasContact.Should().BeFalse();
        manifold.LastUpdatedFrame.Should().Be(42);
    }

    [Fact]
    public void AddContact_ShouldStoreOneContact()
    {
        var manifold = new ContactManifold2D();

        manifold.AddContact(new Vector2d(0, 0), new Vector2d(1, 0), Fixed64.Half, Vector2d.Right);

        manifold.Count.Should().Be(1);
        manifold.HasContact.Should().BeTrue();
        manifold.PrimaryContact.PointA.Should().Be(new Vector2d(0, 0));
        manifold.PrimaryContact.PointB.Should().Be(new Vector2d(1, 0));
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
        manifold.PrimaryContact.Normal.Should().Be(Vector2d.Right);
        manifold.PrimaryContact.ContactId.Should().NotBe(0UL);
    }

    [Fact]
    public void AddContact_WithDuplicateIdentity_ShouldKeepDeeperContact()
    {
        var manifold = new ContactManifold2D();
        Vector2d pointA = new(Fixed64.One, Fixed64.Zero);
        Vector2d pointB = new(Fixed64.One, Fixed64.Half);

        manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 4), Vector2d.Forward);
        manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 2), Vector2d.Right);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 2));
        manifold.PrimaryContact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void AddContact_WithDuplicateIdentity_ShouldIgnoreShallowerContact()
    {
        var manifold = new ContactManifold2D();
        Vector2d pointA = new(Fixed64.One, Fixed64.Zero);
        Vector2d pointB = new(Fixed64.One, Fixed64.Half);

        manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 2), Vector2d.Right);
        manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 4), Vector2d.Forward);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 2));
        manifold.PrimaryContact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void AddContact_ShouldKeepDeepestTwoContacts()
    {
        var manifold = new ContactManifold2D();

        manifold.AddContact(new Vector2d(0, 0), new Vector2d(0, 1), Fixed64.FromFraction(1, 10), Vector2d.Forward);
        manifold.AddContact(new Vector2d(1, 0), new Vector2d(1, 1), Fixed64.FromFraction(2, 10), Vector2d.Forward);
        manifold.AddContact(new Vector2d(2, 0), new Vector2d(2, 1), Fixed64.FromFraction(3, 10), Vector2d.Forward);

        manifold.Count.Should().Be(ContactManifold2D.MaxContactCount);
        manifold.Select(contact => contact.Depth)
            .Should()
            .BeEquivalentTo(new[]
            {
                Fixed64.FromFraction(2, 10),
                Fixed64.FromFraction(3, 10)
            });
    }

    [Fact]
    public void AddContact_WhenFull_ShouldIgnoreShallowerContact()
    {
        var manifold = new ContactManifold2D();

        manifold.AddContact(new Vector2d(0, 0), new Vector2d(0, 1), Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(new Vector2d(1, 0), new Vector2d(1, 1), Fixed64.Half, Vector2d.Forward);
        ulong[] existingIds = manifold.Select(static contact => contact.ContactId).ToArray();

        manifold.AddContact(new Vector2d(2, 0), new Vector2d(2, 1), Fixed64.FromFraction(1, 4), Vector2d.Forward);

        manifold.Select(static contact => contact.ContactId).Should().Equal(existingIds);
    }

    [Fact]
    public void AddContact_WhenFullWithEqualDepth_ShouldReplaceHigherIdentityContact()
    {
        ContactCandidate[] contacts = Enumerable.Range(-8, 17)
            .Select(static value => CreateCandidate(value))
            .OrderBy(static candidate => candidate.ContactId)
            .ToArray();
        var manifold = new ContactManifold2D();
        ContactCandidate replacement = contacts[0];
        ContactCandidate shallow = contacts[2];
        ContactCandidate deep = contacts[3];

        manifold.AddContact(shallow.PointA, shallow.PointB, Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(deep.PointA, deep.PointB, Fixed64.One, Vector2d.Forward);
        manifold.AddContact(replacement.PointA, replacement.PointB, Fixed64.Half, Vector2d.Forward);

        manifold.Select(static contact => contact.ContactId)
            .Should()
            .Equal(new[] { replacement.ContactId, deep.ContactId }.OrderBy(static id => id));
    }

    [Fact]
    public void AddContact_ShouldExposeContactsByStableIdentity()
    {
        var manifold = new ContactManifold2D();

        manifold.AddContact(new Vector2d(2, 0), new Vector2d(2, 1), Fixed64.FromFraction(3, 10), Vector2d.Forward);
        manifold.AddContact(new Vector2d(0, 0), new Vector2d(0, 1), Fixed64.FromFraction(3, 10), Vector2d.Forward);

        manifold.Select(contact => contact.ContactId)
            .Should()
            .BeInAscendingOrder();
    }

    [Fact]
    public void PrimaryContact_ShouldUseDeepestDepthThenLowestContactId()
    {
        var manifold = new ContactManifold2D();
        manifold.AddContact(new Vector2d(2, 0), new Vector2d(2, 1), Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(new Vector2d(0, 0), new Vector2d(0, 1), Fixed64.Half, Vector2d.Forward);

        manifold.PrimaryContact.ContactId.Should().Be(manifold[0].ContactId);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void PrimaryContact_WhenSecondContactIsDeeper_ShouldReturnSecondContact()
    {
        ContactCandidate[] candidates = Enumerable.Range(-4, 9)
            .Select(static value => CreateCandidate(value))
            .OrderBy(static candidate => candidate.ContactId)
            .ToArray();
        var manifold = new ContactManifold2D();
        ContactCandidate first = candidates[0];
        ContactCandidate second = candidates[^1];

        manifold.AddContact(first.PointA, first.PointB, Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(second.PointA, second.PointB, Fixed64.One, Vector2d.Forward);

        manifold.PrimaryContact.ContactId.Should().Be(second.ContactId);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.One);
    }

    [Fact]
    public void SetContact_ShouldReplaceExistingContacts()
    {
        var manifold = new ContactManifold2D();
        manifold.AddContact(new Vector2d(0, 0), new Vector2d(0, 1), Fixed64.One, Vector2d.Forward);
        manifold.AddContact(new Vector2d(1, 0), new Vector2d(1, 1), Fixed64.One, Vector2d.Forward);

        manifold.SetContact(new Vector2d(2, 0), new Vector2d(2, 1), Fixed64.Half, Vector2d.Right);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.PointA.Should().Be(new Vector2d(2, 0));
        manifold.PrimaryContact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void SetContact_WithMaterialOverride_ShouldReplaceContactsAndPreserveMaterials()
    {
        var manifold = new ContactManifold2D();
        PhysicsMaterial materialA = new((Fixed64)2, Fixed64.One, Fixed64.FromFraction(1, 4));
        PhysicsMaterial materialB = new(Fixed64.Half, Fixed64.Half, Fixed64.FromFraction(1, 8));
        manifold.AddContact(new Vector2d(0, 0), new Vector2d(0, 1), Fixed64.One, Vector2d.Forward);
        manifold.AddContact(new Vector2d(1, 0), new Vector2d(1, 1), Fixed64.One, Vector2d.Forward);

        manifold.SetContact(
            new Vector2d(2, 0),
            new Vector2d(2, 1),
            Fixed64.Half,
            Vector2d.Right,
            materialA,
            materialB);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.HasMaterialOverride.Should().BeTrue();
        manifold.PrimaryContact.MaterialA.Should().Be(materialA);
        manifold.PrimaryContact.MaterialB.Should().Be(materialB);
    }

    [Fact]
    public void Reset_ShouldClearContactsAndFrame()
    {
        var manifold = new ContactManifold2D();
        manifold.BeginUpdate(5);
        manifold.AddContact(new Vector2d(0, 0), new Vector2d(1, 0), Fixed64.One, Vector2d.Right);

        manifold.Reset();

        manifold.Count.Should().Be(0);
        manifold.HasContact.Should().BeFalse();
        manifold.LastUpdatedFrame.Should().Be(-1);
    }

    private static ContactCandidate CreateCandidate(int x)
    {
        Vector2d pointA = new((Fixed64)x, Fixed64.Zero);
        Vector2d pointB = new((Fixed64)x, Fixed64.One);
        var manifold = new ContactManifold2D();
        manifold.AddContact(pointA, pointB, Fixed64.One, Vector2d.Forward);
        return new ContactCandidate(pointA, pointB, manifold.PrimaryContact.ContactId);
    }

    private readonly struct ContactCandidate
    {
        public ContactCandidate(Vector2d pointA, Vector2d pointB, ulong contactId)
        {
            PointA = pointA;
            PointB = pointB;
            ContactId = contactId;
        }

        public Vector2d PointA { get; }

        public Vector2d PointB { get; }

        public ulong ContactId { get; }
    }
}
