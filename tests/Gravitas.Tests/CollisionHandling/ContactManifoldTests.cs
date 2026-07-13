using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using System.Collections;
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

        manifold.AddContact(new Vector3d(0, 0, 0), new Vector3d(0, 0, 1), Fixed64.FromFraction(1, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(1, 0, 0), new Vector3d(1, 0, 1), Fixed64.FromFraction(2, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(2, 0, 0), new Vector3d(2, 0, 1), Fixed64.FromFraction(3, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(3, 0, 0), new Vector3d(3, 0, 1), Fixed64.FromFraction(4, 10), Vector3d.Right);
        manifold.AddContact(new Vector3d(4, 0, 0), new Vector3d(4, 0, 1), Fixed64.FromFraction(5, 10), Vector3d.Right);

        manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        manifold.LastUpdatedFrame.Should().Be(7);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(5, 10));
        manifold.Select(contact => contact.Depth)
            .Should()
            .BeEquivalentTo(new[]
            {
                Fixed64.FromFraction(2, 10),
                Fixed64.FromFraction(3, 10),
                Fixed64.FromFraction(4, 10),
                Fixed64.FromFraction(5, 10)
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
        manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 4), Vector3d.Forward);
        manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 2), Vector3d.Forward);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 2));
    }

    [Fact]
    public void NonGenericEnumerator_ShouldExposeCurrentAndSupportReset()
    {
        var manifold = new ContactManifold();
        manifold.AddContact(Vector3d.Zero, Vector3d.Forward, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(Vector3d.Right, Vector3d.Right + Vector3d.Forward, Fixed64.One, Vector3d.Forward);
        IEnumerator enumerator = ((IEnumerable)manifold).GetEnumerator();

        enumerator.MoveNext().Should().BeTrue();
        enumerator.Current.Should().Be(manifold[0]);
        enumerator.MoveNext().Should().BeTrue();
        enumerator.Current.Should().Be(manifold[1]);

        enumerator.Reset();

        enumerator.MoveNext().Should().BeTrue();
        enumerator.Current.Should().Be(manifold[0]);
    }

    [Fact]
    public void AddContact_WhenFullWithEqualDepth_ShouldReplaceHighestIdentityContact()
    {
        ContactCandidate[] candidates = Enumerable.Range(-12, 25)
            .Select(static value => CreateCandidate(value))
            .OrderBy(static candidate => candidate.ContactId)
            .ToArray();
        var manifold = new ContactManifold();

        ContactCandidate replacement = candidates[0];
        ContactCandidate keptA = candidates[1];
        ContactCandidate keptB = candidates[2];
        ContactCandidate keptC = candidates[3];
        ContactCandidate replaced = candidates[^1];

        manifold.AddContact(keptA.PointA, keptA.PointB, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(keptB.PointA, keptB.PointB, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(keptC.PointA, keptC.PointB, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(replaced.PointA, replaced.PointB, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(replacement.PointA, replacement.PointB, Fixed64.Half, Vector3d.Forward);

        manifold.Select(static contact => contact.ContactId)
            .Should()
            .Equal(new[] { replacement.ContactId, keptA.ContactId, keptB.ContactId, keptC.ContactId }.OrderBy(static id => id));
    }

    [Fact]
    public void AddContact_WhenFull_ShouldReplaceShallowestThenTieBreakByHighestIdentity()
    {
        ContactCandidate[] candidates = Enumerable.Range(-12, 25)
            .Select(static value => CreateCandidate(value))
            .OrderBy(static candidate => candidate.ContactId)
            .ToArray();
        var manifold = new ContactManifold();

        ContactCandidate lowId = candidates[0];
        ContactCandidate middleId = candidates[1];
        ContactCandidate highId = candidates[^1];
        ContactCandidate replacement = candidates[2];
        ContactCandidate filler = CreateCandidate(50);

        manifold.AddContact(highId.PointA, highId.PointB, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(middleId.PointA, middleId.PointB, Fixed64.FromFraction(1, 4), Vector3d.Forward);
        manifold.AddContact(lowId.PointA, lowId.PointB, Fixed64.FromFraction(1, 4), Vector3d.Forward);
        manifold.AddContact(filler.PointA, filler.PointB, Fixed64.Half, Vector3d.Forward);
        manifold.AddContact(replacement.PointA, replacement.PointB, Fixed64.Half, Vector3d.Forward);

        manifold.Select(static contact => contact.ContactId)
            .Should()
            .Contain(new[] { lowId.ContactId, replacement.ContactId, highId.ContactId });
        manifold.Select(static contact => contact.ContactId)
            .Should()
            .NotContain(middleId.ContactId);
    }

    private static ContactCandidate CreateCandidate(int x)
    {
        Vector3d pointA = new((Fixed64)x, Fixed64.Zero, Fixed64.Zero);
        Vector3d pointB = new((Fixed64)x, Fixed64.Zero, Fixed64.One);
        var manifold = new ContactManifold();
        manifold.AddContact(pointA, pointB, Fixed64.One, Vector3d.Forward);
        return new ContactCandidate(pointA, pointB, manifold.PrimaryContact.ContactId);
    }

    private readonly struct ContactCandidate
    {
        public ContactCandidate(Vector3d pointA, Vector3d pointB, ulong contactId)
        {
            PointA = pointA;
            PointB = pointB;
            ContactId = contactId;
        }

        public Vector3d PointA { get; }

        public Vector3d PointB { get; }

        public ulong ContactId { get; }
    }
}
