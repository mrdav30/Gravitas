using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System.Collections;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactManifold2DTests
{
    [Theory]
    [InlineData("")] [InlineData("EquivalentFrame")]
    [InlineData("Count")] [InlineData("Groups")] [InlineData("GroupKey")]
    [InlineData("Id")] [InlineData("FeatureA")] [InlineData("FeatureB")]
    [InlineData("Normal")] [InlineData("Depth")] [InlineData("Clamped")]
    [InlineData("Override")] [InlineData("MaterialA")] [InlineData("MaterialB")]
    [InlineData("PointA.X")] [InlineData("PointA.Y")]
    [InlineData("PointB.X")] [InlineData("PointB.Y")]
    [InlineData("Recovery")]
    public void ContactEquality_ShouldCompareTheCompletePhysicalConstraintAcrossAnchorFrames(string change)
    {
        var first = Build(""); var second = Build(change);
        bool equal = change is "" or "EquivalentFrame";
        Assert.Equal(equal, first.HasSameContacts(second));
        Assert.Equal(equal, second.HasSameContacts(first));

        static ContactManifold2D Build(string change)
        {
            var manifold = new ContactManifold2D();
            manifold.NeedsSurfaceRecovery = change == "Recovery";
            ContactAnchor2D a = Point("PointA"), b = Point("PointB");
            var row = new ManifoldContact2D(change == "Id" ? 2UL : 1UL, a, b,
                change == "Depth" ? Fixed64.Two : Fixed64.One,
                change == "Normal" ? Vector2d.Right : Vector2d.Forward, change != "Override",
                change == "MaterialA" ? PhysicsMaterial.Bouncy : PhysicsMaterial.Frictionless,
                change == "MaterialB" ? PhysicsMaterial.Bouncy : PhysicsMaterial.Frictionless,
                change == "Clamped", change == "FeatureA" ? 1 : 0, change == "FeatureB" ? 1 : 0);
            manifold.AddContact(new ContactGroupKey(change == "GroupKey" ? 1 : 0, 0), row);
            if (change == "Count") manifold.AddContact(default, new ManifoldContact2D(3, Vector2d.One, Vector2d.One, Fixed64.One, Vector2d.Forward));
            if (change == "Groups") manifold.AddContact(new ContactGroupKey(2, 0), row);
            return manifold;

            ContactAnchor2D Point(string prefix)
            {
                if (change == "EquivalentFrame") return new ContactAnchor2D(Vector2d.One, -Vector2d.One);
                Vector2d point = change == prefix + ".X" ? Vector2d.Right * Fixed64.Epsilon
                    : change == prefix + ".Y" ? Vector2d.Forward * Fixed64.Epsilon : Vector2d.Zero;
                return ContactAnchor2D.FromWorldPoint(point);
            }
        }
    }

    [Fact]
    public void SetContact_WithCanonicalAnchors_ShouldReplaceExistingContacts()
    {
        var manifold = new ContactManifold2D();
        manifold.AddContact(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Right);
        var anchorA = new ContactAnchor2D(
            Vector2d.One,
            Fixed64.PiOver4,
            Vector2d.Right);
        var anchorB = ContactAnchor2D.FromWorldPoint(Vector2d.Zero);

        manifold.SetContact(
            anchorA,
            anchorB,
            Fixed64.Half,
            Vector2d.Forward,
            depthIsClamped: true);

        manifold.Count.Should().Be(1);
        ManifoldContact2D contact = manifold.PrimaryContact;
        contact.AnchorA.Should().Be(anchorA);
        contact.AnchorB.Should().Be(anchorB);
        contact.Depth.Should().Be(Fixed64.Half);
        contact.DepthIsClamped.Should().BeTrue();
    }

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
    public void AddContact_WithEqualScalarDepth_ShouldPreferConceptuallyClampedContact()
    {
        var manifold = new ContactManifold2D();
        ContactAnchor2D anchorA = ContactAnchor2D.FromWorldPoint(Vector2d.Zero);
        ContactAnchor2D anchorB = ContactAnchor2D.FromWorldPoint(Vector2d.Forward);

        manifold.AddContact(anchorA, anchorB, Fixed64.MaxValue, Vector2d.Forward);
        manifold.AddContact(
            anchorA,
            anchorB,
            Fixed64.MaxValue,
            Vector2d.Right,
            depthIsClamped: true);

        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.DepthIsClamped.Should().BeTrue();
        manifold.PrimaryContact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void RelativeAnchors_ShouldKeepIdentityUnderRigidTranslationWithoutRequiringWorldPoints()
    {
        var first = new ContactManifold2D();
        var translated = new ContactManifold2D();
        Vector2d firstOffset = Vector2d.Right;
        Vector2d secondOffset = Vector2d.Left;
        first.AddContact(
            new ContactAnchor2D(new Vector2d(Fixed64.MaxValue, Fixed64.Zero), firstOffset),
            new ContactAnchor2D(new Vector2d(Fixed64.MaxValue, Fixed64.One), secondOffset),
            Fixed64.Half,
            Vector2d.Forward);
        translated.AddContact(
            new ContactAnchor2D(new Vector2d(Fixed64.MinValue, Fixed64.Zero), firstOffset),
            new ContactAnchor2D(new Vector2d(Fixed64.MinValue, Fixed64.One), secondOffset),
            Fixed64.Half,
            Vector2d.Forward);

        ManifoldContact2D contact = first.PrimaryContact;
        contact.ContactId.Should().Be(translated.PrimaryContact.ContactId);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.AnchorA.Offset.Should().Be(firstOffset);
    }

    [Fact]
    public void CanonicalAnchors_ShouldKeepIdentityAcrossFrameRotation()
    {
        var first = new ContactManifold2D();
        var rotated = new ContactManifold2D();
        var differentFeature = new ContactManifold2D();
        Vector2d localPointA = new(Fixed64.One, Fixed64.Two);
        Vector2d localPointB = new(-Fixed64.One, Fixed64.Half);
        Vector2d displacementA = new(Fixed64.Half, -Fixed64.Half);
        Vector2d displacementB = new(-Fixed64.Half, Fixed64.One);

        first.AddContact(
            new ContactAnchor2D(
                Vector2d.Zero,
                Fixed64.Zero,
                localPointA,
                displacementA),
            new ContactAnchor2D(
                Vector2d.One,
                Fixed64.Zero,
                localPointB,
                displacementB),
            Fixed64.Half,
            Vector2d.Right);
        rotated.AddContact(
            new ContactAnchor2D(
                Vector2d.Forward,
                Fixed64.PiOver4,
                localPointA,
                displacementA),
            new ContactAnchor2D(
                -Vector2d.One,
                -Fixed64.PiOver4,
                localPointB,
                displacementB),
            Fixed64.Half,
            Vector2d.Forward);
        differentFeature.AddContact(
            new ContactAnchor2D(
                Vector2d.Zero,
                Fixed64.Zero,
                localPointA,
                displacementA + Vector2d.Right),
            new ContactAnchor2D(
                Vector2d.One,
                Fixed64.Zero,
                localPointB,
                displacementB),
            Fixed64.Half,
            Vector2d.Right);

        rotated.PrimaryContact.ContactId.Should().Be(first.PrimaryContact.ContactId);
        differentFeature.PrimaryContact.ContactId.Should().NotBe(
            first.PrimaryContact.ContactId);
    }

    [Fact]
    public void CompoundPartNamespaces_ShouldDistinguishPartsAndPreserveOwnerOrder()
    {
        var manifold = new ContactManifold2D();
        ContactAnchor2D anchorA = new(
            Vector2d.Zero,
            Vector2d.Right);
        ContactAnchor2D anchorB = new(
            Vector2d.Forward,
            Vector2d.Left);

        manifold.AddContact(
            anchorA,
            anchorB,
            Fixed64.Half,
            Vector2d.Forward,
            PhysicsMaterial.Default,
            PhysicsMaterial.Default,
            featureNamespaceA: 1,
            featureNamespaceB: -1);
        manifold.AddContact(
            anchorA,
            anchorB,
            Fixed64.One,
            Vector2d.Forward,
            PhysicsMaterial.Default,
            PhysicsMaterial.Default,
            featureNamespaceA: 1,
            featureNamespaceB: -1);
        manifold.AddContact(
            anchorB,
            anchorA,
            Fixed64.Half,
            Vector2d.Left,
            PhysicsMaterial.Default,
            PhysicsMaterial.Default,
            featureNamespaceA: -2,
            featureNamespaceB: 1);

        manifold.Count.Should().Be(2);
        manifold.Select(static contact => contact.ContactId)
            .Should()
            .OnlyHaveUniqueItems();
        manifold.Single(contact =>
                contact.FeatureNamespaceA == 1
                && contact.FeatureNamespaceB == -1)
            .Depth.Should().Be(Fixed64.One);
        manifold.Single(contact => contact.FeatureNamespaceA == -2)
            .FeatureNamespaceB.Should().Be(1);
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
    public void AddContact_WhenFullWithEqualDepth_ShouldReplaceHighestIdentityContact()
    {
        ContactCandidate[] contacts = Enumerable.Range(-8, 17)
            .Select(static value => CreateCandidate(value))
            .OrderBy(static candidate => candidate.ContactId)
            .ToArray();
        var manifold = new ContactManifold2D();
        ContactCandidate replacement = contacts[0];
        ContactCandidate kept = contacts[2];
        ContactCandidate replaced = contacts[3];

        manifold.AddContact(kept.PointA, kept.PointB, Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(replaced.PointA, replaced.PointB, Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(replacement.PointA, replacement.PointB, Fixed64.Half, Vector2d.Forward);

        manifold.Select(static contact => contact.ContactId)
            .Should()
            .Equal(new[] { replacement.ContactId, kept.ContactId }.OrderBy(static id => id));
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
    public void NonGenericEnumerator_ShouldExposeCurrentAndSupportReset()
    {
        var manifold = new ContactManifold2D();
        manifold.AddContact(Vector2d.Zero, Vector2d.Forward, Fixed64.Half, Vector2d.Forward);
        manifold.AddContact(Vector2d.Right, Vector2d.Right + Vector2d.Forward, Fixed64.One, Vector2d.Forward);
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

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentGroups_ShouldKeepTwoSamplesEachInStructuralOrder(bool reverse)
    {
        var manifold = new ContactManifold2D();
        manifold.BeginUpdate(42);
        var material = new PhysicsMaterial(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        for (int g = 0; g < 3; g++)
        {
            int group = reverse ? 2 - g : g;
            for (int sample = 0; sample < 3; sample++)
            {
                var contact = new ManifoldContact2D((ulong)(10 + sample),
                    Vector2d.Zero, Vector2d.Right, (Fixed64)(sample + 1),
                    group == 1 ? Vector2d.Forward : Vector2d.Right,
                    hasMaterialOverride: true, materialA: material,
                    materialB: PhysicsMaterial.Default);
                manifold.AddContact(new ContactGroupKey(0, 0, region: group), contact);
            }
        }

        manifold.GroupCount.Should().Be(3);
        manifold.Count.Should().Be(6);
        for (int g = 0; g < 3; g++)
        {
            manifold.GetGroupStartIndex(g).Should().Be(g * 2);
            manifold.GetGroupContactCount(g).Should().Be(2);
            manifold.GetGroup(g).Key.Region.Should().Be(g);
            manifold[g * 2].ContactId.Should().Be(11UL);
            manifold[g * 2 + 1].ContactId.Should().Be(12UL);
            manifold[g * 2].MaterialA.Should().Be(material);
            manifold[g * 2].Normal.Should().Be(g == 1 ? Vector2d.Forward : Vector2d.Right);
        }
        manifold.PrimaryContact.Depth.Should().Be((Fixed64)3);
        manifold.Select(c => c.ContactId).Should().Equal(11UL, 12UL, 11UL, 12UL, 11UL, 12UL);

        var copy = new ContactManifold2D();
        copy.CopyFrom(manifold, 43);
        copy.CopyFrom(copy, 44);
        copy.LastUpdatedFrame.Should().Be(43);
        copy.Select(c => c.ContactId).Should().Equal(manifold.Select(c => c.ContactId));
        copy.Reset();
        copy.GroupCount.Should().Be(0);
        copy.AddContact(Vector2d.Zero, Vector2d.Right, Fixed64.One, Vector2d.Right);
        copy.GroupCount.Should().Be(1);
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
