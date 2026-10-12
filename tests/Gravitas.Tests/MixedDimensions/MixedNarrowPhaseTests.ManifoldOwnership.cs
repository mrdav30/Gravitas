using Chronicler.Hashing;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Verifies retained mixed rows, compound provenance and pair lifetime ownership.</content>
public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(false, true, 2)]
    [InlineData(true, false, 2)]
    [InlineData(true, true, 4)]
    public void MixedBuilder_ShouldRetainEveryCompoundLeafPair(bool compound3D, bool compound2D, int expected)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider first = compound3D ? CompoundForMixedManifold() : new LSSphereCollider();
        LSCollider2D second = compound2D ? new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Left * Fixed64.Quarter, PhysicsMaterial.Bouncy),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * Fixed64.Quarter, PhysicsMaterial.Frictionless))
            : new LSCircleCollider2D(Fixed64.Half);
        CreateBody3D(context, first, Vector3d.Zero, FixedQuaternion.Identity);
        CreateBody2D(context, second, Vector2d.Zero);
        var manifold = new ContactManifold();

        CollisionDetectionMixed.TryBuildManifold(first, second, manifold).Should().BeTrue();

        manifold.Count.Should().Be(expected);
        manifold.GroupCount.Should().Be(expected);
        for (int i = 0; i < manifold.GroupCount; i++)
        {
            ref ContactGroup group = ref manifold.GetGroup(i);
            ManifoldContact row = group[0];
            group.Count.Should().Be(1);
            row.FeatureNamespaceA.Should().Be(compound3D ? group.Key.NamespaceA : 0);
            row.FeatureNamespaceB.Should().Be(compound2D ? group.Key.NamespaceB : 0);
            row.HasMaterialOverride.Should().BeTrue();
            row.MaterialA.Should().Be(compound3D && row.FeatureNamespaceA == 1
                ? PhysicsMaterial.Frictionless : PhysicsMaterial.Default);
            row.MaterialB.Should().Be(compound2D && row.FeatureNamespaceB == -1
                ? PhysicsMaterial.Bouncy : compound2D ? PhysicsMaterial.Frictionless : PhysicsMaterial.Default);
        }
        CollisionDetectionMixed.TryCollide(first, second, out MixedContact summary).Should().BeTrue();
        summary.Should().Be(MixedContact.FromManifoldContact(manifold.PrimaryContact));
    }

    [Fact]
    public void MixedBuilder_ShouldClearRowsOnSeparationAndValidateCallerBuffer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider first = CreateBody3D(context, new LSSphereCollider(), Vector3d.Zero, FixedQuaternion.Identity).Collider;
        SolidBody2D second = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(first, second.Collider, manifold).Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.HasMaterialOverride.Should().BeFalse();
        second.SetPosition(Vector2d.Right * (Fixed64)6);
        CollisionDetectionMixed.TryBuildManifold(first, second.Collider, manifold).Should().BeFalse();
        manifold.Count.Should().Be(0);
        Action missing3D = () => CollisionDetectionMixed.TryBuildManifold(null!, second.Collider, manifold);
        Action missing2D = () => CollisionDetectionMixed.TryBuildManifold(first, null!, manifold);
        Action missingBuffer = () => CollisionDetectionMixed.TryBuildManifold(first, second.Collider, null!);
        missing3D.Should().Throw<ArgumentNullException>().WithParameterName("collider3D");
        missing2D.Should().Throw<ArgumentNullException>().WithParameterName("collider2D");
        missingBuffer.Should().Throw<ArgumentNullException>().WithParameterName("manifold");
    }

    [Fact]
    public void MixedPair_ShouldPublishAllRowsBeforeCallbacksAndOwnItsCopyAndStepImpulses()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider first = CreateBody3D(context, CompoundForMixedManifold(), Vector3d.Zero, FixedQuaternion.Identity).Collider;
        LSCollider2D second = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider;
        var source = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(first, second, source).Should().BeTrue();
        var pair = new CollisionPairMixed(first, second);
        int entered = 0;
        first.OnMixedContactEnter += _ => { pair.Manifold.Count.Should().Be(2); entered++; };
        pair.MarkColliding(1, source);
        entered.Should().Be(1);
        source.Reset();
        pair.Manifold.Count.Should().Be(2);
        pair.Manifold.LastUpdatedFrame.Should().Be(1);
        pair.MarkResting(1, pair.Manifold);
        pair.Manifold.Count.Should().Be(2);
        Action missingRows = () => pair.MarkResting(1, (ContactManifold)null!);
        missingRows.Should().Throw<ArgumentNullException>();
        pair.SolveImpulses.Set(pair.Manifold[0].ContactId, Vector3d.Right, Fixed64.One, Fixed64.Zero);
        pair.SolveImpulses.Count.Should().Be(1);
        pair.MarkResting(2);
        pair.SolveImpulses.Count.Should().Be(0);
        pair.Manifold.Count.Should().Be(2);
        pair.MarkResting(2, source);
        pair.Manifold.HasContact.Should().BeFalse();
        pair.Contact.HasContact.Should().BeFalse();
        pair.MarkSeparated();
        pair.Manifold.Count.Should().Be(0);
        pair.Contact.HasContact.Should().BeFalse();
        pair.SolveImpulses.Set(1, Vector3d.Right, Fixed64.One, Fixed64.Zero);
        pair.Initialize(first, second);
        pair.SolveImpulses.Count.Should().Be(0);
        pair.Manifold.LastUpdatedFrame.Should().Be(-1);
    }

    [Fact]
    public void MixedContactFallbackMaterials_ShouldPreserveExplicitLeafMaterials()
    {
        var contact = new MixedContact(Vector3d.Zero, Vector3d.Up, Vector3d.Up, Fixed64.One,
            PhysicsMaterial.Bouncy, PhysicsMaterial.Frictionless);
        MixedContact resolved = contact.WithFallbackMaterials(PhysicsMaterial.Default, PhysicsMaterial.Default);
        resolved.Material3D.Should().Be(PhysicsMaterial.Bouncy);
        resolved.Material2D.Should().Be(PhysicsMaterial.Frictionless);
        resolved.Anchor3D.Should().Be(contact.Anchor3D);
        resolved.Anchor2D.Should().Be(contact.Anchor2D);
        resolved.Depth.Should().Be(contact.Depth);
    }

    [Fact]
    public void MixedPair_ReentrantSeparation_ShouldClearAllRowsBeforeExitCallbacks()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider first = CreateBody3D(context, CompoundForMixedManifold(), Vector3d.Zero, FixedQuaternion.Identity).Collider;
        LSCollider2D second = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(first, second, manifold).Should().BeTrue();
        var pair = new CollisionPairMixed(first, second);
        first.OnMixedContactEnter += _ => pair.MarkSeparated();
        int exited = 0;
        first.OnMixedContactExit += _ => { pair.Manifold.Count.Should().Be(0); exited++; };
        pair.MarkColliding(1, manifold);
        exited.Should().Be(1);
        pair.Manifold.Count.Should().Be(0);
    }

    [Fact]
    public void MixedPairReplayHash_ShouldIncludeNonprimaryRows()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider first = CreateBody3D(context, new LSSphereCollider(), Vector3d.Zero, FixedQuaternion.Identity).Collider;
        LSCollider2D second = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider;
        var manifold = new ContactManifold();
        manifold.AddContact(Vector3d.Zero, Vector3d.Right, Fixed64.One, Vector3d.Right);
        var pair = new CollisionPairMixed(first, second);
        pair.MarkResting(1, manifold);
        var before = new ChronicleHashWriter();
        pair.ContributeReplayHash(ref before, GravitasReplayHashMode.Authoritative);
        manifold.AddContact(Vector3d.Up, Vector3d.Forward, Fixed64.Half, Vector3d.Forward);
        pair.MarkResting(1, manifold);
        pair.Contact.Depth.Should().Be(Fixed64.One);
        var after = new ChronicleHashWriter();
        pair.ContributeReplayHash(ref after, GravitasReplayHashMode.Authoritative);
        after.ToHash().Should().NotBe(before.ToHash());
    }

    private static LSCompoundCollider CompoundForMixedManifold() => new(
        CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Left * Fixed64.Quarter, PhysicsMaterial.Frictionless),
        CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right * Fixed64.Quarter));
}
