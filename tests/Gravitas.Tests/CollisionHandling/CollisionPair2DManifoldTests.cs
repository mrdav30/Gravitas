using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionPair2DManifoldTests
{
    [Fact]
    public void Simulate_WithPersistentTriggerPair_ShouldUpdatePairOwnedManifoldAcrossFrames()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D trigger = CreateCircle(context, Vector2d.Zero, immovable: false);
        StiffBody2D other = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        trigger.Collider.IsTrigger = true;

        Step(context);
        CollisionPair2D pair = GetPair(trigger, other);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;

        Step(context);
        CollisionPair2D updatedPair = GetPair(trigger, other);

        updatedPair.Should().BeSameAs(pair);
        updatedPair.LastFrame.Should().Be(context.FrameCount);
        updatedPair.Manifold.LastUpdatedFrame.Should().Be(context.FrameCount);
        updatedPair.Manifold.HasContact.Should().BeTrue();
        updatedPair.Manifold.PrimaryContact.ContactId.Should().Be(contactId);
    }

    [Fact]
    public void MarkSeparated_ShouldResetPairOwnedManifoldAndWarmStartState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D first = CreateCircle(context, Vector2d.Zero, immovable: true);
        StiffBody2D second = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        first.Collider.IsTrigger = true;
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 7).Should().BeTrue();
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        pair.StoreWarmStartImpulse(contactId, Fixed64.One, Fixed64.Half);
        pair.MarkColliding(frame: 7);

        pair.MarkSeparated();

        pair.IsColliding.Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
        pair.Manifold.LastUpdatedFrame.Should().Be(-1);
        pair.TryGetWarmStartImpulse(contactId, out _).Should().BeFalse();
    }

    [Fact]
    public void Initialize_ForReusedPairWithDifferentColliderIds_ShouldResetManifoldAndWarmStartState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D first = CreateCircle(context, Vector2d.Zero, immovable: true);
        StiffBody2D second = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        StiffBody2D third = CreateCircle(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 7).Should().BeTrue();
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        pair.StoreWarmStartImpulse(contactId, Fixed64.One, Fixed64.Half);

        pair.Initialize(first.Collider, third.Collider);

        pair.Id1.Should().Be(first.Collider.Id);
        pair.Id2.Should().Be(third.Collider.Id);
        pair.Manifold.HasContact.Should().BeFalse();
        pair.Manifold.LastUpdatedFrame.Should().Be(-1);
        pair.TryGetWarmStartImpulse(contactId, out _).Should().BeFalse();
    }

    [Fact]
    public void MarkResting_AfterSameManifoldCheck_ShouldPreserveContactIdentity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D first = CreateBox(context, Vector2d.Zero, immovable: true);
        StiffBody2D second = CreateBox(context, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 1).Should().BeTrue();
        ulong[] contactIds = pair.Manifold.Select(static contact => contact.ContactId).ToArray();
        pair.MarkColliding(frame: 1);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 2).Should().BeTrue();
        pair.MarkResting(frame: 2);

        pair.IsColliding.Should().BeTrue();
        pair.LastFrame.Should().Be(2);
        pair.Manifold.LastUpdatedFrame.Should().Be(2);
        pair.Manifold.Select(static contact => contact.ContactId).Should().Equal(contactIds);
    }

    private static CollisionPair2D GetPair(StiffBody2D first, StiffBody2D second)
    {
        if (first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair2D? firstPair) && firstPair != null)
            return firstPair;

        second.Collider.TryGetCollisionPair(first.Collider.Id, out CollisionPair2D? secondPair).Should().BeTrue();
        return secondPair!;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static StiffBody2D CreateCircle(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable)
    {
        var body = new StiffBody2D(
            new TestMatterAgent(context, CreateTransform(position)),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }

    private static StiffBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable)
    {
        var body = new StiffBody2D(
            new TestMatterAgent(context, CreateTransform(position)),
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)))
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }

    private static FixedTransform CreateTransform(Vector2d position) =>
        new(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
}
