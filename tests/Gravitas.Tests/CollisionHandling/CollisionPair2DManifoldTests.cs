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
        LSCircleCollider2D trigger = CreateBodylessCircle(context, Vector2d.Zero);
        SolidBody2D other = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        trigger.IsTrigger = true;

        Step(context);
        CollisionPair2D pair = GetPair(trigger, other.Collider);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;

        Step(context);
        CollisionPair2D updatedPair = GetPair(trigger, other.Collider);

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
        LSCircleCollider2D first = CreateBodylessCircle(context, Vector2d.Zero);
        SolidBody2D second = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        first.IsTrigger = true;
        var pair = new CollisionPair2D(first, second.Collider);
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
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: true);
        SolidBody2D second = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        SolidBody2D third = CreateCircle(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
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
        SolidBody2D first = CreateBox(context, Vector2d.Zero, immovable: true);
        SolidBody2D second = CreateBox(context, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero), immovable: true);
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

    [Fact]
    public void PairTransitions_WithoutContactOrPriorCollision_ShouldRemainSilent()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(context, new Vector2d((Fixed64)4, Fixed64.Zero), immovable: false);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        int notifications = 0;
        first.Collider.OnContactEnter += _ => notifications++;
        first.Collider.OnContact += _ => notifications++;
        first.Collider.OnContactExit += _ => notifications++;
        second.Collider.OnContactEnter += _ => notifications++;
        second.Collider.OnContact += _ => notifications++;
        second.Collider.OnContactExit += _ => notifications++;

        pair.MarkColliding(frame: 1);
        pair.MarkCollidingDeferred(frame: 2);
        pair.MarkSeparated();

        pair.IsColliding.Should().BeFalse();
        pair.LastFrame.Should().Be(-1);
        pair.Manifold.HasContact.Should().BeFalse();
        first.Position.Should().Be(Vector2d.Zero);
        second.Position.Should().Be(new Vector2d((Fixed64)4, Fixed64.Zero));
        first.LinearVelocity.Should().Be(Vector2d.Zero);
        second.LinearVelocity.Should().Be(Vector2d.Zero);
        notifications.Should().Be(0);
    }

    [Fact]
    public void WakeSleepingBodiesForCollision_WithEitherBodyMissing_ShouldNotWakeTheBoundBody()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCircleCollider2D bodylessFirst = CreateBodylessCircle(context, Vector2d.Zero);
        SolidBody2D second = CreateCircle(context, Vector2d.Zero, immovable: false);
        second.Sleep();
        var firstPair = new CollisionPair2D(bodylessFirst, second.Collider);

        SolidBody2D first = CreateCircle(context, new Vector2d((Fixed64)4, Fixed64.Zero), immovable: false);
        LSCircleCollider2D bodylessSecond = CreateBodylessCircle(
            context,
            new Vector2d((Fixed64)4, Fixed64.Zero));
        first.Sleep();
        var secondPair = new CollisionPair2D(first.Collider, bodylessSecond);

        firstPair.WakeSleepingBodiesForCollision();
        secondPair.WakeSleepingBodiesForCollision();

        SolidBody2D awake = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: false);
        SolidBody2D sleeping = CreateCircle(
            context,
            new Vector2d((Fixed64)8 + Fixed64.Half, Fixed64.Zero),
            immovable: false);
        sleeping.Sleep();
        var sleepingSecondPair = new CollisionPair2D(awake.Collider, sleeping.Collider);
        sleepingSecondPair.WakeSleepingBodiesForCollision();

        second.IsSleeping.Should().BeTrue();
        first.IsSleeping.Should().BeTrue();
        sleeping.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void TryCollide_WithCompoundCompoundSeparatedParts_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateCompound(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-2), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)));
        SolidBody2D second = CreateCompound(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        var pair = new CollisionPair2D(first.Collider, second.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 3).Should().BeFalse();

        pair.Manifold.HasContact.Should().BeFalse();
        pair.Manifold.LastUpdatedFrame.Should().Be(3);
    }

    [Fact]
    public void TryCollide_WithCompoundCompoundMatchingLaterParts_ShouldPopulateManifold()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateCompound(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        SolidBody2D second = CreateCompound(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)));
        var pair = new CollisionPair2D(first.Collider, second.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 4).Should().BeTrue();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
        pair.Manifold.LastUpdatedFrame.Should().Be(4);
    }

    [Fact]
    public void TryCollide_WithCompoundAsSecondCollider_ShouldPopulateManifoldUsingReversedPartOrder()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D circle = CreateCircle(context, Vector2d.Zero, immovable: true);
        SolidBody2D compound = CreateCompound(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)3, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Half, Fixed64.Zero)));
        var pair = new CollisionPair2D(circle.Collider, compound.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 5).Should().BeTrue();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
        pair.Manifold.PrimaryContact.Normal.X.Abs().Should().Be(Fixed64.One);
        pair.Manifold.LastUpdatedFrame.Should().Be(5);
    }

    [Fact]
    public void TryCollide_WithCapsuleSideAgainstBox_ShouldPopulateTwoSideContacts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero, immovable: true);
        SolidBody2D box = CreateBox(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true,
            new Vector2d((Fixed64)2, (Fixed64)4));
        var pair = new CollisionPair2D(capsule.Collider, box.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 6).Should().BeTrue();

        pair.Manifold.Count.Should().Be(2);
        pair.Manifold.LastUpdatedFrame.Should().Be(6);
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.Should().OnlyContain(contact => contact.Normal.X < Fixed64.Zero);
    }

    [Fact]
    public void TryCollide_WithBoxAgainstCapsuleSide_ShouldPopulateTwoReversedSideContacts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D box = CreateBox(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true,
            new Vector2d((Fixed64)2, (Fixed64)4));
        SolidBody2D capsule = CreateCapsule(context, Vector2d.Zero, immovable: true);
        var pair = new CollisionPair2D(box.Collider, capsule.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, frame: 7).Should().BeTrue();

        pair.Manifold.Count.Should().Be(2);
        pair.Manifold.LastUpdatedFrame.Should().Be(7);
        pair.Manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        pair.Manifold.Should().OnlyContain(contact => contact.Normal.X < Fixed64.Zero);
    }

    private static CollisionPair2D GetPair(LSCollider2D first, LSCollider2D second)
    {
        if (first.TryGetCollisionPair(second.Id, out CollisionPair2D? firstPair) && firstPair != null)
            return firstPair;

        second.TryGetCollisionPair(first.Id, out CollisionPair2D? secondPair).Should().BeTrue();
        return secondPair!;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static SolidBody2D CreateCircle(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context, CreateTransform(position)),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable) =>
        CreateBox(context, position, immovable, new Vector2d((Fixed64)2, (Fixed64)2));

    private static SolidBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable,
        Vector2d size)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context, CreateTransform(position)),
            new LSAABBoxCollider2D(size))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateCapsule(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context, CreateTransform(position)),
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateCompound(
        GravitasWorldContext context,
        Vector2d position,
        params CompoundColliderPart2D[] parts)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context, CreateTransform(position)),
            new LSCompoundCollider2D(parts))
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(new TestMatterAgent(context, CreateTransform(position)));
        return collider;
    }

    private static FixedTransform CreateTransform(Vector2d position) =>
        new(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
}
