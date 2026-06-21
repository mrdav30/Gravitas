using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class CompoundCollider2DCollisionTests
{
    [Fact]
    public void TryCollide_WithCompoundCircle_ShouldUseMatchingPartContact()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);
        StiffBody2D circle = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        var manifold = new ContactManifold2D();
        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(compound.Shape, circle.Collider.Shape);
        bool collided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(compound, circle.Collider, collisionType),
            manifold,
            frame: 3);

        collided.Should().BeTrue();
        manifold.HasContact.Should().BeTrue();
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        manifold.PrimaryContact.Normal.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void Simulate_WithCompoundInternalOverlap_ShouldNotifyOwningColliderOnce()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        StiffBody2D compoundBody = CreateBody(context, compound, Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        int contactCount = 0;
        compound.OnContact += _ => contactCount++;

        Step(context);

        contactCount.Should().Be(1);
        compoundBody.Collider.CollisionPairCount.Should().Be(1);
    }

    [Fact]
    public void OverlapCircleAll_ShouldReturnCompoundOwnerOnce()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.One, Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapCircleAll(Vector2d.Zero, (Fixed64)3, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(compound);
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepCircleAll_ShouldReturnCompoundOwnerThroughEarliestPart()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-1), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(compound);
        hits[0].Distance.Should().Be((Fixed64)2);
        hits[0].Normal.Should().Be(-Vector2d.Right);
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable = true)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }
}
