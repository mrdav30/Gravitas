using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DQueryTests
{
    [Fact]
    public void OverlapCircleAll_ShouldUsePure2DShapeMathAndStableOrdering()
    {
        using GravitasWorldContext context = Create2DContext();
        StiffBody2D first = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        StiffBody2D second = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        _ = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Physics2D.OverlapCircleAll(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            (Fixed64)3,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(first.Collider);
        hits[1].Collider.Should().BeSameAs(second.Collider);
        hits[0].Distance.Should().BeLessThanOrEqualTo(hits[1].Distance);
    }

    [Fact]
    public void OverlapCircleAll_WithLayerMask_ShouldFilter2DHits()
    {
        using GravitasWorldContext context = Create2DContext();
        _ = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.Zero), new PhysicsLayer(0));
        StiffBody2D included = CreateCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero), new PhysicsLayer(1));
        _ = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), new PhysicsLayer(2));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Physics2D.OverlapCircleAll(
            Vector2d.Zero,
            (Fixed64)4,
            PhysicsLayerMask.FromLayer(1),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
    }

    private static StiffBody2D CreateCircle(GravitasWorldContext context, Vector2d position)
    {
        return CreateCircle(context, position, new PhysicsLayer());
    }

    private static StiffBody2D CreateCircle(GravitasWorldContext context, Vector2d position, PhysicsLayer layer)
    {
        var transform = new FixedTransform(new Vector3d(position.x, Fixed64.Zero, position.y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static StiffBody2D CreateBox(GravitasWorldContext context, Vector2d position)
    {
        return CreateBox(context, position, new PhysicsLayer());
    }

    private static StiffBody2D CreateBox(GravitasWorldContext context, Vector2d position, PhysicsLayer layer)
    {
        var transform = new FixedTransform(new Vector3d(position.x, Fixed64.Zero, position.y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSAABBoxCollider2D(Vector2d.One))
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static GravitasWorldContext Create2DContext()
    {
        return Physics2DTestWorld.CreateContext();
    }
}
