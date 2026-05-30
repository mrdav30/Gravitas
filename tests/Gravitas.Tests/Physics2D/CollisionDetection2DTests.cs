using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class CollisionDetection2DTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.ConvexPolygon)]
    public void TryCollide_ShouldSupportRequiredShapePairs(ColliderType2D firstType, ColliderType2D secondType)
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(firstType);
        LSCollider2D second = CreateCollider(secondType);
        _ = CreateBody(context, first, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        _ = CreateBody(context, second, new Vector2d(Fixed64.Half, Fixed64.Zero));

        bool result = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        result.Should().BeTrue();
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Normal.x.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollide_WithSeparatedPolygons_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(ColliderType2D.ConvexPolygon);
        LSCollider2D second = CreateCollider(ColliderType2D.ConvexPolygon);
        _ = CreateBody(context, first, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        _ = CreateBody(context, second, new Vector2d((Fixed64)5, Fixed64.Zero));

        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    private static StiffBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.x, Fixed64.Zero, position.y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(position);
        return body;
    }

    private static GravitasWorldContext Create2DContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        return context;
    }

    private static LSCollider2D CreateCollider(ColliderType2D type) =>
        type switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.One),
            ColliderType2D.AABox => new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)),
            _ => throw new Xunit.Sdk.XunitException("Unsupported test collider type.")
        };
}
