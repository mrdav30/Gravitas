using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class CollisionDetection2DTests
{
    [Theory]
    [InlineData(Collider2DType.Circle, Collider2DType.Circle)]
    [InlineData(Collider2DType.Circle, Collider2DType.AABox)]
    [InlineData(Collider2DType.AABox, Collider2DType.AABox)]
    [InlineData(Collider2DType.Circle, Collider2DType.ConvexPolygon)]
    [InlineData(Collider2DType.AABox, Collider2DType.ConvexPolygon)]
    [InlineData(Collider2DType.ConvexPolygon, Collider2DType.ConvexPolygon)]
    public void TryCollide_ShouldSupportRequiredShapePairs(Collider2DType firstType, Collider2DType secondType)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
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
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider2D first = CreateCollider(Collider2DType.ConvexPolygon);
        LSCollider2D second = CreateCollider(Collider2DType.ConvexPolygon);
        _ = CreateBody(context, first, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        _ = CreateBody(context, second, new Vector2d((Fixed64)5, Fixed64.Zero));

        CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    private static StiffBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var body = new StiffBody2D(context, collider)
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateCollider(Collider2DType type) =>
        type switch
        {
            Collider2DType.Circle => new LSCircleCollider2D(Fixed64.One),
            Collider2DType.AABox => new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            Collider2DType.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)),
            _ => throw new Xunit.Sdk.XunitException("Unsupported test collider type.")
        };
}
