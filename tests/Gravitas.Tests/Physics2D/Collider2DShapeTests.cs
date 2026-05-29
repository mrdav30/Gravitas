using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Collider2DShapeTests
{
    [Fact]
    public void CircleCollider2D_ShouldOwnPure2DBoundsAndDimension()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSCircleCollider2D(Fixed64.One);
        var body = new StiffBody2D(context, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(new Vector2d((Fixed64)2, (Fixed64)3));

        collider.Dimension.Should().Be(PhysicsDimension.TwoD);
        collider.Shape.Should().Be(Collider2DType.Circle);
        collider.Bounds.Area.Min.Should().Be(new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));
        collider.Bounds.Area.Max.Should().Be(new Vector3d((Fixed64)3, (Fixed64)4, Fixed64.Zero));
        collider.Bounds.PlaneZ.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void PolygonCollider2D_WithConcaveVertices_ShouldThrow()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)2, Fixed64.Zero),
            new(Fixed64.One, Fixed64.Half),
            new((Fixed64)2, (Fixed64)2),
            new(Fixed64.Zero, (Fixed64)2)
        };

        Action create = () => _ = new LSPolygonCollider2D(vertices);

        create.Should()
            .Throw<ArgumentException>()
            .WithMessage("*convex*");
    }

    [Fact]
    public void PolygonCollider2D_ShouldUpdateWorldVerticesFromBodyRotation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));
        var body = new StiffBody2D(context, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(new Vector2d((Fixed64)4, (Fixed64)5), FixedMath.PI / (Fixed64)2);

        collider.GetWorldVertex(0).Should().Be(new Vector2d((Fixed64)5, (Fixed64)4));
        collider.GetWorldVertex(1).Should().Be(new Vector2d((Fixed64)5, (Fixed64)6));
        collider.GetWorldVertex(2).Should().Be(new Vector2d((Fixed64)3, (Fixed64)5));
    }
}
