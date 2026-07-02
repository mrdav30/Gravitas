using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Collider2DShapeTests
{
    [Fact]
    public void CircleCollider2D_ShouldOwnPure2DBounds()
    {
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSCircleCollider2D(Fixed64.One);
        var transform = new FixedTransform(new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)3), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(new Vector2d((Fixed64)2, (Fixed64)3));

        collider.Shape.Should().Be(ColliderType2D.Circle);
        collider.Bounds.Min.Should().Be(new Vector2d(Fixed64.One, (Fixed64)2));
        collider.Bounds.Max.Should().Be(new Vector2d((Fixed64)3, (Fixed64)4));
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
        using GravitasWorldContext context = Create2DContext();
        var collider = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));
        var transform = new FixedTransform(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)5), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(new Vector2d((Fixed64)4, (Fixed64)5), Fixed64.Pi / (Fixed64)2);

        collider.GetWorldVertex(0).Should().Be(new Vector2d((Fixed64)5, (Fixed64)4));
        collider.GetWorldVertex(1).Should().Be(new Vector2d((Fixed64)5, (Fixed64)6));
        collider.GetWorldVertex(2).Should().Be(new Vector2d((Fixed64)3, (Fixed64)5));
    }

    private static GravitasWorldContext Create2DContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        return context;
    }
}
