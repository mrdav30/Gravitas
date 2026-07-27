using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Xunit;

namespace Gravitas.Tests.QueryTests;

public sealed class Physics2DHitTests
{
    [Fact]
    public void RelativeAnchor_ExposesUnavailableWorldPointExplicitly()
    {
        var collider = new LSCircleCollider2D(Fixed64.One);
        var hit = new Physics2DHit(
            collider,
            new ContactAnchor2D(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                Vector2d.Right),
            Vector2d.Left,
            Fixed64.One);

        hit.Anchor.Offset.Should().Be(Vector2d.Right);
        hit.TryGetPoint(out _).Should().BeFalse();
        Action readPoint = () => _ = hit.Point;
        readPoint.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void AbsoluteConstructor_PreservesWorldPoint()
    {
        var collider = new LSCircleCollider2D(Fixed64.One);
        Vector2d point = new(Fixed64.One, Fixed64.Two);
        var hit = new Physics2DHit(
            collider,
            point,
            Vector2d.Forward,
            Fixed64.Two);

        hit.Anchor.Should().BeEquivalentTo(ContactAnchor2D.FromWorldPoint(point));
        hit.TryGetPoint(out Vector2d resolved).Should().BeTrue();
        resolved.Should().Be(point);
        hit.Point.Should().Be(point);
    }
}
