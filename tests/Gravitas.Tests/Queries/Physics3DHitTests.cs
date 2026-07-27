using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Xunit;

namespace Gravitas.Tests.QueryTests;

public sealed class Physics3DHitTests
{
    [Fact]
    public void RelativeAnchor_ExposesUnavailableWorldPointExplicitly()
    {
        var hit = new Physics3DHit(
            collider: null,
            new ContactAnchor(
                new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right),
            Vector3d.Left,
            Fixed64.One,
            Vector3d.Right);

        hit.Anchor.LocalPoint.Should().Be(Vector3d.Right);
        hit.TryGetPoint(out _).Should().BeFalse();
        Action readPoint = () => _ = hit.Point;
        readPoint.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void AbsoluteConstructor_PreservesCompatibilityPoint()
    {
        Vector3d point = new(1, 2, 3);
        var hit = new Physics3DHit(
            collider: null,
            point,
            Vector3d.Up,
            Fixed64.Two,
            Vector3d.Right);

        hit.Anchor.Should().BeEquivalentTo(ContactAnchor.FromWorldPoint(point));
        hit.TryGetPoint(out Vector3d resolved).Should().BeTrue();
        resolved.Should().Be(point);
        hit.Point.Should().Be(point);
    }
}
