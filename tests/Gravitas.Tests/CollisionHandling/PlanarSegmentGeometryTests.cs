using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class PlanarSegmentGeometryTests
{
    [Fact]
    public void ClosestPoint_ShouldProjectOntoFiniteSegment()
    {
        Vector2d start = new(Fixed64.Zero, Fixed64.Zero);
        Vector2d end = new((Fixed64)4, Fixed64.Zero);
        Vector2d point = new((Fixed64)3, (Fixed64)2);

        Vector2d closest = PlanarSegmentGeometry.ClosestPoint(point, start, end);

        closest.Should().Be(new Vector2d((Fixed64)3, Fixed64.Zero));
        PlanarSegmentGeometry.DistanceSquared(point, start, end).Should().Be((Fixed64)4);
    }

    [Fact]
    public void ClosestPoint_ShouldTreatNearZeroSegmentAsCollapsed()
    {
        Vector2d start = new((Fixed64)2, (Fixed64)(-3));
        Vector2d end = start + new Vector2d(Fixed64.Epsilon, Fixed64.Zero);
        Vector2d point = start + new Vector2d((Fixed64)10, (Fixed64)10);

        PlanarSegmentGeometry.ClosestPoint(point, start, end).Should().Be(start);
    }
}
