using FixedMathSharp;
using FluentAssertions;
using Gravitas.Queries;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class ConvexSweepTriangleReducerTests
{
    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithFirstVertexRegion_ShouldReturnFirstVertexWeights()
    {
        Vector3d first = Vector3d.Right;
        Vector3d second = new((Fixed64)2, Fixed64.One, Fixed64.Zero);
        Vector3d third = new((Fixed64)2, -Fixed64.One, Fixed64.Zero);

        ConvexSweepQueryWorker.TriangleWeights weights =
            ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(first, second, third);

        weights.A.Should().Be(Fixed64.One);
        weights.B.Should().Be(Fixed64.Zero);
        weights.C.Should().Be(Fixed64.Zero);
        WeightedPoint(first, second, third, weights).Should().Be(first);
    }

    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithSecondVertexRegion_ShouldReturnSecondVertexWeights()
    {
        Vector3d first = new((Fixed64)2, Fixed64.One, Fixed64.Zero);
        Vector3d second = Vector3d.Right;
        Vector3d third = new((Fixed64)2, -Fixed64.One, Fixed64.Zero);

        ConvexSweepQueryWorker.TriangleWeights weights =
            ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(first, second, third);

        weights.A.Should().Be(Fixed64.Zero);
        weights.B.Should().Be(Fixed64.One);
        weights.C.Should().Be(Fixed64.Zero);
        WeightedPoint(first, second, third, weights).Should().Be(second);
    }

    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithThirdVertexRegion_ShouldReturnThirdVertexWeights()
    {
        Vector3d first = new((Fixed64)2, Fixed64.One, Fixed64.Zero);
        Vector3d second = new((Fixed64)2, -Fixed64.One, Fixed64.Zero);
        Vector3d third = Vector3d.Right;

        ConvexSweepQueryWorker.TriangleWeights weights =
            ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(first, second, third);

        weights.A.Should().Be(Fixed64.Zero);
        weights.B.Should().Be(Fixed64.Zero);
        weights.C.Should().Be(Fixed64.One);
        WeightedPoint(first, second, third, weights).Should().Be(third);
    }

    private static Vector3d WeightedPoint(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        ConvexSweepQueryWorker.TriangleWeights weights) =>
        first * weights.A + second * weights.B + third * weights.C;
}
