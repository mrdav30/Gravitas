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

    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithZeroArea_ShouldReduceToTheClosestEdge()
    {
        Vector3d first = new((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        Vector3d second = new((Fixed64)2, Fixed64.Zero, Fixed64.Zero);
        Vector3d third = new((Fixed64)4, Fixed64.Zero, Fixed64.Zero);

        ConvexSweepQueryWorker.TriangleWeights weights =
            ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(
                first,
                second,
                third);

        WeightedPoint(first, second, third, weights).Should().Be(Vector3d.Zero);
        (weights.A + weights.B + weights.C).Should().Be(Fixed64.One);
    }

    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithPermutedZeroAreaVertices_ShouldKeepTheSamePoint()
    {
        Vector3d first = new((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        Vector3d second = new((Fixed64)2, Fixed64.Zero, Fixed64.Zero);
        Vector3d third = new((Fixed64)4, Fixed64.Zero, Fixed64.Zero);
        Vector3d[][] permutations =
        {
            new[] { first, second, third },
            new[] { first, third, second },
            new[] { second, first, third },
            new[] { second, third, first },
            new[] { third, first, second },
            new[] { third, second, first }
        };

        foreach (Vector3d[] points in permutations)
        {
            ConvexSweepQueryWorker.TriangleWeights weights =
                ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(
                    points[0],
                    points[1],
                    points[2]);

            WeightedPoint(
                    points[0],
                    points[1],
                    points[2],
                    weights)
                .Should().Be(Vector3d.Zero);
        }
    }

    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithFullDomainCollinearity_ShouldRemainDefined()
    {
        Fixed64 step = Fixed64.MinIncrement;
        Vector3d first = Vector3d.One * Fixed64.MaxValue;
        Vector3d second = first - new Vector3d(step, step * (Fixed64)2, step * (Fixed64)3);
        Vector3d third = first - new Vector3d(step * (Fixed64)2, step * (Fixed64)4, step * (Fixed64)6);

        ConvexSweepQueryWorker.TriangleWeights weights =
            ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(
                first,
                second,
                third);

        weights.A.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        weights.B.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        weights.C.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        (weights.A + weights.B + weights.C).Should().Be(Fixed64.One);
    }

    [Fact]
    public void ClosestPointOnTriangleToOrigin_WithSubRawSquaredEdges_ShouldUseStableEdgeReduction()
    {
        Vector3d[][] triangles =
        {
            new[]
            {
                FromRaw(-131_072, -100_000, 0),
                FromRaw(-100_000, -131_072, 0),
                FromRaw(-131_072, -2, 0)
            },
            new[]
            {
                FromRaw(-250_956, -277_020, 385_071),
                FromRaw(77_177, 28_590, 203_164),
                FromRaw(-265_700, -308_004, 351_819)
            }
        };

        foreach (Vector3d[] triangle in triangles)
        {
            ConvexSweepQueryWorker.TriangleWeights first =
                ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(
                    triangle[0],
                    triangle[1],
                    triangle[2]);
            ConvexSweepQueryWorker.TriangleWeights repeated =
                ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin(
                    triangle[0],
                    triangle[1],
                    triangle[2]);

            repeated.Should().Be(first);
            first.A.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
            first.B.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
            first.C.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
            (first.A + first.B + first.C).Should().Be(Fixed64.One);
        }
    }

    [Fact]
    public void MeshSweepPruning_WithTiedOrNearerCandidateBound_ShouldPreserveFinalDistanceOrdering()
    {
        Fixed64 closestNumerator = Fixed64.Half;

        ConvexSweepQueryWorker.RemainingSweepTrianglesCannotBeat(
                Fixed64.Half,
                found: true,
                closestNumerator)
            .Should().BeFalse();
        ConvexSweepQueryWorker.RemainingSweepTrianglesCannotBeat(
                Fixed64.Half - Fixed64.Epsilon,
                found: true,
                closestNumerator)
            .Should().BeFalse();
        ConvexSweepQueryWorker.RemainingSweepTrianglesCannotBeat(
                Fixed64.Half + Fixed64.Epsilon * (Fixed64)2,
                found: true,
                closestNumerator)
            .Should().BeTrue();
        ConvexSweepQueryWorker.RemainingSweepTrianglesCannotBeat(
                Fixed64.One,
                found: false,
                closestNumerator)
            .Should().BeFalse();
    }

    private static Vector3d WeightedPoint(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        ConvexSweepQueryWorker.TriangleWeights weights) =>
        first * weights.A + second * weights.B + third * weights.C;

    private static Vector3d FromRaw(long x, long y, long z) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
}
