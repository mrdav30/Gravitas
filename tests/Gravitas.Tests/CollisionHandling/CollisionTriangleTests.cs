using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.CollisionHandling;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionTriangleTests
{
    [Fact]
    public void Constructor_ShouldPreserveOrderedTriangleGeometryCachedNormalAndQueryBounds()
    {
        Vector3d first = new(Fixed64.One, (Fixed64)2, (Fixed64)(-3));
        Vector3d second = new((Fixed64)4, Fixed64.Zero, (Fixed64)5);
        Vector3d third = new((Fixed64)(-2), (Fixed64)6, Fixed64.Half);
        Vector3d cachedNormal = Vector3d.Down;
        FixedBoundVolume queryBounds = new(
            Vector3d.Min(Vector3d.Min(first, second), third),
            Vector3d.Max(Vector3d.Max(first, second), third));

        CollisionTriangle triangle = new(new FixedTriangle(first, second, third), cachedNormal, queryBounds);

        triangle.A.Should().Be(first);
        triangle.B.Should().Be(second);
        triangle.C.Should().Be(third);
        triangle.Triangle.Should().Be(new FixedTriangle(first, second, third));
        triangle.Center.Should().Be((first + second + third) / (Fixed64)3);
        triangle.GetEdgeVector(0).Should().Be(second - first);
        triangle.GetEdgeVector(1).Should().Be(third - second);
        triangle.GetEdgeVector(2).Should().Be(first - third);
        triangle.Normal.Should().Be(cachedNormal);
        triangle.QueryBounds.Min.Should().Be(queryBounds.Min);
        triangle.QueryBounds.Max.Should().Be(queryBounds.Max);
    }

    [Fact]
    public void Constructor_ShouldKeepCachedNormalInsteadOfRecomputingTriangleNormal()
    {
        FixedTriangle geometry = new(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);
        Vector3d cachedNormal = Vector3d.Left;

        CollisionTriangle triangle = new(
            geometry,
            cachedNormal,
            new FixedBoundVolume(geometry.Bounds.Min, geometry.Bounds.Max));

        geometry.Normal.Should().NotBe(cachedNormal);
        triangle.Normal.Should().Be(cachedNormal);
    }
}
