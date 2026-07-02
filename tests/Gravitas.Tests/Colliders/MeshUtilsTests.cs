using FixedMathSharp;
using FixedMathSharp.Bounds;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class MeshUtilsTests
{
    public static TheoryData<string, Vector3d> ClosestPointCases()
    {
        return new TheoryData<string, Vector3d>
        {
            { "above triangle interior", new Vector3d(Fixed64.Half, (Fixed64)3, Fixed64.Half) },
            { "outside edge AB", new Vector3d(Fixed64.Half, Fixed64.Zero, (Fixed64)(-1)) },
            { "outside edge BC", new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2) },
            { "outside edge CA", new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Half) },
            { "nearest vertex A", new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)(-1)) },
            { "nearest vertex B", new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)3) },
            { "nearest vertex C", new Vector3d((Fixed64)3, Fixed64.Zero, (Fixed64)(-1)) }
        };
    }

    [Theory]
    [MemberData(nameof(ClosestPointCases))]
    public void ClosestPointOnTriangle_ShouldMatchFixedTriangleForNonDegenerateTriangle(string _, Vector3d point)
    {
        FixedTriangle triangle = CreateTriangle();

        Vector3d actual = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, point);
        Vector3d expected = triangle.ClosestPoint(point);

        Vector3d.DistanceSquared(actual, expected).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void ClosestPointOnTriangle_WithRepeatedVertex_ShouldMatchFixedTriangleEdgeFallback()
    {
        FixedTriangle triangle = new(Vector3d.Zero, Vector3d.Right * (Fixed64)2, Vector3d.Right * (Fixed64)2);
        Vector3d point = new(Fixed64.Half, (Fixed64)3, Fixed64.Zero);

        Vector3d actual = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, Vector3d.Up, point);
        Vector3d expected = triangle.ClosestPoint(point);

        Vector3d.DistanceSquared(actual, expected).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void ClosestPointOnEdge_WithZeroLengthEdge_ShouldReturnStart()
    {
        Vector3d start = new(Fixed64.One, (Fixed64)2, (Fixed64)3);
        Vector3d point = new((Fixed64)4, (Fixed64)(-2), Fixed64.Half);

        Vector3d closest = MeshUtils.ClosestPointOnEdge(start, start, point);

        closest.Should().Be(start);
    }

    private static FixedTriangle CreateTriangle()
    {
        return new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Forward * (Fixed64)2,
            Vector3d.Right * (Fixed64)2);
    }
}
