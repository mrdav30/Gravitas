using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class GjkSimplexPolicyTests
{
    [Fact]
    public void AddPoint_WithFullSimplex_ShouldKeepFourNewestPoints()
    {
        Vector3d[] simplex =
        {
            new(1, 0, 0),
            new(0, 1, 0),
            new(0, 0, 1),
            new(-1, 0, 0)
        };
        int count = 4;

        GjkSimplexPolicy.AddPoint(simplex, ref count, new Vector3d(2, 0, 0));

        count.Should().Be(4);
        simplex[0].Should().Be(new Vector3d(2, 0, 0));
        simplex[1].Should().Be(new Vector3d(1, 0, 0));
        simplex[2].Should().Be(new Vector3d(0, 1, 0));
        simplex[3].Should().Be(new Vector3d(0, 0, 1));
    }

    [Fact]
    public void UpdateLine_WithOriginBeyondSegment_ShouldReduceToPoint()
    {
        Vector3d[] simplex =
        {
            new(1, 0, 0),
            new(2, 0, 0),
            Vector3d.Zero,
            Vector3d.Zero
        };
        int count = 2;
        Vector3d direction = Vector3d.Zero;

        bool containsOrigin = GjkSimplexPolicy.UpdateLine(simplex, ref count, ref direction);

        containsOrigin.Should().BeFalse();
        count.Should().Be(1);
        simplex[0].Should().Be(new Vector3d(1, 0, 0));
        direction.Should().Be(new Vector3d(-1, 0, 0));
    }

    [Fact]
    public void UpdateLine_WithVerticalCollinearOrigin_ShouldUseStableFallbackDirection()
    {
        Vector3d[] simplex =
        {
            new(0, 1, 0),
            new(0, -1, 0),
            Vector3d.Zero,
            Vector3d.Zero
        };
        int count = 2;
        Vector3d direction = Vector3d.Zero;

        bool containsOrigin = GjkSimplexPolicy.UpdateLine(simplex, ref count, ref direction);

        containsOrigin.Should().BeFalse();
        count.Should().Be(2);
        direction.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Epsilon);
        direction.Y.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [MemberData(nameof(TriangleRegionData))]
    public void UpdateTriangle_ShouldSelectExpectedSimplexRegion(
        Vector3d a,
        Vector3d b,
        Vector3d c,
        int expectedCount,
        Vector3d expectedFirst,
        Vector3d expectedSecond,
        Vector3d expectedThird,
        Vector3d expectedDirection)
    {
        Vector3d[] simplex = { a, b, c, Vector3d.Zero };
        int count = 3;
        Vector3d direction = Vector3d.Zero;

        bool containsOrigin = GjkSimplexPolicy.UpdateTriangle(simplex, ref count, ref direction);

        containsOrigin.Should().BeFalse();
        count.Should().Be(expectedCount);
        simplex[0].Should().Be(expectedFirst);
        if (expectedCount > 1)
            simplex[1].Should().Be(expectedSecond);
        if (expectedCount > 2)
            simplex[2].Should().Be(expectedThird);
        direction.Should().Be(expectedDirection);
    }

    public static TheoryData<Vector3d, Vector3d, Vector3d, int, Vector3d, Vector3d, Vector3d, Vector3d> TriangleRegionData()
    {
        return new TheoryData<Vector3d, Vector3d, Vector3d, int, Vector3d, Vector3d, Vector3d, Vector3d>
        {
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(1, 1, 0),
                2,
                new Vector3d(-2, 0, 0),
                new Vector3d(1, 1, 0),
                Vector3d.Zero,
                new Vector3d(2, -6, 0)
            },
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(-1, 1, 0),
                2,
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                Vector3d.Zero,
                new Vector3d(2, -4, 0)
            },
            {
                new Vector3d(-1, 0, 0),
                new Vector3d(-2, 0, 0),
                new Vector3d(-1, 1, 0),
                1,
                new Vector3d(-1, 0, 0),
                Vector3d.Zero,
                Vector3d.Zero,
                new Vector3d(1, 0, 0)
            },
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                3,
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                new Vector3d(1, -2, -2)
            },
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(-1, 0, 0),
                new Vector3d(0, 1, 0),
                3,
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(-1, 0, 0),
                new Vector3d(0, 0, -1)
            }
        };
    }

    [Theory]
    [MemberData(nameof(TetrahedronRegionData))]
    public void UpdateTetrahedron_ShouldSelectVisibleFaceOrReportOriginInside(
        Vector3d a,
        Vector3d b,
        Vector3d c,
        Vector3d d,
        bool expectedContainsOrigin,
        int expectedCount,
        Vector3d expectedFirst,
        Vector3d expectedSecond,
        Vector3d expectedThird,
        Vector3d expectedDirection)
    {
        Vector3d[] simplex = { a, b, c, d };
        int count = 4;
        Vector3d direction = Vector3d.Zero;

        bool containsOrigin = GjkSimplexPolicy.UpdateTetrahedron(simplex, ref count, ref direction);

        containsOrigin.Should().Be(expectedContainsOrigin);
        count.Should().Be(expectedCount);
        simplex[0].Should().Be(expectedFirst);
        if (!expectedContainsOrigin)
        {
            simplex[1].Should().Be(expectedSecond);
            simplex[2].Should().Be(expectedThird);
            direction.Should().Be(expectedDirection);
        }
    }

    public static TheoryData<Vector3d, Vector3d, Vector3d, Vector3d, bool, int, Vector3d, Vector3d, Vector3d, Vector3d> TetrahedronRegionData()
    {
        return new TheoryData<Vector3d, Vector3d, Vector3d, Vector3d, bool, int, Vector3d, Vector3d, Vector3d, Vector3d>
        {
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                new Vector3d(-1, 1, 0),
                false,
                3,
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                new Vector3d(1, -2, -2)
            },
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                new Vector3d(1, 1, 0),
                false,
                3,
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 0, 1),
                new Vector3d(1, 1, 0),
                new Vector3d(1, -3, -2)
            },
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                new Vector3d(1, 0, 1),
                false,
                3,
                new Vector3d(-2, 0, 0),
                new Vector3d(1, 0, 1),
                new Vector3d(0, 1, 0),
                new Vector3d(1, -2, -3)
            },
            {
                new Vector3d(-2, 0, 0),
                new Vector3d(-1, 0, 0),
                new Vector3d(0, 1, 0),
                new Vector3d(0, 0, 1),
                true,
                4,
                new Vector3d(-2, 0, 0),
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero
            }
        };
    }
}
