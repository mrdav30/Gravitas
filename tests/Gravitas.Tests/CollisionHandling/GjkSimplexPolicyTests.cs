using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class GjkSimplexPolicyTests
{
    [Theory]
    [InlineData(128)]
    [InlineData(129)]
    public void TriangleAndTetrahedron_ShouldRetainTheSameRegionAcrossTheScalarCrossCutover(int scale)
    {
        // The largest vertex component is exactly 256, then just beyond it.
        Vector3d a = new Vector3d(-2, 0, 0) * (Fixed64)scale;
        Vector3d b = new Vector3d(0, 1, 0) * (Fixed64)scale;
        Vector3d c = new Vector3d(0, 0, 1) * (Fixed64)scale;
        Vector3d d = new Vector3d(-1, 1, 0) * (Fixed64)scale;
        Vector3d[] simplex = { a, b, c, d };
        int count = 3;
        Vector3d direction = Vector3d.Zero;
        Vector3d expected = new Vector3d(1, -2, -2).Normalized;

        GjkSimplexPolicy.UpdateTriangle(simplex, ref count, ref direction).Should().BeFalse();
        count.Should().Be(3);
        simplex[0].Should().Be(a);
        simplex[1].Should().Be(b);
        simplex[2].Should().Be(c);
        direction.Normalized.Should().Be(expected);
        count = 4;
        GjkSimplexPolicy.UpdateTetrahedron(simplex, ref count, ref direction).Should().BeFalse();
        count.Should().Be(3);
        simplex[0].Should().Be(a);
        simplex[1].Should().Be(b);
        simplex[2].Should().Be(c);
        direction.Normalized.Should().Be(expected);
    }

    [Fact]
    public void LargeCollinearLine_ShouldRetainItsStablePerpendicularFallback()
    {
        Vector3d[] simplex = { Vector3d.Up * (Fixed64)1024, -Vector3d.Up * (Fixed64)1024 };
        int count = 2;
        Vector3d direction = Vector3d.Zero;
        GjkSimplexPolicy.UpdateLine(simplex, ref count, ref direction).Should().BeFalse();
        count.Should().Be(2);
        direction.Y.Should().Be(Fixed64.Zero);
        direction.Z.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(0, 1)]
    [InlineData(1, -1)]
    [InlineData(1, 1)]
    [InlineData(2, -1)]
    [InlineData(2, 1)]
    public void SeparatingSupport_ShouldPreserveSignsBeyondEitherProductBound(int axis, int sign)
    {
        Vector3d normal = axis switch { 0 => Vector3d.Right, 1 => Vector3d.Up, _ => Vector3d.Forward };
        normal *= (Fixed64)sign;
        GjkSimplexPolicy.HasSeparatingSupport(normal * (Fixed64)65, -normal).Should().BeTrue();
        GjkSimplexPolicy.HasSeparatingSupport(normal * (Fixed64)65, normal).Should().BeFalse();
        GjkSimplexPolicy.HasSeparatingSupport(normal, -normal * (Fixed64)((1L << 23) + 1)).Should().BeTrue();
        GjkSimplexPolicy.HasSeparatingSupport(normal, normal * (Fixed64)((1L << 23) + 1)).Should().BeFalse();
    }

    [Fact]
    public void SeparatingSupport_ShouldRetainCancellationAndTheStrictToleranceBoundary()
    {
        Vector3d point = Vector3d.One * Fixed64.MaxValue;
        var direction = new Vector3d(-Fixed64.MaxValue, -Fixed64.MaxValue, Fixed64.MaxValue);
        // The exact dot is -MaxValue^2; saturating each product loses this sign.
        GjkSimplexPolicy.HasSeparatingSupport(point, direction).Should().BeTrue();
        GjkSimplexPolicy.HasSeparatingSupport(point, -direction).Should().BeFalse();
        GjkSimplexPolicy.HasSeparatingSupport(Vector3d.Zero, direction).Should().BeFalse();
        GjkSimplexPolicy.HasSeparatingSupport(-Vector3d.Right * Fixed64.Epsilon, Vector3d.Right)
            .Should().BeFalse();
        GjkSimplexPolicy.HasSeparatingSupport(-Vector3d.Right * (Fixed64.Epsilon + Fixed64.MinIncrement), Vector3d.Right)
            .Should().BeTrue();
    }

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

    [Fact]
    public void UpdateLine_WithThresholdAxisCandidates_ShouldUseForwardFallback()
    {
        Fixed64 delta = FixedMath.Sqrt(Fixed64.Epsilon);
        Vector3d a = new(-delta, -delta, Fixed64.Zero);
        Vector3d b = Vector3d.Zero;
        Vector3d ab = b - a;
        Vector3d ao = -a;
        Vector3d tripleCross = Vector3d.Cross(Vector3d.Cross(ab, ao), ab);
        Vector3d upCandidate = Vector3d.Cross(ab, Vector3d.Up);
        Vector3d rightCandidate = Vector3d.Cross(ab, Vector3d.Right);
        Vector3d[] simplex = { a, b, Vector3d.Zero, Vector3d.Zero };
        int count = 2;
        Vector3d direction = Vector3d.Zero;

        (delta * delta).Should().Be(Fixed64.Epsilon);
        Vector3d.Dot(ab, ao).Should().BeGreaterThan(Fixed64.Zero);
        tripleCross.Should().Be(Vector3d.Zero);
        upCandidate.MagnitudeSquared.Should().Be(Fixed64.Epsilon);
        rightCandidate.MagnitudeSquared.Should().Be(Fixed64.Epsilon);

        bool containsOrigin = GjkSimplexPolicy.UpdateLine(simplex, ref count, ref direction);

        containsOrigin.Should().BeFalse();
        count.Should().Be(2);
        simplex[0].Should().Be(a);
        simplex[1].Should().Be(b);
        direction.Should().Be(Vector3d.Forward);
        direction.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Epsilon);
    }

    [Fact]
    public void UpdateTriangle_WithThresholdAcDirection_ShouldUseStablePerpendicularFallback()
    {
        Fixed64 delta = FixedMath.Sqrt(Fixed64.Epsilon);
        Vector3d a = new(-Fixed64.One, -delta, Fixed64.Zero);
        Vector3d b = new(-Fixed64.One, -Fixed64.One - delta, Fixed64.Zero);
        Vector3d c = new(Fixed64.Zero, -delta, Fixed64.Zero);
        Vector3d ab = b - a;
        Vector3d ac = c - a;
        Vector3d ao = -a;
        Vector3d abc = Vector3d.Cross(ab, ac);
        Vector3d acPerpendicular = Vector3d.Cross(abc, ac);
        Vector3d tripleCross = Vector3d.Cross(Vector3d.Cross(ac, ao), ac);
        Vector3d[] simplex = { a, b, c, Vector3d.Zero };
        int count = 3;
        Vector3d direction = Vector3d.Zero;

        (delta * delta).Should().Be(Fixed64.Epsilon);
        Vector3d.Dot(acPerpendicular, ao).Should().BeGreaterThan(Fixed64.Zero);
        Vector3d.Dot(ac, ao).Should().BeGreaterThan(Fixed64.Zero);
        tripleCross.MagnitudeSquared.Should().Be(Fixed64.Epsilon);

        bool containsOrigin = GjkSimplexPolicy.UpdateTriangle(simplex, ref count, ref direction);

        containsOrigin.Should().BeFalse();
        count.Should().Be(2);
        simplex[0].Should().Be(a);
        simplex[1].Should().Be(c);
        direction.Should().Be(Vector3d.Forward);
        direction.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Epsilon);
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
        // Preserve the same simplex regions when cubic/quartic products exceed Fixed64.
        Fixed64 scale = (Fixed64)1024;
        simplex = new[] { a * scale, b * scale, c * scale, Vector3d.Zero };
        count = 3;
        direction = Vector3d.Zero;
        GjkSimplexPolicy.UpdateTriangle(simplex, ref count, ref direction).Should().BeFalse();
        count.Should().Be(expectedCount);
        simplex[0].Should().Be(expectedFirst * scale);
        if (expectedCount > 1)
            simplex[1].Should().Be(expectedSecond * scale);
        if (expectedCount > 2)
            simplex[2].Should().Be(expectedThird * scale);
        Vector3d directionError = direction.Normalized - expectedDirection.Normalized;
        directionError.X.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * Fixed64.Two);
        directionError.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * Fixed64.Two);
        directionError.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * Fixed64.Two);
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
        Fixed64 scale = (Fixed64)1024;
        simplex = new[] { a * scale, b * scale, c * scale, d * scale };
        count = 4;
        direction = Vector3d.Zero;
        GjkSimplexPolicy.UpdateTetrahedron(simplex, ref count, ref direction)
            .Should().Be(expectedContainsOrigin);
        count.Should().Be(expectedCount);
        simplex[0].Should().Be(expectedFirst * scale);
        if (!expectedContainsOrigin)
        {
            simplex[1].Should().Be(expectedSecond * scale);
            simplex[2].Should().Be(expectedThird * scale);
            Vector3d directionError = direction.Normalized - expectedDirection.Normalized;
            directionError.X.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * Fixed64.Two);
            directionError.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * Fixed64.Two);
            directionError.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * Fixed64.Two);
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
