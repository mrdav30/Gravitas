using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class CompoundPolygonMomentTests
{
    [Fact]
    public void TriangleMoment_WithAnExactCentroidOnOneAxis_ShouldRetainTheOtherFirstMoment()
    {
        Vector2d[] vertices = { Vector2d.Zero, Vector2d.Right * (Fixed64)3, new(Fixed64.Zero, Fixed64.One) };
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(vertices, Vector2d.Zero));
        Fixed64 mass = (Fixed64)100_000_000;

        // E[x²] = 3²/6 and E[y²] = 1/6, while only centroid Y needs rounding.
        compound.CalculateMomentOfInertia(mass, Vector2d.Zero)
            .Should().Be(RoundRational((BigInteger)mass.m_rawValue * 5, 3));
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, true, true)]
    public void TriangleMoment_ShouldRetainResidualFirstMomentsAboutRequestedReference(bool clockwise, bool translated, bool quarterTurn)
    {
        Vector2d offset = translated ? new Vector2d((Fixed64)3, (Fixed64)(-2)) : Vector2d.Zero;
        Vector2d reference = translated ? new Vector2d(Fixed64.One, (Fixed64)4) : Vector2d.Zero;
        var compound = CreateTriangle(clockwise, offset, quarterTurn);
        Fixed64 mass = translated ? (Fixed64)10_000_000 : (Fixed64)1_000_000_000;

        compound.CalculateMomentOfInertia(mass, reference)
            .Should().Be(TriangleMoment(mass, Fixed64.One, offset, reference, quarterTurn));
        Vector2d publishedCenter = compound.CalculateLocalCenterOfMassOffset();
        compound.CalculateMomentOfInertia(mass, publishedCenter)
            .Should().Be(TriangleMoment(mass, Fixed64.One, offset, publishedCenter, quarterTurn));
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        compound.CalculateLocalCenterOfMassOffset().Should().Be(publishedCenter);
        compound.CalculateMomentOfInertia(mass, reference)
            .Should().Be(TriangleMoment(mass, Fixed64.One, offset, reference, quarterTurn));
    }

    [Fact]
    public void TriangleMomentCache_ShouldFollowPublishedScaleAndOwnerOffset()
    {
        using var context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var compound = CreateTriangle(false, new Vector2d((Fixed64)2, -Fixed64.One));
        compound.LocalOffset = new Vector2d((Fixed64)3, (Fixed64)(-2));
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Fixed64 mass = (Fixed64)10_000_000;
        Vector2d reference = Vector2d.Zero;
        compound.CalculateMomentOfInertia(mass, reference)
            .Should().Be(TriangleMoment(mass, Fixed64.One, new Vector2d((Fixed64)5, (Fixed64)(-3)), reference));

        compound.LocalOffset = new Vector2d(Fixed64.One, (Fixed64)2);
        compound.Simulate();
        Vector2d center = compound.CalculateLocalCenterOfMassOffset();
        compound.CalculateMomentOfInertia(mass, center)
            .Should().Be(TriangleMoment(mass, Fixed64.One, new Vector2d((Fixed64)3, Fixed64.One), center));

        transform.LocalScale = Vector3d.One * Fixed64.Two;
        compound.Simulate();
        center = compound.CalculateLocalCenterOfMassOffset();
        compound.CalculateMomentOfInertia(mass, center)
            .Should().Be(TriangleMoment(mass, Fixed64.Two, new Vector2d((Fixed64)6, (Fixed64)2), center));
        compound.CalculateMomentOfInertia(mass, reference)
            .Should().Be(TriangleMoment(mass, Fixed64.Two, new Vector2d((Fixed64)6, (Fixed64)2), reference));
    }

    private static LSCompoundCollider2D CreateTriangle(bool clockwise, Vector2d offset, bool quarterTurn = false)
    {
        Vector2d[] vertices = { Vector2d.Zero, Vector2d.Right, new Vector2d(Fixed64.Zero, Fixed64.One) };
        if (clockwise) Array.Reverse(vertices);
        return new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(
            vertices, offset, quarterTurn ? Fixed64.HalfPi : Fixed64.Zero, Vector2d.One));
    }

    private static Fixed64 TriangleMoment(Fixed64 mass, Fixed64 side, Vector2d offset, Vector2d reference, bool quarterTurn = false)
    {
        // Uniform right triangle: E[x] = E[y] = side/3 and E[x²+y²] = side²/3.
        // Integrate about the requested reference directly, independently of a rounded centroid.
        BigInteger s = side.m_rawValue;
        BigInteger dx = (BigInteger)offset.X.m_rawValue - reference.X.m_rawValue;
        BigInteger dy = (BigInteger)offset.Y.m_rawValue - reference.Y.m_rawValue;
        BigInteger numerator = mass.m_rawValue * (s * s + 2 * s * ((quarterTurn ? -dx : dx) + dy) + 3 * (dx * dx + dy * dy));
        BigInteger denominator = 3 * (BigInteger.One << 64);
        return RoundRational(numerator, denominator);
    }

    private static Fixed64 RoundRational(BigInteger numerator, BigInteger denominator)
    {
        BigInteger raw = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        int half = (2 * remainder).CompareTo(denominator);
        if (half > 0 || half == 0 && !raw.IsEven) raw++;
        return Fixed64.FromRaw((long)raw);
    }
}
