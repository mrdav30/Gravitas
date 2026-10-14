using System;
using System.Numerics;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PolygonReferenceMomentTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Triangle_ShouldIntegrateAboutEachRequestedReferenceBeforeRounding(bool clockwise)
    {
        Vector2d[] vertices = Triangle(clockwise);
        var polygon = new LSPolygonCollider2D(vertices);
        Vector2d center = polygon.CalculateLocalCenterOfMassOffset();
        Vector2d[] references = { Vector2d.Zero, new(Fixed64.Half, -Fixed64.Quarter),
            new(-Fixed64.One, Fixed64.Two), center, new(center.X, Fixed64.Zero), new(Fixed64.Zero, center.Y) };
        foreach (long mass in new[] { 1L, 3L, 9L, 19L, 4294967296000000000L })
        foreach (Vector2d reference in references)
            Assert.Equal(Expected(vertices, mass, reference, Vector2d.Zero, Vector2d.One),
                polygon.CalculateMomentOfInertia(Fixed64.FromRaw(mass), reference));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SymmetricTriangle_ShouldRetainItsSingleAxisCentroidResidual(bool transpose)
    {
        Vector2d[] vertices = transpose
            ? new[] { -Vector2d.Forward, Vector2d.Forward, Vector2d.Right }
            : new[] { -Vector2d.Right, Vector2d.Right, Vector2d.Forward };
        var polygon = new LSPolygonCollider2D(vertices);
        Fixed64 mass = (Fixed64)1_000_000_000;
        Vector2d reference = new(Fixed64.Half, -Fixed64.Quarter);
        Assert.Equal(Expected(vertices, mass.m_rawValue, reference, Vector2d.Zero, Vector2d.One),
            polygon.CalculateMomentOfInertia(mass, reference));
    }

    [Theory]
    [InlineData(3, 0)]
    [InlineData(9, 2)]
    [InlineData(15, 2)]
    public void Square_WithExactCentroid_ShouldRetainWholeShapeHalfEvenTies(long mass, long expectedRaw)
    {
        Vector2d[] vertices = { new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half) };
        var polygon = new LSPolygonCollider2D(vertices);
        Assert.Equal(Fixed64.FromRaw(expectedRaw), polygon.CalculateMomentOfInertia(Fixed64.FromRaw(mass), Vector2d.Zero));
        Vector2d reference = new(Fixed64.Quarter, -Fixed64.Half);
        Assert.Equal(Expected(vertices, mass, reference, Vector2d.Zero, Vector2d.One),
            polygon.CalculateMomentOfInertia(Fixed64.FromRaw(mass), reference));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FractionalOffsetAndScale_ShouldRetainSemanticProductsThroughPublicationAndWorldYaw(bool clockwise)
    {
        using var context = Physics2DTestWorld.CreateContext();
        Vector2d[] vertices = Triangle(clockwise);
        Vector2d offset = new(Fixed64.FromRaw(1), Fixed64.FromRaw(-3));
        Vector2d scale = new(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(5, 4));
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(scale.X, Fixed64.One, scale.Y));
        var polygon = new LSPolygonCollider2D(vertices) { LocalOffset = offset };
        Fixed64 mass = (Fixed64)1_000_000_000;
        Vector2d reference = new(-Fixed64.Quarter, Fixed64.Half);
        Assert.Equal(Expected(vertices, mass.m_rawValue, reference, offset, Vector2d.One),
            polygon.CalculateMomentOfInertia(mass, reference));
        polygon.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Assert.Equal(Expected(vertices, mass.m_rawValue, reference, offset, scale), polygon.CalculateMomentOfInertia(mass, reference));
        Vector2d publishedCenter = polygon.CalculateLocalCenterOfMassOffset();
        Assert.Equal(Expected(vertices, mass.m_rawValue, publishedCenter, offset, scale),
            polygon.CalculateMomentOfInertia(mass, publishedCenter));

        transform.LocalRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        polygon.Simulate();
        Assert.Equal(Expected(vertices, mass.m_rawValue, reference, offset, scale), polygon.CalculateMomentOfInertia(mass, reference));
        transform.LocalScale = Vector3d.One * Fixed64.Two;
        polygon.Simulate();
        Assert.Equal(Expected(vertices, mass.m_rawValue, reference, offset, Vector2d.One * Fixed64.Two),
            polygon.CalculateMomentOfInertia(mass, reference));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ReservedPolygonPart_ShouldMatchWholeShapeIntegrationWithItsLocalRotation(bool clockwise, bool quarterTurn)
    {
        using var context = Physics2DTestWorld.CreateContext();
        Vector2d[] vertices = Triangle(clockwise);
        Vector2d partOffset = new(Fixed64.Two, -Fixed64.One);
        Vector2d partScale = new(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 4));
        Vector2d ownerOffset = new(Fixed64.FromRaw(1), Fixed64.FromRaw(-3));
        Fixed64 rotation = quarterTurn ? Fixed64.HalfPi : Fixed64.PiOver4;
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(vertices, partOffset, rotation, partScale))
            { LocalOffset = ownerOffset };
        LSCollider2D part = compound.GetPartCollider(0);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.One, Fixed64.FromFraction(3, 2)));
        foreach (bool initialized in new[] { false, true })
        {
            if (initialized) compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
            Vector2d ownerScale = initialized ? new Vector2d(Fixed64.FromFraction(5, 4), Fixed64.FromFraction(3, 2)) : Vector2d.One;
            foreach (Vector2d reference in new[] { Vector2d.Zero, new Vector2d(-Fixed64.Half, Fixed64.Quarter), compound.CalculateLocalCenterOfMassOffset() })
            {
                Fixed64 mass = (Fixed64)100_000_000;
                Fixed64 expected = Expected(vertices, mass.m_rawValue, reference, ownerOffset, ownerScale, partOffset, partScale, rotation);
                Assert.Equal(expected, part.CalculateMomentOfInertia(mass, reference));
                Assert.Equal(expected, compound.CalculateMomentOfInertia(mass, reference));
            }
        }
    }

    [Fact]
    public void OutOfRangeSemanticCentroid_ShouldAllowTinyMassAndRetainItsOverflowContract()
    {
        Vector2d origin = new((Fixed64)1_600_000_000, Fixed64.Zero);
        Vector2d[] vertices = { origin, origin + Vector2d.Right, origin + Vector2d.Forward };
        Vector2d offset = new((Fixed64)1_000_000_000, Fixed64.Zero);
        Vector2d reference = offset;
        var polygon = new LSPolygonCollider2D(vertices) { LocalOffset = offset };
        Assert.Throws<InvalidOperationException>(() => polygon.CalculateLocalCenterOfMassOffset());
        Assert.Equal(Expected(vertices, 1, reference, offset, Vector2d.One),
            polygon.CalculateMomentOfInertia(Fixed64.FromRaw(1), reference));
        Assert.Throws<InvalidOperationException>(() => polygon.CalculateMomentOfInertia(Fixed64.One, reference));
    }

    [Fact]
    public void RepresentableCentroid_WithUnrepresentableFinalMoment_ShouldSaturate()
    {
        var polygon = new LSPolygonCollider2D(Triangle(false));
        Assert.Equal(Fixed64.MaxValue, polygon.CalculateMomentOfInertia(Fixed64.MaxValue, Vector2d.One * Fixed64.Two));
    }

    [Fact]
    public void CollapsedDetachedPolygonPart_ShouldRetainNominalPointMassBehavior()
    {
        Fixed64 raw = Fixed64.FromRaw(1);
        Vector2d[] vertices = { Vector2d.Zero, new(raw, Fixed64.Zero), new(Fixed64.Zero, raw) };
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(vertices,
            new Vector2d(Fixed64.Two, (Fixed64)(-3)), Fixed64.Zero, Vector2d.One * Fixed64.Half));
        Assert.Equal((Fixed64)34, compound.GetPartCollider(0).CalculateMomentOfInertia(Fixed64.Two, Vector2d.One));
    }

    [Fact]
    public void BodyWithExplicitReference_ShouldRefreshCompletePolygonMomentWhenMassAndScaleChange()
    {
        using var context = Physics2DTestWorld.CreateContext();
        Vector2d[] vertices = Triangle(false);
        var polygon = new LSPolygonCollider2D(vertices);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), polygon)
            { Mass = (Fixed64)1_000_000_000, LocalCenterOfMassOffset = Vector2d.Zero };
        body.Initialize(Vector2d.Zero);
        Assert.Equal(Expected(vertices, body.Mass.m_rawValue, Vector2d.Zero, Vector2d.Zero, Vector2d.One), body.MomentOfInertia);
        body.Mass = (Fixed64)100_000_000;
        Assert.Equal(Expected(vertices, body.Mass.m_rawValue, Vector2d.Zero, Vector2d.Zero, Vector2d.One), body.MomentOfInertia);
        transform.LocalScale = Vector3d.One * Fixed64.Two;
        polygon.Simulate();
        Assert.Equal(Expected(vertices, body.Mass.m_rawValue, Vector2d.Zero, Vector2d.Zero, Vector2d.One * Fixed64.Two), body.MomentOfInertia);
        Assert.Equal(Vector2d.Zero, body.LocalCenterOfMassOffset);
    }

    private static Vector2d[] Triangle(bool clockwise)
    {
        Vector2d[] vertices = { Vector2d.Zero, Vector2d.Right, Vector2d.Forward };
        if (clockwise) Array.Reverse(vertices);
        return vertices;
    }

    private static Fixed64 Expected(Vector2d[] vertices, long mass, Vector2d reference, Vector2d ownerOffset, Vector2d ownerScale,
        Vector2d partOffset = default, Vector2d partScale = default, Fixed64 rotation = default)
    {
        if (partScale == Vector2d.Zero) partScale = Vector2d.One;
        BigInteger q = BigInteger.One << 32, area = 0, firstX = 0, firstY = 0, polar = 0;
        BigInteger[] x = new BigInteger[vertices.Length], y = new BigInteger[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            x[i] = Round((BigInteger)vertices[i].X.m_rawValue * ownerScale.X.m_rawValue * partScale.X.m_rawValue, q * q);
            y[i] = Round((BigInteger)vertices[i].Y.m_rawValue * ownerScale.Y.m_rawValue * partScale.Y.m_rawValue, q * q);
        }
        for (int i = 0; i < vertices.Length; i++)
        {
            int j = (i + 1) % vertices.Length;
            BigInteger cross = x[i] * y[j] - x[j] * y[i];
            area += cross;
            firstX += (x[i] + x[j]) * cross;
            firstY += (y[i] + y[j]) * cross;
            polar += cross * (x[i] * x[i] + x[i] * x[j] + x[j] * x[j] + y[i] * y[i] + y[i] * y[j] + y[j] * y[j]);
        }
        if (area.Sign < 0) { area = -area; firstX = -firstX; firstY = -firstY; polar = -polar; }
        BigInteger cx = Round(firstX, 3 * area), cy = Round(firstY, 3 * area);
        BigInteger centered = polar - 4 * (cx * firstX + cy * firstY) + 6 * area * (cx * cx + cy * cy);
        BigInteger fx = firstX - 3 * area * cx, fy = firstY - 3 * area * cy;
        BigInteger cosine = FixedMath.Cos(rotation).m_rawValue, sine = FixedMath.Sin(rotation).m_rawValue;
        BigInteger dx = ((BigInteger)ownerOffset.X.m_rawValue + partOffset.X.m_rawValue) * ownerScale.X.m_rawValue
            + cx * cosine - cy * sine - reference.X.m_rawValue * q;
        BigInteger dy = ((BigInteger)ownerOffset.Y.m_rawValue + partOffset.Y.m_rawValue) * ownerScale.Y.m_rawValue
            + cx * sine + cy * cosine - reference.Y.m_rawValue * q;
        // Polar inertia is rotation invariant; only centroid/residual vectors
        // use published trigonometric coefficients, retaining their full products.
        BigInteger numerator = centered * q * q + 4 * (dx * (fx * cosine - fy * sine) + dy * (fx * sine + fy * cosine))
            + 6 * area * (dx * dx + dy * dy);
        BigInteger result = Round(mass * numerator, 6 * area * q * q * q * q);
        return Fixed64.FromRaw((long)BigInteger.Min(long.MaxValue, BigInteger.Max(long.MinValue, result)));
    }

    private static BigInteger Round(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int half = (2 * remainder).CompareTo(denominator);
        if (half > 0 || half == 0 && !quotient.IsEven) quotient++;
        return numerator.Sign < 0 ? -quotient : quotient;
    }
}
