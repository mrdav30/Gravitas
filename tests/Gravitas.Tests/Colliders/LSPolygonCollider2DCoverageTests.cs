using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSPolygonCollider2DCoverageTests
{
    [Theory]
    [InlineData(false, 1)]
    [InlineData(true, 1)]
    [InlineData(false, 3)]
    [InlineData(true, 3)]
    public void LargeSquare_ShouldRetainRepresentableCenteredMoment(bool clockwise, int massValue)
    {
        Fixed64 half = (Fixed64)128;
        Vector2d translation = new((Fixed64)30_000, (Fixed64)(-40_000));
        Vector2d[] vertices =
        {
            translation + new Vector2d(-half, -half),
            translation + new Vector2d(half, -half),
            translation + new Vector2d(half, half),
            translation + new Vector2d(-half, half)
        };
        if (clockwise)
            System.Array.Reverse(vertices);
        var polygon = new LSPolygonCollider2D(vertices);
        Fixed64 mass = (Fixed64)massValue;
        Fixed64 expected = Fixed64.FromFraction(65_536L * massValue, 6);

        polygon.CalculateLocalCenterOfMassOffset().Should().Be(translation);
        polygon.CalculateMomentOfInertia(mass, translation).Should().Be(expected);
        polygon.CalculateMomentOfInertia(mass, translation + Vector2d.Right).Should().Be(expected + mass);
    }

    [Fact]
    public void LargeSquare_WithSmallMass_ShouldNotRoundDensityToZero()
    {
        Fixed64 half = (Fixed64)128;
        var polygon = new LSPolygonCollider2D(
            new Vector2d(-half, -half),
            new Vector2d(half, -half),
            new Vector2d(half, half),
            new Vector2d(-half, half));

        polygon.CalculateMomentOfInertia(Fixed64.FromRaw(1), Vector2d.Zero)
            .Should().Be(Fixed64.FromRaw(10_923));
    }

    [Fact]
    public void LargeSquare_WithUnrepresentableArea_ShouldStillRetainRepresentableMoment()
    {
        Fixed64 half = (Fixed64)50_000;
        var polygon = new LSPolygonCollider2D(
            new Vector2d(-half, -half),
            new Vector2d(half, -half),
            new Vector2d(half, half),
            new Vector2d(-half, half));

        // Round (100000^2 / 6) * 2^32 once; FromFraction converts through double.
        polygon.CalculateMomentOfInertia(Fixed64.One, Vector2d.Zero)
            .Should().Be(Fixed64.FromRaw(7_158_278_826_666_666_667));
        polygon.CalculateMomentOfInertia((Fixed64)2, Vector2d.Zero)
            .Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void ScaledCompoundPolygon_ShouldMatchEquivalentAabbAndRetainMomentAcrossPoses()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };
        Vector2d offset = new((Fixed64)5, (Fixed64)(-7));
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(vertices, offset, Fixed64.Zero,
                new Vector2d((Fixed64)128, (Fixed64)64)));
        var body = new SolidBody2D(new TestMatterAgent(context), compound) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        var aabb = new LSAABBoxCollider2D(new Vector2d((Fixed64)256, (Fixed64)128));
        Fixed64 expected = aabb.CalculateMomentOfInertia(Fixed64.One, Vector2d.Zero);

        body.LocalCenterOfMassOffset.Should().Be(offset);
        (body.MomentOfInertia - expected).Abs().m_rawValue.Should().BeLessThanOrEqualTo(1);
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.SetPosition(Vector2d.Right);
            body.SetRotation(Fixed64.PiOver4 * Fixed64.Two);
            body.SetPosition(Vector2d.Zero);
            body.SetRotation(Fixed64.Zero);
        });
        allocated.Should().Be(0);
        (body.MomentOfInertia - expected).Abs().m_rawValue.Should().BeLessThanOrEqualTo(1);

        body.Mass = (Fixed64)3;
        body.MomentOfInertia.Should().Be((Fixed64)20_480);

        body.Agent.Transform.LocalScale = Vector3d.One * Fixed64.Two;
        compound.Simulate();
        body.LocalCenterOfMassOffset.Should().Be(offset * Fixed64.Two);
        body.MomentOfInertia.Should().Be((Fixed64)81_920);
        var committedBounds = compound.Bounds;
        body.Agent.Transform.LocalScale = Vector3d.Zero;
        System.Action rejectScale = () => compound.Simulate();
        rejectScale.Should().Throw<System.ArgumentException>();
        compound.Bounds.Should().Be(committedBounds);
        body.MomentOfInertia.Should().Be((Fixed64)81_920);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LargeTranslation_ShouldPreservePolygonCentroidAndInertia(bool clockwise)
    {
        Vector2d[] vertices = clockwise
            ? new[] { Vector2d.Zero, Vector2d.Forward, Vector2d.Right }
            : new[] { Vector2d.Zero, Vector2d.Right, Vector2d.Forward };
        Vector2d translation = new((Fixed64)30000, (Fixed64)30000);
        Vector2d[] translatedVertices =
        {
            vertices[0] + translation,
            vertices[1] + translation,
            vertices[2] + translation
        };
        var origin = new LSPolygonCollider2D(vertices);
        var translated = new LSPolygonCollider2D(translatedVertices);
        Vector2d originCentroid = origin.CalculateLocalCenterOfMassOffset();
        Vector2d translatedCentroid = translated.CalculateLocalCenterOfMassOffset();
        Vector2d originReference = new((Fixed64)2, (Fixed64)(-3));
        Fixed64 mass = (Fixed64)100;

        translatedCentroid.Should().Be(originCentroid + translation);
        translated.CalculateMomentOfInertia(mass, translatedCentroid).m_rawValue
            .Should().Be(origin.CalculateMomentOfInertia(mass, originCentroid).m_rawValue);
        translated.CalculateMomentOfInertia(mass, originReference + translation).m_rawValue
            .Should().Be(origin.CalculateMomentOfInertia(mass, originReference).m_rawValue);
    }

    [Fact]
    public void LargeCompoundOffset_ShouldKeepPartTranslationInOwnerFrame()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Vector2d[] vertices = { Vector2d.Zero, Vector2d.Right, Vector2d.Forward };
        Vector2d translation = new((Fixed64)30000, (Fixed64)30000);
        Vector2d scale = new((Fixed64)2, (Fixed64)3);
        var originCompound = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(vertices, Vector2d.Zero, Fixed64.Zero, scale));
        var translatedCompound = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(vertices, translation, Fixed64.Zero, scale));
        var originBody = new SolidBody2D(new TestMatterAgent(context), originCompound);
        var translatedBody = new SolidBody2D(new TestMatterAgent(context), translatedCompound);
        originBody.Initialize(Vector2d.Zero);
        translatedBody.Initialize(Vector2d.Zero);
        var origin = (LSPolygonCollider2D)originCompound.GetPartCollider(0);
        var translated = (LSPolygonCollider2D)translatedCompound.GetPartCollider(0);
        Vector2d originCentroid = origin.CalculateLocalCenterOfMassOffset();
        Vector2d translatedCentroid = translated.CalculateLocalCenterOfMassOffset();
        Vector2d originReference = new((Fixed64)(-4), (Fixed64)5);
        Fixed64 mass = (Fixed64)100;

        translatedCentroid.Should().Be(originCentroid + translation);
        translated.CalculateMomentOfInertia(mass, translatedCentroid).m_rawValue
            .Should().Be(origin.CalculateMomentOfInertia(mass, originCentroid).m_rawValue);
        translated.CalculateMomentOfInertia(mass, originReference + translation).m_rawValue
            .Should().Be(origin.CalculateMomentOfInertia(mass, originReference).m_rawValue);
    }

    [Fact]
    public void TranslatedPolygon_ShouldCalculateStableCenteredInertia()
    {
        Fixed64 translation = (Fixed64)1000;
        var collider = new LSPolygonCollider2D(
            new Vector2d(translation, translation),
            new Vector2d(translation + Fixed64.One, translation),
            new Vector2d(translation, translation + Fixed64.One));
        Vector2d expectedCentroid = new(
            translation + Fixed64.FromFraction(1, 3),
            translation + Fixed64.FromFraction(1, 3));

        Vector2d centroid = collider.CalculateLocalCenterOfMassOffset();
        Fixed64 inertia = collider.CalculateMomentOfInertia((Fixed64)100, centroid);

        centroid.Should().Be(expectedCentroid);
        inertia.Should().Be(Fixed64.FromFraction(100, 9));
    }

    [Fact]
    public void DegenerateEffectiveScale_ShouldUseOffsetCentroidAndZeroInertia()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Vector2d localOffset = new((Fixed64)2, (Fixed64)3);
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(
                vertices,
                localOffset,
                Fixed64.Zero,
                new Vector2d(Fixed64.Epsilon, Fixed64.Epsilon)));
        var body = new SolidBody2D(new TestMatterAgent(context), compound)
        {
            Mass = (Fixed64)5
        };
        body.Initialize(Vector2d.Zero);
        var polygon = (LSPolygonCollider2D)compound.GetPartCollider(0);
        Vector2d expectedCentroid = localOffset;

        polygon.CalculateLocalCenterOfMassOffset().Should().Be(expectedCentroid);
        polygon.CalculateMomentOfInertia((Fixed64)5, expectedCentroid).Should().Be(Fixed64.Zero);
        polygon.Bounds.Min.Should().Be(expectedCentroid - new Vector2d(Fixed64.Epsilon, Fixed64.Epsilon));
        polygon.Bounds.Max.Should().Be(expectedCentroid + new Vector2d(Fixed64.Epsilon, Fixed64.Epsilon));
    }

    [Fact]
    public void LoadingSameVertexCount_ShouldReplacePolygonGeometryInStableOrder()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSPolygonCollider2D(
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Forward);
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = (Fixed64)6
        };
        body.Initialize(new Vector2d((Fixed64)3, (Fixed64)4));
        Vector2d[] loadedVertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.Zero, (Fixed64)2)
        };
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Vertices"] = loadedVertices
        });

        collider.RecordData(chronicler);

        collider.Count.Should().Be(3);
        collider.GetWorldVertex(0).Should().Be(new Vector2d((Fixed64)2, (Fixed64)3));
        collider.GetWorldVertex(1).Should().Be(new Vector2d((Fixed64)4, (Fixed64)3));
        collider.GetWorldVertex(2).Should().Be(new Vector2d((Fixed64)3, (Fixed64)6));
        collider.Bounds.Min.Should().Be(new Vector2d((Fixed64)2, (Fixed64)3));
        collider.Bounds.Max.Should().Be(new Vector2d((Fixed64)4, (Fixed64)6));
        collider.CalculateLocalCenterOfMassOffset().Should().Be(Vector2d.Zero);
        collider.CalculateMomentOfInertia((Fixed64)6, Vector2d.Zero).Should().Be((Fixed64)4);
    }

    [Fact]
    public void LoadingEmptyVertices_ShouldRetainExistingPolygonGeometry()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSPolygonCollider2D(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, (Fixed64)2));
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(new Vector2d((Fixed64)3, (Fixed64)4));
        Vector2d[] expectedVertices =
        {
            collider.GetWorldVertex(0),
            collider.GetWorldVertex(1),
            collider.GetWorldVertex(2)
        };
        var expectedBounds = collider.Bounds;
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Vertices"] = System.Array.Empty<Vector2d>()
        });

        collider.RecordData(chronicler);

        collider.Count.Should().Be(3);
        collider.GetWorldVertex(0).Should().Be(expectedVertices[0]);
        collider.GetWorldVertex(1).Should().Be(expectedVertices[1]);
        collider.GetWorldVertex(2).Should().Be(expectedVertices[2]);
        collider.Bounds.Should().Be(expectedBounds);
        collider.CalculateLocalCenterOfMassOffset()
            .Should().Be(new Vector2d(Fixed64.FromFraction(2, 3), Fixed64.FromFraction(2, 3)));
    }

    [Fact]
    public void GetClosestPoint_WithSeveralSaturatedDistances_ShouldSelectTheExactNearestEdge()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero);

        Vector2d closest = collider.GetClosestPoint(new Vector2d((Fixed64)200_000, (Fixed64)100_000));

        closest.Should().Be(Vector2d.One);
    }
}
