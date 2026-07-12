using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSPolygonCollider2DCoverageTests
{
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
        inertia.m_rawValue.Should().Be(47721858817L);
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
        Vector2d expectedCentroid = localOffset * Fixed64.Epsilon;

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
}
