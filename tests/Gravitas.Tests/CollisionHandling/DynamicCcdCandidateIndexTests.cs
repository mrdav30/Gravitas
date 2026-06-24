using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DynamicCcdCandidateIndexTests
{
    [Fact]
    public void Query_ShouldIncludeTargetsWhoseSweptBoundsOverlapAfterMovement()
    {
        var index = new DynamicCcdCandidateIndex(4);
        index.Add(
            dynamicId: 7,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                -Vector3d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Add(
            dynamicId: 11,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d((Fixed64)5, Fixed64.Zero, (Fixed64)16),
                -Vector3d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right * (Fixed64)5,
                Fixed64.Half),
            results);

        results.Count.Should().Be(1);
        results[0].Should().Be(7);
    }

    [Fact]
    public void Query_ShouldUseStableBoundsThenDynamicIdOrdering()
    {
        var index = new DynamicCcdCandidateIndex(4);
        index.Add(30, CreateBounds(Fixed64.One, Fixed64.Zero));
        index.Add(10, CreateBounds(Fixed64.One, Fixed64.Zero));
        index.Add(20, CreateBounds(-Fixed64.One, Fixed64.Zero));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(new FixedBoundVolume(new Vector3d((Fixed64)(-4), -Fixed64.One, -Fixed64.One), new Vector3d((Fixed64)4, Fixed64.One, Fixed64.One)), results);

        results.Count.Should().Be(3);
        results[0].Should().Be(20);
        results[1].Should().Be(10);
        results[2].Should().Be(30);
    }

    [Fact]
    public void Query2D_ShouldIncludeTargetsWhoseSweptBoundsOverlapAfterMovement()
    {
        var index = new DynamicCcdCandidateIndex2D(4);
        index.Add(
            dynamicId: 7,
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                new Vector2d((Fixed64)5, Fixed64.Zero),
                -Vector2d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Add(
            dynamicId: 11,
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                new Vector2d((Fixed64)5, (Fixed64)16),
                -Vector2d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                new Vector2d((Fixed64)(-5), Fixed64.Zero),
                Vector2d.Right * (Fixed64)5,
                Fixed64.Half),
            results);

        results.Count.Should().Be(1);
        results[0].Should().Be(7);
    }

    [Fact]
    public void Query2D_ShouldUseStableBoundsThenDynamicIdOrdering()
    {
        var index = new DynamicCcdCandidateIndex2D(4);
        index.Add(30, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index.Add(10, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index.Add(20, CreatePlanarBounds(-Fixed64.One, Fixed64.Zero));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(new DynamicCcdPlanarBounds((Fixed64)(-4), -Fixed64.One, (Fixed64)4, Fixed64.One), results);

        results.Count.Should().Be(3);
        results[0].Should().Be(20);
        results[1].Should().Be(10);
        results[2].Should().Be(30);
    }

    [Fact]
    public void CreateSweptCircleBounds_ShouldCreatePlanarBounds()
    {
        DynamicCcdPlanarBounds bounds = DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
            new Vector2d((Fixed64)(-2), Fixed64.One),
            new Vector2d((Fixed64)4, (Fixed64)(-2)),
            Fixed64.Half);

        bounds.MinX.Should().Be(Fixed64.FromFraction(-5, 2));
        bounds.MinZ.Should().Be(Fixed64.FromFraction(-3, 2));
        bounds.MaxX.Should().Be(Fixed64.FromFraction(5, 2));
        bounds.MaxZ.Should().Be(Fixed64.FromFraction(3, 2));
    }

    private static FixedBoundVolume CreateBounds(Fixed64 x, Fixed64 z)
    {
        Vector3d center = new(x, Fixed64.Zero, z);
        Vector3d extents = new(Fixed64.Half, Fixed64.Half, Fixed64.Half);
        return new FixedBoundVolume(center - extents, center + extents);
    }

    private static DynamicCcdPlanarBounds CreatePlanarBounds(Fixed64 x, Fixed64 z)
    {
        return new DynamicCcdPlanarBounds(
            x - Fixed64.Half,
            z - Fixed64.Half,
            x + Fixed64.Half,
            z + Fixed64.Half);
    }
}
