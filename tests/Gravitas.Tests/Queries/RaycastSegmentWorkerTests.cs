using FixedMathSharp;
using FluentAssertions;
using Gravitas.Queries;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class RaycastSegmentWorkerTests
{
    [Theory]
    [MemberData(nameof(BoxBoundaryPoints))]
    public void CheckAABBoxOverlaps_WithPointInsideOrOnBoundary_ShouldReturnPoint(Vector3d point)
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(point, point);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(point);
    }

    [Theory]
    [InlineData(-2, 0, 0)]
    [InlineData(0, 2, 0)]
    [InlineData(0, 0, 2)]
    public void CheckAABBoxOverlaps_WithPointOutsideBox_ShouldReturnFalse(int x, int y, int z)
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d point = new((Fixed64)x, (Fixed64)y, (Fixed64)z);

        worker.PrepareSegmentCheck(point, point);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckAABBoxOverlaps_WithPointInsideAndIntersectionsDisabled_ShouldNotWritePoint()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero, calculateIntersectionPoints: false);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    public static TheoryData<Vector3d> BoxBoundaryPoints() => new()
    {
        Vector3d.Zero,
        new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
        new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
        new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
    };
}
