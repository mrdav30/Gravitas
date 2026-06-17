using FixedMathSharp;
using FluentAssertions;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Grids.Topology;
using GridForge.Spatial;
using Gravitas.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Support;

public sealed class GridForgeTraversalTests
{
    [Fact]
    public void TryVisitUnique_ShouldUseSelectedPaddingModeAndSuppressDuplicates()
    {
        using GravitasWorldContext context = CreateContextWithRectangularGrid(
            GridTopologyMetrics.Rectangular((Fixed64)2, (Fixed64)9, (Fixed64)4));
        Voxel voxel = GetVoxel(context, Vector3d.Zero);
        var visited = new SwiftHashSet<int>();

        var maxTraversal = new GridForgeTraversalState(
            context.World,
            GridForgeTraversalPaddingMode.MaxCellEdge);

        maxTraversal.TryVisitUnique(voxel, visited, out Fixed64 maxPadding).Should().BeTrue();
        maxPadding.Should().Be((Fixed64)9);
        maxTraversal.TryVisitUnique(voxel, visited, out _).Should().BeFalse();

        visited.Clear();
        var planarTraversal = new GridForgeTraversalState(
            context.World,
            GridForgeTraversalPaddingMode.PlanarMaxCellEdge);

        planarTraversal.TryVisitUnique(voxel, visited, out Fixed64 planarPadding).Should().BeTrue();
        planarPadding.Should().Be((Fixed64)4);
    }

    [Fact]
    public void TryGetUniquePartition_ShouldReturnAttachedPartitionOnceWithoutFilteringEmptyPartitions()
    {
        using GravitasWorldContext context = CreateContextWithRectangularGrid(
            GridTopologyMetrics.Rectangular(Fixed64.One));
        Voxel voxel = GetVoxel(context, Vector3d.Zero);
        PhysicsPartition partition = context.Collisions.RentPartition();
        voxel.TryAddPartition(partition).Should().BeTrue();
        var visited = new SwiftHashSet<int>();

        GridForgeTraversal.TryGetUniquePartition(voxel, visited, out PhysicsPartition? resolved).Should().BeTrue();

        resolved.Should().BeSameAs(partition);
        resolved!.IsEmpty.Should().BeTrue();
        GridForgeTraversal.TryGetUniquePartition(voxel, visited, out resolved).Should().BeFalse();
    }

    [Fact]
    public void PaddedBounds_ShouldIncludeNegativeEdgeCoordinatesAndRejectOutsidePositions()
    {
        Vector3d min = new(-3, -2, -4);
        Vector3d max = new(-1, 0, -2);
        Fixed64 cellEdge = (Fixed64)2;
        Fixed64 outside = Fixed64.One / (Fixed64)16;

        GridForgeTraversal.IsWorldPositionInPaddedBounds(
            min,
            max,
            cellEdge,
            new Vector3d(-4, -3, -5)).Should().BeTrue();

        GridForgeTraversal.IsWorldPositionInPaddedBounds(
            min,
            max,
            cellEdge,
            new Vector3d((Fixed64)(-4) - outside, (Fixed64)(-3), (Fixed64)(-5))).Should().BeFalse();

        GridForgeTraversal.IsPlanarPositionInPaddedBounds(
            new Vector2d(-3, -4),
            new Vector2d(-1, -2),
            cellEdge,
            new Vector3d((Fixed64)(-4), Fixed64.Zero, (Fixed64)(-5))).Should().BeTrue();

        GridForgeTraversal.IsPlanarPositionInPaddedBounds(
            new Vector2d(-3, -4),
            new Vector2d(-1, -2),
            cellEdge,
            new Vector3d((Fixed64)(-4), Fixed64.Zero, (Fixed64)(-5) - outside)).Should().BeFalse();
    }

    private static GravitasWorldContext CreateContextWithRectangularGrid(GridTopologyMetrics metrics)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridConfiguration configuration = new(
            new Vector3d(-8, -8, -8),
            new Vector3d(8, 8, 8),
            topologyMetrics: metrics);

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        return context;
    }

    private static Voxel GetVoxel(GravitasWorldContext context, Vector3d position)
    {
        context.World.TryGetVoxel(position, out Voxel? voxel).Should().BeTrue();
        return voxel!;
    }
}
