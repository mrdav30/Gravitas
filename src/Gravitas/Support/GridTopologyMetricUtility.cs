using FixedMathSharp;
using GridForge.Grids;
using GridForge.Grids.Topology;
using System.Runtime.CompilerServices;

namespace Gravitas.Support;

internal static class GridTopologyMetricUtility
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 GetMaxCellEdge(VoxelGrid grid)
    {
        GridTopologyMetrics metrics = grid.Configuration.TopologyMetrics;
        if (grid.Configuration.TopologyKind == GridTopologyKind.HexPrism)
            return FixedMath.Max(metrics.CellRadius * (Fixed64)2, metrics.LayerHeight);

        Fixed64 max = FixedMath.Max(metrics.CellWidth, metrics.LayerHeight);
        return FixedMath.Max(max, metrics.CellLength);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 GetPlanarMaxCellEdge(VoxelGrid grid)
    {
        GridTopologyMetrics metrics = grid.Configuration.TopologyMetrics;
        return grid.Configuration.TopologyKind == GridTopologyKind.HexPrism
            ? metrics.CellRadius * (Fixed64)2
            : FixedMath.Max(metrics.CellWidth, metrics.CellLength);
    }

    internal static Fixed64 GetRepresentativeCellEdge(GridWorld world)
    {
        foreach (VoxelGrid grid in world.ActiveGrids)
            if (grid.IsActive)
                return GetMaxCellEdge(grid);

        return GridWorld.DefaultRectangularCellSize;
    }
}
