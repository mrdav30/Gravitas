using FixedMathSharp;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Support;

internal enum GridForgeTraversalPaddingMode
{
    MaxCellEdge,
    PlanarMaxCellEdge
}

internal struct GridForgeTraversalState
{
    private readonly GridWorld _world;
    private readonly GridForgeTraversalPaddingMode _paddingMode;
    private ushort _currentGridIndex;
    private Fixed64 _cellEdge;
    private bool _hasCachedGrid;

    public GridForgeTraversalState(GridWorld world, GridForgeTraversalPaddingMode paddingMode)
    {
        _world = world;
        _paddingMode = paddingMode;
        _currentGridIndex = 0;
        _cellEdge = Fixed64.Zero;
        _hasCachedGrid = false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryVisitUnique(Voxel voxel, SwiftHashSet<int> visited, out Fixed64 cellEdge)
    {
        cellEdge = Fixed64.Zero;
        if (!visited.Add(voxel.SpawnToken))
            return false;

        cellEdge = GetCellEdge(voxel);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetCellEdge(Voxel voxel)
    {
        if (_hasCachedGrid && voxel.GridIndex == _currentGridIndex)
            return _cellEdge;

        _currentGridIndex = voxel.GridIndex;
        _hasCachedGrid = true;
        VoxelGrid grid = _world.ActiveGrids[_currentGridIndex];
        _cellEdge = _paddingMode == GridForgeTraversalPaddingMode.PlanarMaxCellEdge
            ? GridTopologyMetricUtility.GetPlanarMaxCellEdge(grid)
            : GridTopologyMetricUtility.GetMaxCellEdge(grid);
        return _cellEdge;
    }
}

internal static class GridForgeTraversal
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetUniquePartition<TPartition>(
        Voxel voxel,
        SwiftHashSet<int> visited,
        out TPartition? partition)
        where TPartition : class, IVoxelPartition
    {
        partition = null;
        return visited.Add(voxel.SpawnToken)
            && voxel.TryGetPartition(out partition);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsWorldPositionInPaddedBounds(
        Vector3d min,
        Vector3d max,
        Fixed64 cellEdge,
        Vector3d worldPosition)
    {
        Fixed64 padding = cellEdge * Fixed64.Half;
        return worldPosition.X >= min.X - padding
            && worldPosition.X <= max.X + padding
            && worldPosition.Y >= min.Y - padding
            && worldPosition.Y <= max.Y + padding
            && worldPosition.Z >= min.Z - padding
            && worldPosition.Z <= max.Z + padding;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPlanarPositionInPaddedBounds(
        Vector2d min,
        Vector2d max,
        Fixed64 cellEdge,
        Vector3d worldPosition)
    {
        Fixed64 padding = cellEdge * Fixed64.Half;
        return worldPosition.X >= min.X - padding
            && worldPosition.X <= max.X + padding
            && worldPosition.Z >= min.Y - padding
            && worldPosition.Z <= max.Y + padding;
    }
}
