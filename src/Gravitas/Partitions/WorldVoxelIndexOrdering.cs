//=======================================================================
// WorldVoxelIndexOrdering.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using GridForge.Spatial;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal static class WorldVoxelIndexOrdering
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Compare3D(WorldVoxelIndex left, WorldVoxelIndex right)
    {
        int compare = CompareGrid(left, right);
        if (compare != 0)
            return compare;

        compare = left.VoxelIndex.x.CompareTo(right.VoxelIndex.x);
        if (compare != 0)
            return compare;

        compare = left.VoxelIndex.y.CompareTo(right.VoxelIndex.y);
        if (compare != 0)
            return compare;

        return left.VoxelIndex.z.CompareTo(right.VoxelIndex.z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ComparePlanar(WorldVoxelIndex left, WorldVoxelIndex right)
    {
        int compare = CompareGrid(left, right);
        if (compare != 0)
            return compare;

        compare = left.VoxelIndex.x.CompareTo(right.VoxelIndex.x);
        if (compare != 0)
            return compare;

        compare = left.VoxelIndex.z.CompareTo(right.VoxelIndex.z);
        if (compare != 0)
            return compare;

        return left.VoxelIndex.y.CompareTo(right.VoxelIndex.y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareGrid(WorldVoxelIndex left, WorldVoxelIndex right)
    {
        int compare = left.GridIndex.CompareTo(right.GridIndex);
        if (compare != 0)
            return compare;

        return left.GridSpawnToken.CompareTo(right.GridSpawnToken);
    }
}
