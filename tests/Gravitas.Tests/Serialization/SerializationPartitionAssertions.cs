using FluentAssertions;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;

namespace Gravitas.Tests.Serialization;

internal static class SerializationPartitionAssertions
{
    public static WorldVoxelIndex[] CopyCoordinates(SwiftList<WorldVoxelIndex> coordinates)
    {
        var copy = new WorldVoxelIndex[coordinates.Count];
        for (int i = 0; i < coordinates.Count; i++)
            copy[i] = coordinates[i];

        return copy;
    }

    public static void Primary3DPartitionsShouldContain(
        GravitasWorldContext context,
        SwiftList<WorldVoxelIndex> coordinates,
        int colliderId)
    {
        coordinates.Count.Should().BeGreaterThan(0);
        for (int i = 0; i < coordinates.Count; i++)
            Primary3DPartitionContains(context, coordinates[i], colliderId).Should().BeTrue();
    }

    public static void Primary2DPartitionsShouldContain(
        GravitasWorldContext context,
        SwiftList<WorldVoxelIndex> coordinates,
        int colliderId)
    {
        coordinates.Count.Should().BeGreaterThan(0);
        for (int i = 0; i < coordinates.Count; i++)
            Primary2DPartitionContains(context, coordinates[i], colliderId).Should().BeTrue();
    }

    public static void Mixed3DPartitionsShouldContain(
        GravitasWorldContext context,
        SwiftList<WorldVoxelIndex> coordinates,
        int colliderId)
    {
        coordinates.Count.Should().BeGreaterThan(0);
        for (int i = 0; i < coordinates.Count; i++)
            Mixed3DPartitionContains(context, coordinates[i], colliderId).Should().BeTrue();
    }

    public static void Mixed2DPartitionsShouldContain(
        GravitasWorldContext context,
        SwiftList<WorldVoxelIndex> coordinates,
        int colliderId)
    {
        coordinates.Count.Should().BeGreaterThan(0);
        for (int i = 0; i < coordinates.Count; i++)
            Mixed2DPartitionContains(context, coordinates[i], colliderId).Should().BeTrue();
    }

    public static bool StalePrimary3DPartitionsShouldBeCleared(
        GravitasWorldContext context,
        WorldVoxelIndex[] oldCoordinates,
        SwiftList<WorldVoxelIndex> currentCoordinates,
        int colliderId)
    {
        bool foundStaleCoordinate = false;
        for (int i = 0; i < oldCoordinates.Length; i++)
        {
            WorldVoxelIndex coordinate = oldCoordinates[i];
            if (ContainsCoordinate(currentCoordinates, coordinate))
                continue;

            foundStaleCoordinate = true;
            Primary3DPartitionContains(context, coordinate, colliderId).Should().BeFalse();
        }

        return foundStaleCoordinate;
    }

    public static bool StalePrimary2DPartitionsShouldBeCleared(
        GravitasWorldContext context,
        WorldVoxelIndex[] oldCoordinates,
        SwiftList<WorldVoxelIndex> currentCoordinates,
        int colliderId)
    {
        bool foundStaleCoordinate = false;
        for (int i = 0; i < oldCoordinates.Length; i++)
        {
            WorldVoxelIndex coordinate = oldCoordinates[i];
            if (ContainsCoordinate(currentCoordinates, coordinate))
                continue;

            foundStaleCoordinate = true;
            Primary2DPartitionContains(context, coordinate, colliderId).Should().BeFalse();
        }

        return foundStaleCoordinate;
    }

    public static bool StaleMixed3DPartitionsShouldBeCleared(
        GravitasWorldContext context,
        WorldVoxelIndex[] oldCoordinates,
        SwiftList<WorldVoxelIndex> currentCoordinates,
        int colliderId)
    {
        bool foundStaleCoordinate = false;
        for (int i = 0; i < oldCoordinates.Length; i++)
        {
            WorldVoxelIndex coordinate = oldCoordinates[i];
            if (ContainsCoordinate(currentCoordinates, coordinate))
                continue;

            foundStaleCoordinate = true;
            Mixed3DPartitionContains(context, coordinate, colliderId).Should().BeFalse();
        }

        return foundStaleCoordinate;
    }

    public static bool StaleMixed2DPartitionsShouldBeCleared(
        GravitasWorldContext context,
        WorldVoxelIndex[] oldCoordinates,
        SwiftList<WorldVoxelIndex> currentCoordinates,
        int colliderId)
    {
        bool foundStaleCoordinate = false;
        for (int i = 0; i < oldCoordinates.Length; i++)
        {
            WorldVoxelIndex coordinate = oldCoordinates[i];
            if (ContainsCoordinate(currentCoordinates, coordinate))
                continue;

            foundStaleCoordinate = true;
            Mixed2DPartitionContains(context, coordinate, colliderId).Should().BeFalse();
        }

        return foundStaleCoordinate;
    }

    private static bool ContainsCoordinate(SwiftList<WorldVoxelIndex> coordinates, WorldVoxelIndex coordinate)
    {
        for (int i = 0; i < coordinates.Count; i++)
        {
            if (coordinates[i] == coordinate)
                return true;
        }

        return false;
    }

    public static bool Primary3DPartitionContains(
        GravitasWorldContext context,
        WorldVoxelIndex coordinate,
        int colliderId)
    {
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        if (!voxel!.TryGetPartition(out PhysicsPartition? partition))
            return false;

        return partition!.ContainedDynamicObjects?.Contains(colliderId) == true
            || partition.ContainedKinematicObjects?.Contains(colliderId) == true
            || partition.ContainedStaticObjects?.Contains(colliderId) == true;
    }

    public static bool Primary2DPartitionContains(
        GravitasWorldContext context,
        WorldVoxelIndex coordinate,
        int colliderId)
    {
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        if (!voxel!.TryGetPartition(out PhysicsPartition2D? partition))
            return false;

        return partition!.ContainedDynamicObjects?.Contains(colliderId) == true
            || partition.ContainedKinematicObjects?.Contains(colliderId) == true
            || partition.ContainedStaticObjects?.Contains(colliderId) == true;
    }

    public static bool Mixed3DPartitionContains(
        GravitasWorldContext context,
        WorldVoxelIndex coordinate,
        int colliderId)
    {
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        if (!voxel!.TryGetPartition(out PhysicsMixedPartition? partition))
            return false;

        return partition!.ContainedDynamic3DObjects?.Contains(colliderId) == true
            || partition.ContainedKinematic3DObjects?.Contains(colliderId) == true
            || partition.ContainedStatic3DObjects?.Contains(colliderId) == true;
    }

    public static bool Mixed2DPartitionContains(
        GravitasWorldContext context,
        WorldVoxelIndex coordinate,
        int colliderId)
    {
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        if (!voxel!.TryGetPartition(out PhysicsMixedPartition? partition))
            return false;

        return partition!.ContainedDynamic2DObjects?.Contains(colliderId) == true
            || partition.ContainedKinematic2DObjects?.Contains(colliderId) == true
            || partition.ContainedStatic2DObjects?.Contains(colliderId) == true;
    }
}
