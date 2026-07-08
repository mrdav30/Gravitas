using FluentAssertions;
using GridForge.Spatial;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class WorldVoxelIndexOrderingTests
{
    [Fact]
    public void Compare3D_ShouldOrderByGridIdentityThenXyzCoordinates()
    {
        WorldVoxelIndex origin = Create(gridIndex: 1, gridSpawnToken: 10, x: 0, y: 0, z: 0);

        WorldVoxelIndexOrdering.Compare3D(origin, Create(gridIndex: 2, gridSpawnToken: 0, x: -1, y: -1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.Compare3D(origin, Create(gridIndex: 1, gridSpawnToken: 11, x: -1, y: -1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.Compare3D(origin, Create(gridIndex: 1, gridSpawnToken: 10, x: 1, y: -1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.Compare3D(origin, Create(gridIndex: 1, gridSpawnToken: 10, x: 0, y: 1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.Compare3D(origin, Create(gridIndex: 1, gridSpawnToken: 10, x: 0, y: 0, z: 1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.Compare3D(origin, origin).Should().Be(0);
    }

    [Fact]
    public void ComparePlanar_ShouldOrderByGridIdentityThenXzyCoordinates()
    {
        WorldVoxelIndex origin = Create(gridIndex: 1, gridSpawnToken: 10, x: 0, y: 0, z: 0);

        WorldVoxelIndexOrdering.ComparePlanar(origin, Create(gridIndex: 2, gridSpawnToken: 0, x: -1, y: -1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.ComparePlanar(origin, Create(gridIndex: 1, gridSpawnToken: 11, x: -1, y: -1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.ComparePlanar(origin, Create(gridIndex: 1, gridSpawnToken: 10, x: 1, y: -1, z: -1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.ComparePlanar(origin, Create(gridIndex: 1, gridSpawnToken: 10, x: 0, y: -1, z: 1))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.ComparePlanar(origin, Create(gridIndex: 1, gridSpawnToken: 10, x: 0, y: 1, z: 0))
            .Should()
            .BeLessThan(0);
        WorldVoxelIndexOrdering.ComparePlanar(origin, origin).Should().Be(0);
    }

    private static WorldVoxelIndex Create(ushort gridIndex, int gridSpawnToken, int x, int y, int z) =>
        new(3, gridIndex, gridSpawnToken, new VoxelIndex(x, y, z));
}
