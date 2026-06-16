using FixedMathSharp;
using GridForge.Configuration;

namespace Gravitas.Benchmarks;

internal static class BenchmarkScenarioFactory
{
    public static GridConfiguration[] CreateTiledFlatGridConfigurations(
        int tilesX,
        int tilesZ,
        int extent,
        int scanCellSize = GridConfiguration.DefaultScanCellSize,
        bool overlapBoundaries = false,
        int originX = 0,
        int originZ = 0)
    {
        GridConfiguration[] configurations = new GridConfiguration[tilesX * tilesZ];
        int step = overlapBoundaries ? extent : extent + 1;
        int index = 0;

        for (int z = 0; z < tilesZ; z++)
        {
            for (int x = 0; x < tilesX; x++)
            {
                int minX = originX + x * step;
                int minZ = originZ + z * step;

                configurations[index++] = new GridConfiguration(
                    new Vector3d(minX, 0, minZ),
                    new Vector3d(minX + extent, 0, minZ + extent),
                    scanCellSize);
            }
        }

        return configurations;
    }

    public static FixedBoundArea[] CreateBlockerAreas(
        int count,
        int span,
        int columns,
        int stride,
        int offset = 4)
    {
        FixedBoundArea[] areas = new FixedBoundArea[count];

        for (int i = 0; i < areas.Length; i++)
        {
            int row = i / columns;
            int column = i % columns;
            int x = offset + column * stride + (row & 1);
            int z = offset + row * stride + (column & 1);

            Vector3d min = new(x, 0, z);
            Vector3d max = new(x + span, 0, z + span);

            areas[i] = new FixedBoundArea(min, max);
        }

        return areas;
    }
}
