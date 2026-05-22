using FixedMathSharp;
using GridForge.Configuration;
using System;

namespace Trailblazer.Benchmarks;

/// <summary>
/// Builds deterministic NavigationChart fixtures for benchmark scenarios.
/// All charts use a 1-unit interval and are registered plus initialized through PathManager.
/// Charts have a flat Y=0 surface layer unless otherwise noted.
/// </summary>
internal static class BenchmarkChartFactory
{
    // -------------------------------------------------------------------------
    // Grid configuration helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns a GridConfiguration large enough to contain a square chart whose cells
    /// span [0, size-1] on X and Z at Y=0.
    /// </summary>
    public static GridConfiguration GridConfigForSquare(int size, int padding = 4)
    {
        int extent = size + padding;
        return new GridConfiguration(
            new Vector3d(-padding, -padding, -padding),
            new Vector3d(extent, extent, extent));
    }

    /// <summary>
    /// Returns a GridConfiguration large enough to contain a corridor of <paramref name="length"/> cells
    /// running along the X axis at Y=0, Z=0.
    /// </summary>
    public static GridConfiguration GridConfigForCorridor(int length, int padding = 4)
    {
        int extent = length + padding;
        return new GridConfiguration(
            new Vector3d(-padding, -padding, -padding),
            new Vector3d(extent, padding, padding));
    }

    /// <summary>
    /// Returns a shallow grid configuration large enough to contain surface-only benchmark charts
    /// placed inside [0, maxXExclusive) and [0, maxZExclusive).
    /// </summary>
    public static GridConfiguration GridConfigForArea(int maxXExclusive, int maxZExclusive, int padding = 4)
    {
        return new GridConfiguration(
            new Vector3d(-padding, -padding, -padding),
            new Vector3d(maxXExclusive + padding, padding, maxZExclusive + padding));
    }

    // -------------------------------------------------------------------------
    // Cache pressure set (unique request key factory)
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns an array of <paramref name="count"/> unique start positions spread across
    /// an already-registered open-plane chart. Used for cache pressure scenarios.
    /// The caller is responsible for registering an open-plane chart that covers these positions.
    /// </summary>
    /// <param name="size">Side length of the registered open plane.</param>
    /// <param name="count">Number of unique positions to generate.</param>
    /// <param name="destination">A fixed destination position used for all requests.</param>
    public static Vector3d[] GenerateUniqueStartPositions(
        int size,
        int count,
        out Vector3d destination,
        Vector3d? origin = null)
    {
        Vector3d minBounds = origin ?? Vector3d.Zero;
        destination = minBounds + new Vector3d(size - 1, 0, size - 1);
        var positions = new Vector3d[count];
        int index = 0;
        for (int z = 0; z < size && index < count; z++)
        {
            for (int x = 0; x < size && index < count; x++)
            {
                // Skip the destination cell itself.
                if (x == size - 1 && z == size - 1)
                    continue;
                positions[index++] = minBounds + new Vector3d(x, 0, z);
            }
        }

        if (index < count)
            throw new InvalidOperationException(
                $"Open plane of size {size} provides only {index} unique positions but {count} were requested.");

        return positions;
    }

    /// <summary>
    /// Returns unique adjacent start/destination pairs inside an already-registered open plane.
    /// Each pair has roughly equivalent route cost, which keeps cache-pressure benchmarks from
    /// mixing eviction overhead with path-length differences.
    /// </summary>
    public static void GenerateAdjacentRequestPairs(
        int size,
        int count,
        Vector3d[] starts,
        Vector3d[] destinations,
        Vector3d? origin = null)
    {
        if (starts == null)
            throw new ArgumentNullException(nameof(starts));

        if (destinations == null)
            throw new ArgumentNullException(nameof(destinations));

        if (starts.Length < count || destinations.Length < count)
            throw new ArgumentException("Start and destination buffers must be at least count elements long.");

        Vector3d minBounds = origin ?? Vector3d.Zero;
        int index = 0;
        for (int z = 0; z < size && index < count; z++)
        {
            for (int x = 0; x < size - 1 && index < count; x++)
            {
                starts[index] = minBounds + new Vector3d(x, 0, z);
                destinations[index] = minBounds + new Vector3d(x + 1, 0, z);
                index++;
            }
        }

        if (index < count)
            throw new InvalidOperationException(
                $"Open plane of size {size} provides only {index} adjacent pairs but {count} were requested.");
    }
}
