using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Configuration;
using SwiftCollections;
using SwiftCollections.Diagnostics;
using System;

namespace Gravitas.Benchmarks;

internal static class BenchmarkPhysicsScene
{
    private const int DefaultGridPadding = 4;
    private const int DefaultColumns = 8;
    private const int DefaultSpacing = 2;

    public static GravitasWorldContext CreateContext(int gridExtent, bool clearAllPools = false)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools);
        AddGrid(context, gridExtent);
        return context;
    }

    public static int CreateDynamicSphereGrid(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
            CreateDynamicSphere(context, PositionForGridIndex(i));

        return context.Physics.AssimilatedBodyCount;
    }

    public static int CreateDynamicSphereGrid(
        GravitasWorldContext context,
        int count,
        SwiftList<StiffBody> bodies)
    {
        bodies.FastClear();
        for (int i = 0; i < count; i++)
            bodies.Add(CreateDynamicSphere(context, PositionForGridIndex(i)));

        return bodies.Count;
    }

    public static int CreateDynamicSphereLine(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
            CreateDynamicSphere(context, new Vector3d(i * DefaultSpacing, 0, 0));

        return context.Physics.AssimilatedBodyCount;
    }

    public static int CreateOverlappingDynamicSpherePairs(GravitasWorldContext context, int pairCount)
    {
        for (int i = 0; i < pairCount; i++)
        {
            Vector3d position = PositionForGridIndex(i);
            CreateDynamicSphere(context, position);
            CreateDynamicSphere(context, position + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        }

        return context.Physics.AssimilatedBodyCount;
    }

    public static int CreateStaticSphereGrid(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
        {
            Vector3d position = PositionForGridIndex(i);
            var agent = new BenchmarkMatterAgent(context, position);
            var collider = new LSSphereCollider();
            collider.InitializeWithNoBody(agent);
        }

        return context.Physics.AssimilatedColliderCount;
    }

    public static int GridExtentForLine(int count) =>
        Math.Max(DefaultGridPadding * 2, count * DefaultSpacing + DefaultGridPadding);

    public static int GridExtentForGrid(int count)
    {
        int rows = (count + DefaultColumns - 1) / DefaultColumns;
        int maxDimension = Math.Max(DefaultColumns, rows) * DefaultSpacing + DefaultGridPadding;
        return Math.Max(DefaultGridPadding * 2, maxDimension);
    }

    private static StiffBody CreateDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static Vector3d PositionForGridIndex(int index)
    {
        int x = index % DefaultColumns;
        int z = index / DefaultColumns;
        return new Vector3d(x * DefaultSpacing, 0, z * DefaultSpacing);
    }

    private static void AddGrid(GravitasWorldContext context, int extent)
    {
        var configuration = new GridConfiguration(
            new Vector3d(-DefaultGridPadding, -DefaultGridPadding, -DefaultGridPadding),
            new Vector3d(extent, DefaultGridPadding, extent));

        SwiftThrowHelper.ThrowIfTrue(
            !context.World.TryAddGrid(configuration, out _),
            message: "Unable to add benchmark GridForge grid.");
    }
}
