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

    public static LSMeshCollider CreateDynamicConvexCube(GravitasWorldContext context, Vector3d position)
    {
        LSMeshCollider collider = CreateConvexCubeMesh();
        CreateDynamicBody(context, collider, position);
        return collider;
    }

    public static LSCapsuleCollider CreateDynamicCapsule(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion rotation)
    {
        var collider = new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) };
        CreateDynamicBody(context, collider, position, rotation);
        return collider;
    }

    public static LSCuboidCollider CreateDynamicCuboid(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSCuboidCollider();
        CreateDynamicBody(context, collider, position);
        return collider;
    }

    public static LSCylinderCollider CreateDynamicCylinder(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion rotation)
    {
        var collider = new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) };
        CreateDynamicBody(context, collider, position, rotation);
        return collider;
    }

    public static LSCompoundCollider CreateDynamicConvexMeshCompound(GravitasWorldContext context, Vector3d position)
    {
        CreateConvexCubeTopology(out Vector3d[] vertices, out int[] triangles);
        var collider = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                MeshInertiaPolicy.SurfaceApproximation),
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                MeshInertiaPolicy.SurfaceApproximation));
        CreateDynamicBody(context, collider, position);
        return collider;
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

    public static int CreateStaticMeshWallLine(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
        {
            var agent = new BenchmarkMatterAgent(context, new Vector3d(i * DefaultSpacing, 0, 0));
            LSMeshCollider collider = CreateVerticalQuadMesh();
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
        return CreateDynamicBody(context, new LSSphereCollider(), position);
    }

    private static StiffBody CreateDynamicBody(
        GravitasWorldContext context,
        LSCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, rotation ?? FixedQuaternion.Identity);
        return body;
    }

    private static LSMeshCollider CreateConvexCubeMesh()
    {
        CreateConvexCubeTopology(out Vector3d[] vertices, out int[] triangles);
        return new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static void CreateConvexCubeTopology(out Vector3d[] vertices, out int[] triangles)
    {
        Fixed64 half = Fixed64.Half;
        vertices = new[]
        {
            new Vector3d(-half, -half, -half),
            new Vector3d(half, -half, -half),
            new Vector3d(-half, half, -half),
            new Vector3d(half, half, -half),
            new Vector3d(-half, -half, half),
            new Vector3d(half, -half, half),
            new Vector3d(-half, half, half),
            new Vector3d(half, half, half)
        };
        triangles = new[]
        {
            0, 2, 1, 1, 2, 3,
            4, 5, 6, 5, 7, 6,
            0, 4, 2, 2, 4, 6,
            1, 3, 5, 5, 3, 7,
            0, 1, 4, 1, 5, 4,
            2, 6, 3, 3, 6, 7
        };
    }

    private static LSMeshCollider CreateVerticalQuadMesh()
    {
        var vertices = new[]
        {
            new Vector3d(Fixed64.Zero, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One)
        };
        var triangles = new[] { 0, 1, 2, 2, 1, 3 };
        return new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
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
