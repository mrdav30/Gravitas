using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Configuration;
using SwiftCollections;
using SwiftCollections.Diagnostics;
using System;
using System.Collections.Generic;

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

    public static GravitasWorldContext CreateMixedContext(int extentX, int extentZ, bool clearAllPools = false)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        SwiftThrowHelper.ThrowIfTrue(
            !context.World.TryAddGrid(
                new GridConfiguration(
                    new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                    new Vector3d((Fixed64)extentX, (Fixed64)4, (Fixed64)extentZ)),
                out _),
            message: "Unable to add mixed benchmark GridForge grid.");

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
        SwiftList<SolidBody> bodies)
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

    public static LSMeshCollider CreateDynamicSubdividedConvexCube(
        GravitasWorldContext context,
        Vector3d position,
        int subdivision)
    {
        CreateSubdividedClosedCubeTopology(subdivision, out Vector3d[] vertices, out int[] triangles);
        var collider = new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
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

    public static LSConeCollider CreateDynamicCone(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion rotation)
    {
        var collider = new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) };
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
            CreateStaticCollider(context, new LSSphereCollider(), PositionForGridIndex(i));

        return context.Physics.AssimilatedColliderCount;
    }

    public static int CreateStaticMeshWallLine(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
            CreateStaticCollider(context, CreateVerticalQuadMesh(), new Vector3d(i * DefaultSpacing, 0, 0));

        return context.Physics.AssimilatedColliderCount;
    }

    public static TCollider CreateStaticCollider<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(context, position);
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    public static LSMeshCollider CreateSubdividedVerticalQuadMesh(
        int subdivision,
        MeshColliderMode mode = MeshColliderMode.Convex)
    {
        SwiftThrowHelper.ThrowIfArgument(
            subdivision <= 0,
            nameof(subdivision),
            "Subdivision count must be greater than zero.");

        int width = subdivision + 1;
        var vertices = new Vector3d[width * width];
        var triangles = new int[subdivision * subdivision * 6];

        for (int y = 0; y <= subdivision; y++)
        {
            Fixed64 vertexY = Fixed64.FromFraction(y, subdivision) - Fixed64.Half;
            for (int z = 0; z <= subdivision; z++)
            {
                Fixed64 vertexZ = Fixed64.FromFraction(z, subdivision) - Fixed64.Half;
                vertices[(y * width) + z] = new Vector3d(Fixed64.Zero, vertexY, vertexZ);
            }
        }

        int triangleOffset = 0;
        for (int y = 0; y < subdivision; y++)
        {
            for (int z = 0; z < subdivision; z++)
            {
                int lowerLeft = (y * width) + z;
                int lowerRight = lowerLeft + 1;
                int upperLeft = ((y + 1) * width) + z;
                int upperRight = upperLeft + 1;

                AddTriangle(triangles, ref triangleOffset, lowerLeft, upperLeft, lowerRight);
                AddTriangle(triangles, ref triangleOffset, lowerRight, upperLeft, upperRight);
            }
        }

        return new LSMeshCollider(
            vertices,
            triangles,
            mode,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    public static LSMeshCollider CreateRepeatedSlabClippedProxyOnlyTriangleMesh(int triangleCount)
    {
        SwiftThrowHelper.ThrowIfArgument(
            triangleCount <= 0,
            nameof(triangleCount),
            "Triangle count must be greater than zero.");

        var vertices = new Vector3d[triangleCount * 3];
        var triangles = new int[triangleCount * 3];
        for (int i = 0; i < triangleCount; i++)
        {
            int vertexOffset = i * 3;
            vertices[vertexOffset] = new Vector3d(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.FromFraction(49, 100));
            vertices[vertexOffset + 1] = new Vector3d(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.FromFraction(71, 100));
            vertices[vertexOffset + 2] = new Vector3d(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One);

            triangles[vertexOffset] = vertexOffset;
            triangles[vertexOffset + 1] = vertexOffset + 1;
            triangles[vertexOffset + 2] = vertexOffset + 2;
        }

        return new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    public static int GridExtentForLine(int count) =>
        Math.Max(DefaultGridPadding * 2, count * DefaultSpacing + DefaultGridPadding);

    public static int GridExtentForGrid(int count)
    {
        int rows = (count + DefaultColumns - 1) / DefaultColumns;
        int maxDimension = Math.Max(DefaultColumns, rows) * DefaultSpacing + DefaultGridPadding;
        return Math.Max(DefaultGridPadding * 2, maxDimension);
    }

    public static void CreateSubdividedClosedCubeTopology(
        int subdivision,
        out Vector3d[] vertices,
        out int[] triangles)
    {
        if (subdivision <= 0)
            throw new ArgumentOutOfRangeException(nameof(subdivision), subdivision, "Subdivision count must be greater than zero.");

        var vertexLookup = new Dictionary<long, int>();
        var vertexList = new List<Vector3d>(6 * (subdivision + 1) * (subdivision + 1));
        var triangleList = new List<int>(36 * subdivision * subdivision);

        for (int x = 0; x < subdivision; x++)
        {
            for (int y = 0; y < subdivision; y++)
            {
                AddQuadZ(vertexLookup, vertexList, triangleList, x, y, 0, subdivision, false);
                AddQuadZ(vertexLookup, vertexList, triangleList, x, y, subdivision, subdivision, true);
            }
        }

        for (int y = 0; y < subdivision; y++)
        {
            for (int z = 0; z < subdivision; z++)
            {
                AddQuadX(vertexLookup, vertexList, triangleList, 0, y, z, subdivision, false);
                AddQuadX(vertexLookup, vertexList, triangleList, subdivision, y, z, subdivision, true);
            }
        }

        for (int x = 0; x < subdivision; x++)
        {
            for (int z = 0; z < subdivision; z++)
            {
                AddQuadY(vertexLookup, vertexList, triangleList, x, 0, z, subdivision, false);
                AddQuadY(vertexLookup, vertexList, triangleList, x, subdivision, z, subdivision, true);
            }
        }

        vertices = vertexList.ToArray();
        triangles = triangleList.ToArray();
    }

    private static SolidBody CreateDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        return CreateDynamicBody(context, new LSSphereCollider(), position);
    }

    private static SolidBody CreateDynamicBody(
        GravitasWorldContext context,
        LSCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody(agent, collider)
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

    private static void AddQuadZ(
        Dictionary<long, int> lookup,
        List<Vector3d> vertices,
        List<int> triangles,
        int x,
        int y,
        int z,
        int subdivision,
        bool positive)
    {
        int a = GetSubdividedCubeVertex(lookup, vertices, x, y, z, subdivision);
        int b = GetSubdividedCubeVertex(lookup, vertices, x + 1, y, z, subdivision);
        int c = GetSubdividedCubeVertex(lookup, vertices, x, y + 1, z, subdivision);
        int d = GetSubdividedCubeVertex(lookup, vertices, x + 1, y + 1, z, subdivision);

        if (positive)
        {
            AddTriangle(triangles, a, b, c);
            AddTriangle(triangles, b, d, c);
            return;
        }

        AddTriangle(triangles, a, c, b);
        AddTriangle(triangles, b, c, d);
    }

    private static void AddQuadX(
        Dictionary<long, int> lookup,
        List<Vector3d> vertices,
        List<int> triangles,
        int x,
        int y,
        int z,
        int subdivision,
        bool positive)
    {
        int a = GetSubdividedCubeVertex(lookup, vertices, x, y, z, subdivision);
        int b = GetSubdividedCubeVertex(lookup, vertices, x, y + 1, z, subdivision);
        int c = GetSubdividedCubeVertex(lookup, vertices, x, y, z + 1, subdivision);
        int d = GetSubdividedCubeVertex(lookup, vertices, x, y + 1, z + 1, subdivision);

        if (positive)
        {
            AddTriangle(triangles, a, b, c);
            AddTriangle(triangles, c, b, d);
            return;
        }

        AddTriangle(triangles, a, c, b);
        AddTriangle(triangles, b, c, d);
    }

    private static void AddQuadY(
        Dictionary<long, int> lookup,
        List<Vector3d> vertices,
        List<int> triangles,
        int x,
        int y,
        int z,
        int subdivision,
        bool positive)
    {
        int a = GetSubdividedCubeVertex(lookup, vertices, x, y, z, subdivision);
        int b = GetSubdividedCubeVertex(lookup, vertices, x + 1, y, z, subdivision);
        int c = GetSubdividedCubeVertex(lookup, vertices, x, y, z + 1, subdivision);
        int d = GetSubdividedCubeVertex(lookup, vertices, x + 1, y, z + 1, subdivision);

        if (positive)
        {
            AddTriangle(triangles, a, c, b);
            AddTriangle(triangles, b, c, d);
            return;
        }

        AddTriangle(triangles, a, b, c);
        AddTriangle(triangles, b, d, c);
    }

    private static int GetSubdividedCubeVertex(
        Dictionary<long, int> lookup,
        List<Vector3d> vertices,
        int x,
        int y,
        int z,
        int subdivision)
    {
        long key = ((long)x << 42) | ((long)y << 21) | (uint)z;
        if (lookup.TryGetValue(key, out int index))
            return index;

        Fixed64 fx = Fixed64.FromFraction(x, subdivision) - Fixed64.Half;
        Fixed64 fy = Fixed64.FromFraction(y, subdivision) - Fixed64.Half;
        Fixed64 fz = Fixed64.FromFraction(z, subdivision) - Fixed64.Half;
        index = vertices.Count;
        vertices.Add(new Vector3d(fx, fy, fz));
        lookup.Add(key, index);
        return index;
    }

    private static void AddTriangle(List<int> triangles, int a, int b, int c)
    {
        triangles.Add(a);
        triangles.Add(b);
        triangles.Add(c);
    }

    private static void AddTriangle(int[] triangles, ref int offset, int a, int b, int c)
    {
        triangles[offset++] = a;
        triangles[offset++] = b;
        triangles[offset++] = c;
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
