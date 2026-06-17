using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Tests.Support;

internal static class MeshTestFixtures
{
    public static LSMeshCollider CreateConvexCube(
        MeshColliderMode mode = MeshColliderMode.Convex,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume)
    {
        Fixed64 half = Fixed64.Half;
        var vertices = new[]
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

        return new LSMeshCollider(
            vertices,
            new[]
            {
                0, 2, 1, 1, 2, 3,
                4, 5, 6, 5, 7, 6,
                0, 4, 2, 2, 4, 6,
                1, 3, 5, 5, 3, 7,
                0, 1, 4, 1, 5, 4,
                2, 6, 3, 3, 6, 7
            },
            mode,
            inertiaPolicy);
    }

    public static LSMeshCollider CreateInsideCorner(
        MeshColliderMode mode = MeshColliderMode.Concave,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume)
    {
        Fixed64 four = (Fixed64)4;
        var vertices = new[]
        {
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(four, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, four),
            new Vector3d(four, Fixed64.Zero, four),

            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, four, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, four),
            new Vector3d(Fixed64.Zero, four, four),

            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(four, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, four, Fixed64.Zero),
            new Vector3d(four, four, Fixed64.Zero)
        };

        return new LSMeshCollider(
            vertices,
            new[]
            {
                0, 2, 1, 1, 2, 3,
                4, 5, 6, 6, 5, 7,
                8, 9, 10, 10, 9, 11
            },
            mode,
            inertiaPolicy);
    }

    public static LSMeshCollider CreateUChannel(
        MeshColliderMode mode = MeshColliderMode.Concave,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume)
    {
        Fixed64 left = (Fixed64)(-2);
        Fixed64 right = (Fixed64)2;
        Fixed64 height = (Fixed64)2;
        Fixed64 depth = (Fixed64)4;

        var vertices = new[]
        {
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(left, height, Fixed64.Zero),
            new Vector3d(left, Fixed64.Zero, depth),
            new Vector3d(left, height, depth),

            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right, Fixed64.Zero, depth),
            new Vector3d(right, height, Fixed64.Zero),
            new Vector3d(right, height, depth),

            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(left, height, Fixed64.Zero),
            new Vector3d(right, height, Fixed64.Zero)
        };

        return new LSMeshCollider(
            vertices,
            new[]
            {
                0, 1, 2, 2, 1, 3,
                4, 5, 6, 6, 5, 7,
                8, 9, 10, 10, 9, 11
            },
            mode,
            inertiaPolicy);
    }

    public static LSMeshCollider CreateVerticalQuad(
        Fixed64 x,
        Fixed64 zMin,
        Fixed64 zMax,
        MeshColliderMode mode = MeshColliderMode.Convex,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume)
    {
        Fixed64 minY = Fixed64.Zero;
        Fixed64 maxY = (Fixed64)2;
        var vertices = new[]
        {
            new Vector3d(x, minY, zMin),
            new Vector3d(x, maxY, zMin),
            new Vector3d(x, minY, zMax),
            new Vector3d(x, maxY, zMax)
        };

        return new LSMeshCollider(vertices, new[] { 0, 1, 2, 2, 1, 3 }, mode, inertiaPolicy);
    }
}
