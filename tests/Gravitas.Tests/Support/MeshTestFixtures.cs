using FixedMathSharp;
using Gravitas.Colliders;
using System;
using System.Collections.Generic;

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

    public static LSMeshCollider CreateSubdividedInsideCorner(
        int subdivisions,
        MeshColliderMode mode = MeshColliderMode.Concave,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume)
    {
        ThrowIfSubdivisionInvalid(subdivisions);

        Fixed64 four = (Fixed64)4;
        var vertices = new List<Vector3d>(3 * (subdivisions + 1) * (subdivisions + 1));
        var triangles = new List<int>(18 * subdivisions * subdivisions);

        AddSubdividedQuad(
            vertices,
            triangles,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, four),
            new Vector3d(four, Fixed64.Zero, Fixed64.Zero),
            subdivisions);
        AddSubdividedQuad(
            vertices,
            triangles,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, four, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, four),
            subdivisions);
        AddSubdividedQuad(
            vertices,
            triangles,
            Vector3d.Zero,
            new Vector3d(four, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, four, Fixed64.Zero),
            subdivisions);

        return new LSMeshCollider(vertices.ToArray(), triangles.ToArray(), mode, inertiaPolicy);
    }

    public static LSMeshCollider CreateSubdividedUChannel(
        int subdivisions,
        MeshColliderMode mode = MeshColliderMode.Concave,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume)
    {
        ThrowIfSubdivisionInvalid(subdivisions);

        Fixed64 left = (Fixed64)(-2);
        Fixed64 right = (Fixed64)2;
        Fixed64 height = (Fixed64)2;
        Fixed64 depth = (Fixed64)4;
        var vertices = new List<Vector3d>(3 * (subdivisions + 1) * (subdivisions + 1));
        var triangles = new List<int>(18 * subdivisions * subdivisions);

        AddSubdividedQuad(
            vertices,
            triangles,
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, height, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, depth),
            subdivisions);
        AddSubdividedQuad(
            vertices,
            triangles,
            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, depth),
            new Vector3d(Fixed64.Zero, height, Fixed64.Zero),
            subdivisions);
        AddSubdividedQuad(
            vertices,
            triangles,
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right - left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, height, Fixed64.Zero),
            subdivisions);

        return new LSMeshCollider(vertices.ToArray(), triangles.ToArray(), mode, inertiaPolicy);
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

    private static void ThrowIfSubdivisionInvalid(int subdivisions)
    {
        if (subdivisions <= 0)
            throw new ArgumentOutOfRangeException(nameof(subdivisions), subdivisions, "Subdivision count must be greater than zero.");
    }

    private static void AddSubdividedQuad(
        List<Vector3d> vertices,
        List<int> triangles,
        Vector3d origin,
        Vector3d edgeA,
        Vector3d edgeB,
        int subdivisions)
    {
        for (int a = 0; a < subdivisions; a++)
        {
            Fixed64 a0 = Fixed64.FromFraction(a, subdivisions);
            Fixed64 a1 = Fixed64.FromFraction(a + 1, subdivisions);

            for (int b = 0; b < subdivisions; b++)
            {
                Fixed64 b0 = Fixed64.FromFraction(b, subdivisions);
                Fixed64 b1 = Fixed64.FromFraction(b + 1, subdivisions);
                int p00 = AddVertex(vertices, origin + edgeA * a0 + edgeB * b0);
                int p10 = AddVertex(vertices, origin + edgeA * a1 + edgeB * b0);
                int p01 = AddVertex(vertices, origin + edgeA * a0 + edgeB * b1);
                int p11 = AddVertex(vertices, origin + edgeA * a1 + edgeB * b1);

                AddTriangle(triangles, p00, p10, p01);
                AddTriangle(triangles, p01, p10, p11);
            }
        }
    }

    private static int AddVertex(List<Vector3d> vertices, Vector3d vertex)
    {
        int index = vertices.Count;
        vertices.Add(vertex);
        return index;
    }

    private static void AddTriangle(List<int> triangles, int a, int b, int c)
    {
        triangles.Add(a);
        triangles.Add(b);
        triangles.Add(c);
    }
}
