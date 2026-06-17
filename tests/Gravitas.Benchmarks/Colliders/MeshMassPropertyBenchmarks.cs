using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using System.Collections.Generic;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshMassPropertyBenchmarks
{
    private Vector3d[] _vertices;
    private int[] _triangles;
    private PhysicsMesh _mesh;

    [Params(1, 8, 16)]
    public int Subdivision { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        CreateSubdividedCube(Subdivision, out _vertices, out _triangles);
        _mesh = new PhysicsMesh(_vertices, _triangles, Vector3d.Zero, FixedQuaternion.Identity);
        _mesh.TryGetClosedVolumeMassProperties(out _, out _);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _vertices = null;
        _triangles = null;
        _mesh = null;
    }

    [Benchmark]
    public MeshVolumeValidationResult BuildAndValidateClosedVolume()
    {
        var mesh = new PhysicsMesh(_vertices, _triangles, Vector3d.Zero, FixedQuaternion.Identity);
        mesh.TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult result);
        return result;
    }

    [Benchmark]
    public Fixed64 CalculateCachedClosedVolumeInertiaTensor()
    {
        Fixed3x3 tensor = _mesh.CalculateInertiaTensor(Fixed64.One);
        return tensor.M11 + tensor.M22 + tensor.M33;
    }

    [Benchmark]
    public Fixed64 CalculateSurfaceApproximationInertiaTensor()
    {
        Fixed3x3 tensor = _mesh.CalculateInertiaTensor(Fixed64.One, MeshInertiaPolicy.SurfaceApproximation);
        return tensor.M11 + tensor.M22 + tensor.M33;
    }

    private static void CreateSubdividedCube(
        int subdivision,
        out Vector3d[] vertices,
        out int[] triangles)
    {
        var vertexLookup = new Dictionary<long, int>();
        var vertexList = new List<Vector3d>();
        var triangleList = new List<int>(6 * subdivision * subdivision * 6);

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
        int a = GetVertex(lookup, vertices, x, y, z, subdivision);
        int b = GetVertex(lookup, vertices, x + 1, y, z, subdivision);
        int c = GetVertex(lookup, vertices, x, y + 1, z, subdivision);
        int d = GetVertex(lookup, vertices, x + 1, y + 1, z, subdivision);

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
        int a = GetVertex(lookup, vertices, x, y, z, subdivision);
        int b = GetVertex(lookup, vertices, x, y + 1, z, subdivision);
        int c = GetVertex(lookup, vertices, x, y, z + 1, subdivision);
        int d = GetVertex(lookup, vertices, x, y + 1, z + 1, subdivision);

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
        int a = GetVertex(lookup, vertices, x, y, z, subdivision);
        int b = GetVertex(lookup, vertices, x + 1, y, z, subdivision);
        int c = GetVertex(lookup, vertices, x, y, z + 1, subdivision);
        int d = GetVertex(lookup, vertices, x + 1, y, z + 1, subdivision);

        if (positive)
        {
            AddTriangle(triangles, a, c, b);
            AddTriangle(triangles, b, c, d);
            return;
        }

        AddTriangle(triangles, a, b, c);
        AddTriangle(triangles, b, d, c);
    }

    private static int GetVertex(
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
}
