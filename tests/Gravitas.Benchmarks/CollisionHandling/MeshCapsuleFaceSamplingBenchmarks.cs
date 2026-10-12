using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshCapsuleFaceSamplingBenchmarks
{
    private PhysicsMesh _mesh;
    private int[] _admitted;
    private FixedQuaternion _rotation;
    private Fixed64 _length;
    private readonly MeshSurfaceConnectivity _connectivity = new();
    private readonly MeshCapsuleFaceRegions _faces = new();

    [Params("Axial", "Angled", "Clipped", "SubdividedAngled")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        bool axial = Geometry == "Axial";
        _rotation = axial ? FixedQuaternion.Identity : FixedQuaternion.LookRotation(
            new Vector3d(1, 0, 1).Normalized, new Vector3d(1, 1, -1).Normalized);
        _length = axial ? Fixed64.One : (Fixed64)8;
        Fixed64 extent = Geometry == "Clipped" ? Fixed64.Quarter : Fixed64.Two;
        Vector3d[] vertices = { new(-extent, Fixed64.Zero, -extent), new(extent, Fixed64.Zero, -extent),
            new(extent, Fixed64.Zero, extent), new(-extent, Fixed64.Zero, extent) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 };
        if (Geometry == "SubdividedAngled")
        {
            const int sideSegments = 8;
            Vector3d[] corners = vertices;
            vertices = new Vector3d[4 * sideSegments + 1];
            triangles = new int[12 * sideSegments];
            for (int side = 0; side < 4; side++)
            for (int step = 0; step < sideSegments; step++)
            {
                int index = side * sideSegments + step;
                vertices[index + 1] = Vector3d.Lerp(corners[side], corners[(side + 1) % 4], Fixed64.FromFraction(step, sideSegments));
                triangles[3 * index + 1] = index + 1;
                triangles[3 * index + 2] = (index + 1) % (4 * sideSegments) + 1;
            }
        }
        _mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        _admitted = new int[_mesh.TriangleCount];
        for (int triangle = 0; triangle < _admitted.Length; triangle++) _admitted[triangle] = triangle;
        // Every fixture triangle contains the capsule core's plane crossing.
        if (FaceSamples() != (axial ? 1 : 4))
            throw new InvalidOperationException("The admitted section must retain its physical support or finite coverage.");
    }

    [Benchmark]
    public int FaceSamples()
    {
        _connectivity.BuildCapsule(_mesh, 0, Vector3d.Zero, _rotation, _length, Fixed64.Half, _admitted);
        _faces.Build(_mesh, 0, Vector3d.Zero, _rotation, _length, Fixed64.Half, _connectivity);
        for (int sample = 0; sample < _faces.GetSampleCount(0); sample++)
            if (!_faces.TryGetSample(0, sample, out _)) throw new InvalidOperationException("An admitted ray must materialize.");
        return _faces.GetSampleCount(0);
    }
}
