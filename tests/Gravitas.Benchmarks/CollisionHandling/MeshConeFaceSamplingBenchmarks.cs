//=======================================================================
// MeshConeFaceSamplingBenchmarks.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshConeFaceSamplingBenchmarks
{
    private PhysicsMesh _mesh;
    private int[] _admitted;
    private FixedQuaternion _rotation;
    private Fixed64 _height, _radius;
    private readonly MeshConeSurfaceConnectivity _connectivity = new();
    private readonly MeshConeFaceRegions _regions = new();
    private readonly MeshConeSurfaceBoundary _boundary = new();

    [Params("Interior", "Boundary", "TiltedInterior", "SubdividedTilted", "SubdividedBoundary")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        bool tilted = Geometry.EndsWith("Tilted", StringComparison.Ordinal) || Geometry == "TiltedInterior";
        bool subdivided = Geometry.StartsWith("Subdivided", StringComparison.Ordinal);
        Fixed64 extent = Geometry.EndsWith("Boundary", StringComparison.Ordinal) ? Fixed64.Half : Fixed64.Two;
        _rotation = tilted ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5)
            : FixedQuaternion.Identity;
        _height = tilted ? (Fixed64)1000 : (Fixed64)4;
        _radius = tilted ? Fixed64.One : Fixed64.Two;
        Vector3d[] corners = { new(-extent,Fixed64.Zero,-extent), new(extent,Fixed64.Zero,-extent),
            new(extent,Fixed64.Zero,extent), new(-extent,Fixed64.Zero,extent) };
        Vector3d[] vertices = corners;
        int[] triangles = { 0,1,2, 0,2,3 };
        if (subdivided)
        {
            const int segments = 8;
            vertices = new Vector3d[segments * 4 + 1];
            triangles = new int[segments * 4 * 3];
            for (int side = 0; side < 4; side++)
            for (int step = 0; step < segments; step++)
            {
                int index = side * segments + step;
                vertices[index + 1] = Vector3d.Lerp(corners[side], corners[(side + 1) % 4], Fixed64.FromFraction(step, segments));
                triangles[index * 3 + 1] = index + 1;
                triangles[index * 3 + 2] = (index + 1) % (segments * 4) + 1;
            }
        }
        _mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        // Every triangle includes the cone's interior center, exactly. The
        // prepared fixture therefore supplies a complete admitted list;
        // measured work is connectivity, orientation and sample reduction.
        _admitted = new int[_mesh.TriangleCount];
        for (int triangle = 0; triangle < _admitted.Length; triangle++) _admitted[triangle] = triangle;
        FaceSamples();
        if (_regions.RegionCount != 1 || _regions.GetSampleCount(0) != 4)
            throw new InvalidOperationException("Fixture must retain one admitted four-sample face region.");
        ExposedBoundary();
        if (Geometry.EndsWith("Boundary", StringComparison.Ordinal) && _boundary.FamilySampleCount == 0)
            throw new InvalidOperationException("Finite interior boundary must admit continuous base families.");
    }

    [Benchmark]
    public int FaceSamples()
    {
        _connectivity.Build(_mesh, 0, Vector3d.Zero, _rotation, _height, _radius, _admitted);
        _regions.Build(_mesh, 0, Vector3d.Zero, _rotation, _height, _radius, _connectivity, _admitted);
        return _regions.GetSampleCount(0);
    }

    [Benchmark]
    public int ExposedBoundary()
    {
        _boundary.Build(_mesh, 0, Vector3d.Zero, _rotation, _height, _radius);
        return _boundary.IsolatedCount + _boundary.FamilySampleCount;
    }
}
