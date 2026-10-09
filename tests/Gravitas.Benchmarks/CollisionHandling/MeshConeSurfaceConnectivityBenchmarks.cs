using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshConeSurfaceConnectivityBenchmarks
{
    private Vector3d[] _vertices;
    private int[] _triangles;
    private int[] _admitted;
    private PhysicsMesh _mesh;
    private readonly MeshConeSurfaceConnectivity _connectivity = new();

    [Params(16, 64, 256)]
    public int SideSegments { get; set; }

    [Params(false, true)]
    public bool Concave { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int count = SideSegments * 4;
        _vertices = new Vector3d[count + 1];
        _triangles = new int[count * 3];
        _admitted = new int[count];
        for (int i = 0; i < SideSegments; i++)
        {
            int coordinate = -SideSegments + i * 2;
            _vertices[1 + i] = new Vector3d(coordinate, 0, -SideSegments);
            _vertices[1 + SideSegments + i] = new Vector3d(SideSegments, 0, coordinate);
            _vertices[1 + 2 * SideSegments + i] = new Vector3d(-coordinate, 0, SideSegments);
            _vertices[1 + 3 * SideSegments + i] = new Vector3d(-SideSegments, 0, -coordinate);
        }
        // A shallow inward tab keeps the fan embedded while requiring the
        // nonconvex preparation certificate rather than the convex fast path.
        if (Concave) _vertices[1 + 2 * SideSegments + SideSegments / 2] = new Vector3d(0, 0, SideSegments / 2);
        for (int i = 0; i < count; i++)
        {
            _triangles[i * 3 + 1] = i + 1;
            _triangles[i * 3 + 2] = (i + 1) % count + 1;
            _admitted[i] = count - i - 1;
        }
        _mesh = (PhysicsMesh)Prepare();
        AdmittedApexFan();
        if (_connectivity.RegionCount != 1)
            throw new System.InvalidOperationException("Every fan triangle must share the admitted apex.");
    }

    [Benchmark]
    public void AdmittedApexFan() => _connectivity.Build(_mesh, 0,
        new Vector3d(0, -2, 0), FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2, _admitted);

    [Benchmark]
    public object Prepare() => new PhysicsMesh(_vertices, _triangles, Vector3d.Zero,
        FixedQuaternion.Identity, MeshColliderMode.Concave);
}
