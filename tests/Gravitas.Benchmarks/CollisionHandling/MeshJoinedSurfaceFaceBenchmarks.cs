using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Measures finite face preparation, distinguishing pre-admitted triangles from complete owner gathering.</summary>
[MemoryDiagnoser]
public class MeshJoinedSurfaceFaceBenchmarks
{
    private GravitasWorldContext _context;
    private LSMeshCollider _continuous;
    private (LSMeshCollider Collider, int Triangle)[] _members;
    private (LSMeshCollider Collider, int Triangle)[] _continuousMember;
    private readonly MeshSurfaceConnectivity _connectivity = new();
    private readonly MeshCapsuleFaceRegions _continuousFaces = new(), _joinedFaces = new();
    private readonly int[] _triangles = { 0, 1 };
    private FixedQuaternion _rotation;
    private Fixed64 _length;
    private readonly Vector3d _center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);

    [Params("SphereInterior", "AxialInterior", "Angled")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _continuous = Rectangle((Fixed64)(-4), (Fixed64)4);
        _continuousMember = new[] { (_continuous, 0) };
        _members = new[] { (Rectangle((Fixed64)(-4), Fixed64.Zero), 0), (Rectangle(Fixed64.Zero, (Fixed64)4), 0) };
        _rotation = Geometry == "Angled" ? FixedQuaternion.LookRotation(
            new Vector3d(1, 0, 1).Normalized, new Vector3d(1, 1, -1).Normalized) : FixedQuaternion.Identity;
        _length = Geometry == "SphereInterior" ? Fixed64.Zero : Fixed64.Two;
        int continuous = Continuous(), gathered = GatheredContinuous(), joined = Joined();
        if (continuous != joined || continuous != gathered || continuous == 0)
            throw new InvalidOperationException("Equivalent sheets must retain the same finite face sample count.");
    }

    [Benchmark]
    public int Continuous()
    {
        _connectivity.BuildCapsule(_continuous.Mesh, 0, _center, _rotation, _length, Fixed64.One, _triangles);
        _continuousFaces.Build(_continuous.Mesh, 0, _center, _rotation, _length, Fixed64.One, _connectivity);
        int count = _continuousFaces.GetSampleCount(0);
        for (int index = 0; index < count; index++)
            if (!_continuousFaces.TryGetSample(0, index, out _)) throw new InvalidOperationException("Continuous witness must materialize.");
        return count;
    }

    [Benchmark(Baseline = true)]
    public int GatheredContinuous() => Gather(_continuousMember);

    [Benchmark]
    public int Joined() => Gather(_members);

    private int Gather(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> members)
    {
        _joinedFaces.BuildJoined(members, _center, _rotation, _length, Fixed64.One);
        if (_joinedFaces.RegionCount != 1) throw new InvalidOperationException("The finite section must connect across the seam.");
        int count = _joinedFaces.GetSampleCount(0);
        for (int index = 0; index < count; index++)
            if (!_joinedFaces.TryGetJoinedSample(0, index, out _, out _)) throw new InvalidOperationException("Joined witness must materialize.");
        return count;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    private LSMeshCollider Rectangle(Fixed64 left, Fixed64 right)
    {
        var collider = new LSMeshCollider(new[] { new Vector3d(left, Fixed64.Zero, (Fixed64)(-4)),
            new(right, Fixed64.Zero, (Fixed64)(-4)), new(right, Fixed64.Zero, (Fixed64)4), new(left, Fixed64.Zero, (Fixed64)4) },
            new[] { 0, 2, 1, 0, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        collider.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        return collider;
    }
}
