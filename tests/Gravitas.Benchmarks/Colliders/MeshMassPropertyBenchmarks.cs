using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshMassPropertyBenchmarks
{
    private Vector3d[] _vertices;
    private int[] _triangles;
    private PhysicsMesh _mesh;
    private int _scaleTick;

    [Params(1, 8, 16)]
    public int Subdivision { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        BenchmarkPhysicsScene.CreateSubdividedClosedCubeTopology(Subdivision, out _vertices, out _triangles);
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

    [Benchmark]
    public Fixed64 UpdateNonUniformMeshScale()
    {
        Vector3d scale = (_scaleTick++ & 1) == 0
            ? new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4)
            : new Vector3d((Fixed64)3, (Fixed64)2, (Fixed64)5);
        _mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        return _mesh.TotalArea;
    }

    [Benchmark]
    public Fixed64 UpdateNonUniformMeshScaleAndCalculateSurfaceInertia()
    {
        Vector3d scale = (_scaleTick++ & 1) == 0
            ? new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4)
            : new Vector3d((Fixed64)3, (Fixed64)2, (Fixed64)5);
        _mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        Fixed3x3 tensor = _mesh.CalculateInertiaTensor(Fixed64.One, MeshInertiaPolicy.SurfaceApproximation);
        return tensor.M11 + tensor.M22 + tensor.M33;
    }
}
