using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class InertiaTensorBenchmarks
{
    private Fixed3x3 _diagonalTensor;
    private Fixed3x3 _fullTensor;
    private Vector3d _parallelAxisOffset;
    private GravitasWorldContext _context;
    private LSCompoundCollider _compound;

    [Params(1024)]
    public int Iterations { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _diagonalTensor = new Fixed3x3(
            (Fixed64)2, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, (Fixed64)4, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)8);
        _fullTensor = new Fixed3x3(
            (Fixed64)4, Fixed64.One, Fixed64.Half,
            Fixed64.One, (Fixed64)3, Fixed64.FromFraction(1, 4),
            Fixed64.Half, Fixed64.FromFraction(1, 4), (Fixed64)2);
        _parallelAxisOffset = new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3);
        _context = BenchmarkPhysicsScene.CreateContext(8);
        _compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.One,
                -Vector3d.Right),
            CompoundColliderPart.Cuboid(
                new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
                Vector3d.Right));
        var body = new SolidBody(
            new BenchmarkMatterAgent(_context, Vector3d.Zero),
            _compound)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
    }

    [GlobalCleanup]
    public void Cleanup() =>
        _context?.Dispose();

    [Benchmark(Baseline = true)]
    public Fixed3x3 InvertDiagonalTensor()
    {
        Fixed3x3 result = Fixed3x3.Zero;
        for (int i = 0; i < Iterations; i++)
            result += InertiaTensorMath.InvertForSolver(_diagonalTensor);

        return result;
    }

    [Benchmark]
    public Fixed3x3 InvertFullTensor()
    {
        Fixed3x3 result = Fixed3x3.Zero;
        for (int i = 0; i < Iterations; i++)
            result += InertiaTensorMath.InvertForSolver(_fullTensor);

        return result;
    }

    [Benchmark]
    public Fixed3x3 AddFullParallelAxisTensor()
    {
        Fixed3x3 result = Fixed3x3.Zero;
        for (int i = 0; i < Iterations; i++)
            result += InertiaTensorMath.AddParallelAxisTensor(_diagonalTensor, Fixed64.One, _parallelAxisOffset);

        return result;
    }

    [Benchmark]
    public Fixed3x3 CalculateCompoundInertiaTensor()
    {
        Fixed3x3 result = Fixed3x3.Zero;
        for (int i = 0; i < Iterations; i++)
        {
            result = _compound.CalculateInertiaTensor(
                Fixed64.One,
                Vector3d.Zero);
        }

        return result;
    }
}
