using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class SolidBodyPointTransformBenchmarks
{
    private GravitasWorldContext _context = null!;
    private SolidBody _ordinaryBody = null!;
    private SolidBody _fullDomainBody = null!;
    private Vector3d _ordinaryLocalPoint;
    private Vector3d _ordinaryWorldPoint;
    private Vector3d _fullDomainLocalPoint;
    private Vector3d _fullDomainWorldPoint;

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(16, clearAllPools: true);
        _ordinaryBody = CreateBody(
            new Vector3d((Fixed64)7, (Fixed64)(-3), (Fixed64)11),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)37, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Half, (Fixed64)4));
        _ordinaryLocalPoint = new Vector3d(Fixed64.Half, (Fixed64)(-2), (Fixed64)3);
        _ordinaryWorldPoint = _ordinaryBody.TransformPoint(_ordinaryLocalPoint);

        _fullDomainBody = CreateBody(
            new Vector3d((Fixed64)(-2_000_000_000), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One));
        _fullDomainLocalPoint = new Vector3d((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);
        _fullDomainWorldPoint = new Vector3d((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    [Benchmark]
    public Vector3d OrdinaryRoundTrip()
    {
        _ordinaryBody.TryTransformPoint(_ordinaryLocalPoint, out Vector3d worldPoint);
        _ordinaryBody.TryInverseTransformPoint(_ordinaryWorldPoint, out Vector3d localPoint);
        return worldPoint + localPoint;
    }

    [Benchmark]
    public Vector3d FullDomainRoundTrip()
    {
        _fullDomainBody.TryTransformPoint(_fullDomainLocalPoint, out Vector3d worldPoint);
        _fullDomainBody.TryInverseTransformPoint(_fullDomainWorldPoint, out Vector3d localPoint);
        return worldPoint + localPoint;
    }

    private SolidBody CreateBody(
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d scale)
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        agent.Transform.LocalScale = scale;
        var body = new SolidBody(agent, new LSSphereCollider())
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return body;
    }
}
