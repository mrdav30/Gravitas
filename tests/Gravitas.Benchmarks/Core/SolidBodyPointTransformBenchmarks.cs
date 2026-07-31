using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class SolidBodyPointTransformBenchmarks
{
    private GravitasWorldContext _context = null!;
    private GravitasWorldContext _context2D = null!;
    private SolidBody _ordinaryBody = null!;
    private SolidBody _fullDomainBody = null!;
    private SolidBody2D _ordinaryBody2D = null!;
    private Vector3d _ordinaryLocalPoint;
    private Vector3d _ordinaryWorldPoint;
    private Vector3d _fullDomainLocalPoint;
    private Vector3d _fullDomainWorldPoint;
    private Vector2d _ordinaryLocalPoint2D;
    private Vector2d _ordinaryWorldPoint2D;

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(16, clearAllPools: true);
        _ordinaryBody = CreateBody(
            new Vector3d((Fixed64)7, (Fixed64)(-3), (Fixed64)11),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)37, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Half, (Fixed64)4));
        _ordinaryLocalPoint = new Vector3d(Fixed64.Half, (Fixed64)(-2), (Fixed64)3);
        _ordinaryWorldPoint = _ordinaryBody.GetWorldPoint(_ordinaryLocalPoint);

        _fullDomainBody = CreateBody(
            new Vector3d((Fixed64)(-2_000_000_000), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One));
        _fullDomainLocalPoint = new Vector3d((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);
        _fullDomainWorldPoint = new Vector3d((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);

        _context2D = BenchmarkPhysicsScene.CreateContext(16);
        _context2D.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        _ordinaryBody2D = CreateBody2D(
            new Vector2d(7, 11),
            (Fixed64)37 * Fixed64.Deg2Rad,
            new Vector2d(3, 4));
        _ordinaryLocalPoint2D = new Vector2d(Fixed64.Half, (Fixed64)3);
        _ordinaryWorldPoint2D = _ordinaryBody2D.GetWorldPoint(_ordinaryLocalPoint2D);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context2D.Dispose();
    }

    [Benchmark]
    public Vector3d OrdinaryRoundTrip()
    {
        _ordinaryBody.TryGetWorldPoint(_ordinaryLocalPoint, out Vector3d worldPoint);
        _ordinaryBody.TryGetLocalPoint(_ordinaryWorldPoint, out Vector3d localPoint);
        return worldPoint + localPoint;
    }

    [Benchmark]
    public Vector3d FullDomainRoundTrip()
    {
        _fullDomainBody.TryGetWorldPoint(_fullDomainLocalPoint, out Vector3d worldPoint);
        _fullDomainBody.TryGetLocalPoint(_fullDomainWorldPoint, out Vector3d localPoint);
        return worldPoint + localPoint;
    }

    [Benchmark]
    public Vector2d Ordinary2DRoundTrip()
    {
        _ordinaryBody2D.TryGetWorldPoint(_ordinaryLocalPoint2D, out Vector2d worldPoint);
        _ordinaryBody2D.TryGetLocalPoint(_ordinaryWorldPoint2D, out Vector2d localPoint);
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

    private SolidBody2D CreateBody2D(
        Vector2d position,
        Fixed64 rotation,
        Vector2d scale)
    {
        var agent = new BenchmarkMatterAgent(
            _context2D,
            position.ToVector3d(Fixed64.Zero));
        agent.Transform.LocalScale = new Vector3d(
            scale.X,
            Fixed64.One,
            scale.Y);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.One))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return body;
    }
}
