using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class PolygonMassPropertyBenchmarks
{
    private GravitasWorldContext _context;
    private LSPolygonCollider2D _polygon;
    private Vector2d _center;
    private int _massTick, _referenceTick;

    [Params(false, true)]
    public bool Square { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        _polygon = Square
            ? new LSPolygonCollider2D(Vector2d.Zero, Vector2d.Right, Vector2d.One, Vector2d.Forward)
            : new LSPolygonCollider2D(Vector2d.Zero, Vector2d.Right, Vector2d.Forward);
        _polygon.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        _center = _polygon.CalculateLocalCenterOfMassOffset();
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    [Benchmark]
    public long CalculateCenteredMoment() =>
        _polygon.CalculateMomentOfInertia(Fixed64.One, _center).m_rawValue;

    [Benchmark]
    public long CalculateChangingMassCenteredMoment()
    {
        Fixed64 mass = (_massTick++ & 1) == 0 ? Fixed64.One : Fixed64.Two;
        return _polygon.CalculateMomentOfInertia(mass, _center).m_rawValue;
    }

    [Benchmark]
    public long CalculateReferenceMoment() =>
        _polygon.CalculateMomentOfInertia(Fixed64.One, Vector2d.Zero).m_rawValue;

    [Benchmark]
    public long CalculateChangingMassReferenceMoment()
    {
        Fixed64 mass = (_massTick++ & 1) == 0 ? Fixed64.One : Fixed64.Two;
        return _polygon.CalculateMomentOfInertia(mass, Vector2d.Zero).m_rawValue;
    }

    [Benchmark]
    public long CalculateChangingReferenceMoment()
    {
        Vector2d reference = (_referenceTick++ & 1) == 0
            ? Vector2d.Zero : new Vector2d(Fixed64.Quarter, Fixed64.FromFraction(1, 8));
        return _polygon.CalculateMomentOfInertia(Fixed64.One, reference).m_rawValue;
    }
}
