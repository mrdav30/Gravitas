using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class BodyMotionTypeTransitionBenchmarks
{
    private GravitasWorldContext _context3D = null!;
    private GravitasWorldContext _context2D = null!;
    private SolidBody _body3D = null!;
    private SolidBody2D _body2D = null!;

    [GlobalSetup]
    public void Setup()
    {
        _context3D = BenchmarkPhysicsScene.CreateContext(16, clearAllPools: true);
        _body3D = new SolidBody(
            new BenchmarkMatterAgent(_context3D, Vector3d.Zero),
            new LSSphereCollider())
        {
            Mass = Fixed64.One
        };
        _body3D.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        _context2D = BenchmarkPhysicsScene.CreateContext(16);
        _context2D.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        _body2D = new SolidBody2D(
            new BenchmarkMatterAgent(_context2D, Vector3d.Zero),
            new LSCircleCollider2D(Fixed64.One))
        {
            Mass = Fixed64.One
        };
        _body2D.Initialize(Vector2d.Zero);

        _ = Transition3D();
        _ = Transition2D();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context3D.Dispose();
        _context2D.Dispose();
    }

    [Benchmark]
    public int Transition3D()
    {
        _body3D.SetMotionType(BodyMotionType.Static);
        _body3D.SetMotionType(BodyMotionType.Kinematic);
        _body3D.SetMotionType(BodyMotionType.Dynamic);
        return _body3D.DynamicId;
    }

    [Benchmark]
    public int Transition2D()
    {
        _body2D.SetMotionType(BodyMotionType.Static);
        _body2D.SetMotionType(BodyMotionType.Kinematic);
        _body2D.SetMotionType(BodyMotionType.Dynamic);
        return _body2D.DynamicId;
    }
}
