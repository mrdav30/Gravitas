using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ContinuousCollisionBenchmarks
{
    private static readonly PhysicsLayerMask NoGround = PhysicsLayerMask.None;
    private static readonly Vector3d StartPosition = new((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
    private static readonly Vector3d FastImpulse = new((Fixed64)4, Fixed64.Zero, Fixed64.Zero);
    private static readonly Vector3d WallPosition = Vector3d.Zero;
    private static readonly Vector3d WallSize = new(Fixed64.FromFraction(1, 10), (Fixed64)8, (Fixed64)8);

    private GravitasWorldContext _discreteContext;
    private GravitasWorldContext _continuousContext;
    private GravitasWorldContext _dynamicContext;
    private GravitasWorldContext _meshContext;
    private SolidBody _discreteBody;
    private SolidBody _continuousBody;
    private SolidBody _dynamicLeftBody;
    private SolidBody _dynamicRightBody;
    private SolidBody _meshBody;

    [GlobalSetup]
    public void Setup()
    {
        _discreteContext = CreateContext();
        CreateStaticWall(_discreteContext);
        _discreteBody = CreateMovingSphere(_discreteContext, ContinuousCollisionMode.Discrete);

        _continuousContext = CreateContext();
        CreateStaticWall(_continuousContext);
        _continuousBody = CreateMovingSphere(_continuousContext, ContinuousCollisionMode.Continuous);

        _dynamicContext = CreateContext();
        _dynamicLeftBody = CreateMovingSphere(_dynamicContext, ContinuousCollisionMode.Continuous, new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        _dynamicRightBody = CreateMovingSphere(_dynamicContext, ContinuousCollisionMode.Continuous, new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));

        _meshContext = CreateContext();
        CreateStaticMeshWall(_meshContext);
        _meshBody = CreateMovingSphere(_meshContext, ContinuousCollisionMode.Continuous);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _discreteContext.Dispose();
        _continuousContext.Dispose();
        _dynamicContext.Dispose();
        _meshContext.Dispose();
        _discreteContext = null;
        _continuousContext = null;
        _dynamicContext = null;
        _meshContext = null;
        _discreteBody = null;
        _continuousBody = null;
        _dynamicLeftBody = null;
        _dynamicRightBody = null;
        _meshBody = null;
    }

    [Benchmark(Baseline = true)]
    public Vector3d DiscreteFastMove()
    {
        ResetBody(_discreteBody);
        _discreteBody.AddLinearImpulse(FastImpulse);
        _discreteContext.LateSimulate();
        return _discreteBody.Position3d;
    }

    [Benchmark]
    public Vector3d ContinuousFastMoveAgainstThinWall()
    {
        ResetBody(_continuousBody);
        _continuousBody.AddLinearImpulse(FastImpulse);
        _continuousContext.LateSimulate();
        return _continuousBody.Position3d;
    }

    [Benchmark]
    public Vector3d ContinuousOpposingDynamicSpheres()
    {
        ResetBody(_dynamicLeftBody, new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ResetBody(_dynamicRightBody, new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));
        _dynamicLeftBody.AddForce(Vector3d.Right * (Fixed64)5);
        _dynamicRightBody.AddForce(-Vector3d.Right * (Fixed64)5);
        _dynamicContext.LateSimulate();
        return _dynamicLeftBody.Position3d + _dynamicRightBody.Position3d;
    }

    [Benchmark]
    public Vector3d ContinuousFastMoveAgainstPositionFrozenMesh()
    {
        ResetBody(_meshBody);
        _meshBody.AddLinearImpulse(FastImpulse);
        _meshContext.LateSimulate();
        return _meshBody.Position3d;
    }

    private static GravitasWorldContext CreateContext()
    {
        GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(8);
        context.SetFrameRate(1);
        context.Settings.GroundCheckLayerMask = NoGround;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
        return context;
    }

    private static void CreateStaticWall(GravitasWorldContext context)
    {
        var agent = new BenchmarkMatterAgent(context, WallPosition);
        var collider = new LSCuboidCollider
        {
            Size = WallSize
        };

        collider.InitializeWithNoBody(agent);
    }

    private static void CreateStaticMeshWall(GravitasWorldContext context)
    {
        var vertices = new[]
        {
            new Vector3d(Fixed64.Zero, (Fixed64)(-4), (Fixed64)(-4)),
            new Vector3d(Fixed64.Zero, (Fixed64)4, (Fixed64)(-4)),
            new Vector3d(Fixed64.Zero, (Fixed64)(-4), (Fixed64)4),
            new Vector3d(Fixed64.Zero, (Fixed64)4, (Fixed64)4)
        };
        var triangles = new[] { 0, 1, 2, 2, 1, 3 };
        var agent = new BenchmarkMatterAgent(context, WallPosition);
        var collider = new LSMeshCollider(vertices, triangles, MeshColliderMode.Convex, MeshInertiaPolicy.SurfaceApproximation);
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            GroundProbeMode = GroundProbeMode.Ray
        };

        body.Initialize(WallPosition, FixedQuaternion.Identity, BodyMotionType.Static);
    }

    private static SolidBody CreateMovingSphere(
        GravitasWorldContext context,
        ContinuousCollisionMode mode,
        Vector3d? startPosition = null)
    {
        Vector3d position = startPosition ?? StartPosition;
        var agent = new BenchmarkMatterAgent(context, position);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            ContinuousCollisionMode = mode,
            Mass = Fixed64.One,
            GroundProbeMode = GroundProbeMode.Ray
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static void ResetBody(SolidBody body, Vector3d? position = null) =>
        body.ResetPosition(position ?? StartPosition, FixedQuaternion.Identity);
}
