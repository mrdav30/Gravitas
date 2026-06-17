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
    private StiffBody _discreteBody;
    private StiffBody _continuousBody;

    [GlobalSetup]
    public void Setup()
    {
        _discreteContext = CreateContext();
        CreateStaticWall(_discreteContext);
        _discreteBody = CreateMovingSphere(_discreteContext, ContinuousCollisionMode.Discrete);

        _continuousContext = CreateContext();
        CreateStaticWall(_continuousContext);
        _continuousBody = CreateMovingSphere(_continuousContext, ContinuousCollisionMode.Continuous);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _discreteContext.Dispose();
        _continuousContext.Dispose();
        _discreteContext = null;
        _continuousContext = null;
        _discreteBody = null;
        _continuousBody = null;
    }

    [Benchmark(Baseline = true)]
    public Vector3d DiscreteFastMove()
    {
        ResetBody(_discreteBody);
        _discreteBody.AddLinearImpulse(FastImpulse);
        return _discreteBody.Position3d;
    }

    [Benchmark]
    public Vector3d ContinuousFastMoveAgainstThinWall()
    {
        ResetBody(_continuousBody);
        _continuousBody.AddLinearImpulse(FastImpulse);
        return _continuousBody.Position3d;
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

    private static StiffBody CreateMovingSphere(GravitasWorldContext context, ContinuousCollisionMode mode)
    {
        var agent = new BenchmarkMatterAgent(context, StartPosition);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            ContinuousCollisionMode = mode,
            Mass = Fixed64.One,
            GroundProbeMode = GroundProbeMode.Ray
        };

        body.Initialize(StartPosition, FixedQuaternion.Identity);
        return body;
    }

    private static void ResetBody(StiffBody body) =>
        body.ResetPosition(StartPosition, FixedQuaternion.Identity);
}
