using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using SwiftCollections;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ContinuousCollisionToiIterationBenchmarks
{
    private const int Frames = 16;
    private const int LaneSpacing = 6;
    private static readonly Vector3d VerticalWallSize3D = new(Fixed64.FromFraction(1, 10), (Fixed64)8, (Fixed64)4);
    private static readonly Vector3d HorizontalWallSize3D = new((Fixed64)8, (Fixed64)8, Fixed64.FromFraction(1, 10));
    private static readonly Vector2d VerticalWallSize2D = new(Fixed64.FromFraction(1, 10), (Fixed64)4);
    private static readonly Vector2d HorizontalWallSize2D = new((Fixed64)8, Fixed64.FromFraction(1, 10));

    private GravitasWorldContext _context3D;
    private GravitasWorldContext _context2D;
    private GravitasWorldContext _chain3DContext;
    private GravitasWorldContext _chain2DContext;
    private GravitasWorldContext _mixedChainContext;
    private SwiftList<SolidBody> _bodies3D;
    private SwiftList<SolidBody2D> _bodies2D;
    private SwiftList<SolidBody> _chain3DBodies;
    private SwiftList<SolidBody2D> _chain2DBodies;
    private SwiftList<SolidBody> _mixedChain3DBodies;
    private SwiftList<SolidBody2D> _mixedChain2DBodies;
    private Vector3d[] _positions3D;
    private Vector2d[] _positions2D;
    private Vector3d[] _chain3DPositions;
    private Vector2d[] _chain2DPositions;
    private Vector3d[] _mixedChain3DPositions;
    private Vector2d[] _mixedChain2DPositions;

    [Params(64, 256)]
    public int BodyCount { get; set; }

    [Params(1, 2, 4)]
    public int MaxToiIterations { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extentZ = BodyCount * LaneSpacing + LaneSpacing;
        _context3D = CreateContext3D(extentX: 8, extentZ: extentZ);
        _context2D = CreateContext2D(extentX: 8, extentZ: extentZ);
        _chain3DContext = CreateContext3D(extentX: 8, extentZ: extentZ);
        _chain2DContext = CreateContext2D(extentX: 8, extentZ: extentZ);
        _mixedChainContext = CreateMixedContext(extentX: 8, extentZ: extentZ);
        _context3D.Settings.ContinuousCollisionMaxToiIterations = MaxToiIterations;
        _context2D.Settings.ContinuousCollisionMaxToiIterations = MaxToiIterations;
        _chain3DContext.Settings.ContinuousCollisionMaxToiIterations = MaxToiIterations;
        _chain2DContext.Settings.ContinuousCollisionMaxToiIterations = MaxToiIterations;
        _mixedChainContext.Settings.ContinuousCollisionMaxToiIterations = MaxToiIterations;
        _bodies3D = new SwiftList<SolidBody>(BodyCount);
        _bodies2D = new SwiftList<SolidBody2D>(BodyCount);
        _chain3DBodies = new SwiftList<SolidBody>(BodyCount * 3);
        _chain2DBodies = new SwiftList<SolidBody2D>(BodyCount * 3);
        _mixedChain3DBodies = new SwiftList<SolidBody>(BodyCount * 2);
        _mixedChain2DBodies = new SwiftList<SolidBody2D>(BodyCount);
        _positions3D = new Vector3d[BodyCount];
        _positions2D = new Vector2d[BodyCount];
        _chain3DPositions = new Vector3d[BodyCount * 3];
        _chain2DPositions = new Vector2d[BodyCount * 3];
        _mixedChain3DPositions = new Vector3d[BodyCount * 2];
        _mixedChain2DPositions = new Vector2d[BodyCount];

        for (int i = 0; i < BodyCount; i++)
        {
            Fixed64 laneZ = (Fixed64)(i * LaneSpacing);
            Vector3d position3D = new((Fixed64)(-2), Fixed64.Zero, laneZ);
            Vector2d position2D = position3D.ToVector2d();
            _positions3D[i] = position3D;
            _positions2D[i] = position2D;
            _bodies3D.Add(CreateSphere3D(_context3D, position3D));
            _bodies2D.Add(CreateCircle2D(_context2D, position2D));

            CreateStaticCuboid3D(_context3D, new Vector3d(Fixed64.Zero, Fixed64.Zero, laneZ), VerticalWallSize3D);
            CreateStaticCuboid3D(_context3D, new Vector3d((Fixed64)(-1), Fixed64.Zero, laneZ + (Fixed64)3), HorizontalWallSize3D);
            CreateStaticAabb2D(_context2D, new Vector2d(Fixed64.Zero, laneZ), VerticalWallSize2D);
            CreateStaticAabb2D(_context2D, new Vector2d((Fixed64)(-1), laneZ + (Fixed64)3), HorizontalWallSize2D);
            AddPure3DChain(i, laneZ);
            AddPure2DChain(i, laneZ);
            AddMixedChain(i, laneZ);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context3D.Dispose();
        _context2D.Dispose();
        _chain3DContext.Dispose();
        _chain2DContext.Dispose();
        _mixedChainContext.Dispose();
        _context3D = null;
        _context2D = null;
        _chain3DContext = null;
        _chain2DContext = null;
        _mixedChainContext = null;
        _bodies3D = null;
        _bodies2D = null;
        _chain3DBodies = null;
        _chain2DBodies = null;
        _mixedChain3DBodies = null;
        _mixedChain2DBodies = null;
        _positions3D = null;
        _positions2D = null;
        _chain3DPositions = null;
        _chain2DPositions = null;
        _mixedChain3DPositions = null;
        _mixedChain2DPositions = null;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector3d Pure3DStaticTwoContactToiIterationCcd()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            Reset3DBodies(_bodies3D, _positions3D, ToiIterationForce3D);
            _context3D.LateSimulate();
            total += Sum3D(_bodies3D);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector2d Pure2DStaticTwoContactToiIterationCcd()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            Reset2DBodies(_bodies2D, _positions2D, ToiIterationForce2D);
            _context2D.LateSimulate();
            total += Sum2D(_bodies2D);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector3d Pure3DChainedDynamicIslandCcd()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            ResetPure3DChains();
            _chain3DContext.LateSimulate();
            total += Sum3D(_chain3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector2d Pure2DChainedDynamicIslandCcd()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            ResetPure2DChains();
            _chain2DContext.LateSimulate();
            total += Sum2D(_chain2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector3d MixedChainedDynamicIslandCcd()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            ResetMixedChains();
            _mixedChainContext.LateSimulate();
            total += Sum3D(_mixedChain3DBodies);
            total += Sum2D(_mixedChain2DBodies).ToVector3d(Fixed64.Zero);
        }

        return total;
    }

    private void AddPure3DChain(int chainIndex, Fixed64 laneZ)
    {
        int baseIndex = chainIndex * 3;
        Add3DChainBody(_chain3DContext, _chain3DBodies, _chain3DPositions, baseIndex, new Vector3d((Fixed64)2, Fixed64.Zero, laneZ));
        Add3DChainBody(_chain3DContext, _chain3DBodies, _chain3DPositions, baseIndex + 1, new Vector3d(Fixed64.Zero, Fixed64.Zero, laneZ));
        Add3DChainBody(_chain3DContext, _chain3DBodies, _chain3DPositions, baseIndex + 2, new Vector3d((Fixed64)(-5), Fixed64.Zero, laneZ));
    }

    private void AddPure2DChain(int chainIndex, Fixed64 laneZ)
    {
        int baseIndex = chainIndex * 3;
        Add2DChainBody(_chain2DContext, _chain2DBodies, _chain2DPositions, baseIndex, new Vector2d((Fixed64)2, laneZ));
        Add2DChainBody(_chain2DContext, _chain2DBodies, _chain2DPositions, baseIndex + 1, new Vector2d(Fixed64.Zero, laneZ));
        Add2DChainBody(_chain2DContext, _chain2DBodies, _chain2DPositions, baseIndex + 2, new Vector2d((Fixed64)(-5), laneZ));
    }

    private void AddMixedChain(int chainIndex, Fixed64 laneZ)
    {
        int body3DIndex = chainIndex * 2;
        Add3DChainBody(_mixedChainContext, _mixedChain3DBodies, _mixedChain3DPositions, body3DIndex, new Vector3d((Fixed64)2, Fixed64.Zero, laneZ));
        Add3DChainBody(_mixedChainContext, _mixedChain3DBodies, _mixedChain3DPositions, body3DIndex + 1, new Vector3d((Fixed64)(-5), Fixed64.Zero, laneZ));
        Add2DChainBody(_mixedChainContext, _mixedChain2DBodies, _mixedChain2DPositions, chainIndex, new Vector2d(Fixed64.Zero, laneZ));
    }

    private static void Add3DChainBody(
        GravitasWorldContext context,
        SwiftList<SolidBody> bodies,
        Vector3d[] positions,
        int index,
        Vector3d position)
    {
        SolidBody body = CreateSphere3D(context, position);
        body.UseManualGrounding();
        bodies.Add(body);
        positions[index] = position;
    }

    private static void Add2DChainBody(
        GravitasWorldContext context,
        SwiftList<SolidBody2D> bodies,
        Vector2d[] positions,
        int index,
        Vector2d position)
    {
        SolidBody2D body = CreateCircle2D(context, position);
        bodies.Add(body);
        positions[index] = position;
    }

    private void ResetPure3DChains()
    {
        for (int i = 0; i < BodyCount; i++)
        {
            int baseIndex = i * 3;
            Reset3DChainBody(_chain3DBodies[baseIndex], _chain3DPositions[baseIndex], sleep: true);
            Reset3DChainBody(_chain3DBodies[baseIndex + 1], _chain3DPositions[baseIndex + 1], sleep: true);
            Reset3DChainBody(_chain3DBodies[baseIndex + 2], _chain3DPositions[baseIndex + 2], sleep: false);
            _chain3DBodies[baseIndex + 2].AddForce(Vector3d.Right * (Fixed64)10);
        }
    }

    private void ResetPure2DChains()
    {
        for (int i = 0; i < BodyCount; i++)
        {
            int baseIndex = i * 3;
            Reset2DChainBody(_chain2DBodies[baseIndex], _chain2DPositions[baseIndex], sleep: true);
            Reset2DChainBody(_chain2DBodies[baseIndex + 1], _chain2DPositions[baseIndex + 1], sleep: true);
            Reset2DChainBody(_chain2DBodies[baseIndex + 2], _chain2DPositions[baseIndex + 2], sleep: false);
            _chain2DBodies[baseIndex + 2].AddForce(Vector2d.Right * (Fixed64)10);
        }
    }

    private void ResetMixedChains()
    {
        for (int i = 0; i < BodyCount; i++)
        {
            int body3DIndex = i * 2;
            Reset3DChainBody(_mixedChain3DBodies[body3DIndex], _mixedChain3DPositions[body3DIndex], sleep: true);
            Reset3DChainBody(_mixedChain3DBodies[body3DIndex + 1], _mixedChain3DPositions[body3DIndex + 1], sleep: false);
            Reset2DChainBody(_mixedChain2DBodies[i], _mixedChain2DPositions[i], sleep: true);
            _mixedChain3DBodies[body3DIndex + 1].AddForce(Vector3d.Right * (Fixed64)10);
        }
    }

    private static void Reset3DChainBody(SolidBody body, Vector3d position, bool sleep)
    {
        body.ResetPosition(position, FixedQuaternion.Identity);
        if (sleep)
            body.Sleep();
    }

    private static void Reset2DChainBody(SolidBody2D body, Vector2d position, bool sleep)
    {
        body.SetPosition(position);
        if (sleep)
            body.Sleep();
    }
}
