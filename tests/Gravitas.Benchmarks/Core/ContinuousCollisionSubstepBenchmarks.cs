using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using SwiftCollections;
using System;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ContinuousCollisionSubstepBenchmarks
{
    private const int Frames = 16;
    private const int LaneSpacing = 6;
    private static readonly Vector3d VerticalWallSize3D = new(Fixed64.FromFraction(1, 10), (Fixed64)8, (Fixed64)4);
    private static readonly Vector3d HorizontalWallSize3D = new((Fixed64)8, (Fixed64)8, Fixed64.FromFraction(1, 10));
    private static readonly Vector2d VerticalWallSize2D = new(Fixed64.FromFraction(1, 10), (Fixed64)4);
    private static readonly Vector2d HorizontalWallSize2D = new((Fixed64)8, Fixed64.FromFraction(1, 10));

    private GravitasWorldContext _context3D;
    private GravitasWorldContext _context2D;
    private SwiftList<StiffBody> _bodies3D;
    private SwiftList<StiffBody2D> _bodies2D;
    private Vector3d[] _positions3D;
    private Vector2d[] _positions2D;

    [Params(64, 256)]
    public int BodyCount { get; set; }

    [Params(1, 2, 4)]
    public int MaxSubsteps { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extentZ = BodyCount * LaneSpacing + LaneSpacing;
        _context3D = CreateContext3D(extentX: 8, extentZ: extentZ);
        _context2D = CreateContext2D(extentX: 8, extentZ: extentZ);
        _context3D.Settings.ContinuousCollisionMaxSubsteps = MaxSubsteps;
        _context2D.Settings.ContinuousCollisionMaxSubsteps = MaxSubsteps;
        _bodies3D = new SwiftList<StiffBody>(BodyCount);
        _bodies2D = new SwiftList<StiffBody2D>(BodyCount);
        _positions3D = new Vector3d[BodyCount];
        _positions2D = new Vector2d[BodyCount];

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
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context3D.Dispose();
        _context2D.Dispose();
        _context3D = null;
        _context2D = null;
        _bodies3D = null;
        _bodies2D = null;
        _positions3D = null;
        _positions2D = null;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector3d Pure3DStaticTwoContactSubstepCcd()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            Reset3DBodies(_bodies3D, _positions3D, SubstepForce3D);
            _context3D.LateSimulate();
            total += Sum3D(_bodies3D);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = Frames)]
    public Vector2d Pure2DStaticTwoContactSubstepCcd()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < Frames; i++)
        {
            Reset2DBodies(_bodies2D, _positions2D, SubstepForce2D);
            _context2D.LateSimulate();
            total += Sum2D(_bodies2D);
        }

        return total;
    }
}
