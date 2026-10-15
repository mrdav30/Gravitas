using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Queries;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class FiniteSlabProjectionSweepBenchmarks
{
    private GravitasWorldContext _context;
    private LSCapsuleCollider _capsule;
    private LSCylinderCollider _cylinder;
    private LSConeCollider _cone;
    private Vector2d _start;
    private Vector2d _end;
    private Fixed64 _length;
    private Fixed64 _slabY;

    [Params("Capsule", "Cylinder", "Cone")]
    public string Shape { get; set; }

    [Params(1, 200, 500)]
    public int Scale { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkEnvironment.PrepareOwnedContext();
        Fixed64 radius = (Fixed64)(5 * Scale);
        LSCollider collider;
        if (Shape == "Capsule")
            collider = _capsule = new LSCapsuleCollider();
        else if (Shape == "Cylinder")
            collider = _cylinder = new LSCylinderCollider();
        else
            collider = _cone = new LSConeCollider();

        collider.Radius = radius;
        // Keep the nonzero core/height fixed while testing different radial scales.
        collider.Size = new Vector3d(radius * Fixed64.Two,
            _capsule != null ? radius * Fixed64.Two + Fixed64.Two : Fixed64.Two,
            radius * Fixed64.Two);
        var agent = new BenchmarkMatterAgent(_context, Vector3d.Zero);
        agent.Transform.LocalRotation = new FixedQuaternion(Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        collider.InitializeWithNoBody(agent);

        Fixed64 chordX = Fixed64.Zero;
        if (_cone != null)
        {
            if (!FixedSegment.TryGetCenteredAxisEndpoint(_cone.Center, _cone.WorldAxis,
                    _cone.Height, false, out Vector3d baseCenter))
                throw new InvalidOperationException("The cone base must be representable.");
            chordX = baseCenter.X;
        }

        _start = new Vector2d(chordX, -radius);
        _end = new Vector2d(chordX, radius);
        _length = radius * Fixed64.Two;
        _slabY = (Fixed64)(3 * Scale);
        long result = Sweep();
        // The horizontal 3-4-5 slice enters at Scale, independently of GJK.
        if (result == long.MinValue || (Fixed64.FromRaw(result) - (Fixed64)Scale).Abs()
                > Fixed64.FromFraction(1, 1_048_576))
            throw new InvalidOperationException($"Unexpected {Shape} entry: {result}.");
    }

    [Benchmark]
    public long Sweep()
    {
        bool found;
        Fixed64 distance;
        if (_capsule != null)
            found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(_start, _end,
                Vector2d.Forward, _length, Fixed64.Zero, _slabY, _slabY, _capsule, out distance);
        else if (_cylinder != null)
            found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(_start, _end,
                Vector2d.Forward, _length, Fixed64.Zero, _slabY, _slabY, _cylinder, out distance);
        else
            found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(_start, _end,
                Vector2d.Forward, _length, Fixed64.Zero, _slabY, _slabY, _cone, out distance);
        return found ? distance.m_rawValue : long.MinValue;
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
