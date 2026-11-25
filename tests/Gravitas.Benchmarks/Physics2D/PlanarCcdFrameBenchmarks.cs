using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

/// <summary>Independent dynamic circle pairs through commands, a complete 2D step and scheduled queries.</summary>
[MemoryDiagnoser]
public class PlanarCcdFrameBenchmarks
{
    private const int Spacing = 16;
    private const int UniformColumns = 16;
    private static readonly Vector2d InitialOffset = new((Fixed64)2, Fixed64.Zero);
    private static readonly Vector2d Impulse = new((Fixed64)4, Fixed64.Zero);
    private GravitasWorldContext _context;
    private SolidBody2D[] _left;
    private SolidBody2D[] _right;
    private Vector2d[] _centers;
    private SwiftList<Physics2DHit> _rayHits;
    private SwiftList<Physics2DHit> _overlapHits;

    [Params(1, 32, 128)]
    public int PairCount { get; set; }

    [Params("Uniform", "ClusteredX")]
    public string Layout { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        if (PairCount < 1 || (Layout != "Uniform" && Layout != "ClusteredX"))
            throw new InvalidOperationException("Unknown planar CCD frame workload.");

        int columns = Layout == "ClusteredX" ? 1 : UniformColumns;
        int rows = (PairCount + columns - 1) / columns;
        _context = BenchmarkPhysicsScene.CreatePlanarContext(new Vector2d(-16, -16),
            new Vector2d(columns * Spacing + 16, rows * Spacing + 16), cellSize: 8);
        _context.SetFrameRate(1);
        _context.Settings.GroundCheckLayerMask = PhysicsLayerMask.None;
        _context.Environment.Gravity = Fixed64.Zero;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        _context.Environment.MaxSpeed = (Fixed64)16;
        _left = new SolidBody2D[PairCount];
        _right = new SolidBody2D[PairCount];
        _centers = new Vector2d[PairCount];
        _rayHits = new SwiftList<Physics2DHit>(2);
        _overlapHits = new SwiftList<Physics2DHit>(2);

        for (int i = 0; i < PairCount; i++)
        {
            Vector2d center = new(i % columns * Spacing, i / columns * Spacing);
            _centers[i] = center;
            _left[i] = CreateBody(center - InitialOffset);
            _right[i] = CreateBody(center + InitialOffset);
        }

        // Repeat real collisions: settled bodies must never make subsequent samples cheaper.
        for (int frame = 0; frame < 24; frame++)
        {
            RunFrame();
            VerifyFrame();
        }
    }

    private SolidBody2D CreateBody(Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new BenchmarkMatterAgent(_context,
            new Vector3d(position.X, Fixed64.Zero, position.Y)), collider)
        {
            Mass = Fixed64.One,
            GravityScale = Fixed64.Zero,
            SleepEnabled = false,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        body.UseManualGrounding();
        body.Initialize(position);
        return body;
    }

    /// <summary>Includes pose/motion reset commands, both fixed-step phases and two post-step queries.</summary>
    [Benchmark]
    public Fixed64 RunFrame()
    {
        for (int i = 0; i < PairCount; i++)
        {
            _left[i].ResetPosition(_centers[i] - InitialOffset);
            _right[i].ResetPosition(_centers[i] + InitialOffset);
            _left[i].AddLinearImpulse(Impulse);
            _right[i].AddLinearImpulse(-Impulse);
        }

        _context.Simulate();
        _context.LateSimulate();
        _context.Query2D.RaycastAll(new Vector2d(-4, 0), new Vector2d(4, 0), _rayHits);
        _context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, _overlapHits);

        Fixed64 result = (Fixed64)(_rayHits.Count + _overlapHits.Count);
        for (int i = 0; i < PairCount; i++)
            result += _left[i].Position.X + (Fixed64)2 * _right[i].Position.X
                + _left[i].LinearVelocity.X + (Fixed64)2 * _right[i].LinearVelocity.X;
        for (int i = 0; i < _rayHits.Count; i++)
            result += _rayHits[i].Distance;
        return result;
    }

    /// <summary>Checks the axis-aligned equal-mass, zero-restitution contact against its analytic result.</summary>
    public void VerifyFrame()
    {
        if (_context.Physics2D.BodyCount != 2 * PairCount || _context.Physics2D.ColliderCount != 2 * PairCount)
            throw new InvalidOperationException("Planar CCD frame lost a dynamic registration.");

        for (int i = 0; i < PairCount; i++)
        {
            SolidBody2D left = _left[i];
            SolidBody2D right = _right[i];
            // Initial separation four, relative speed eight: TOI is 3/8 and centers stop at +/-1/2.
            if ((left.Position.X - (_centers[i].X - Fixed64.Half)).Abs() > Fixed64.Epsilon
                || (right.Position.X - (_centers[i].X + Fixed64.Half)).Abs() > Fixed64.Epsilon
                || left.Position.Y != _centers[i].Y || right.Position.Y != _centers[i].Y
                || left.LinearVelocity != Vector2d.Zero || right.LinearVelocity != Vector2d.Zero
                || left.LastContinuousCollisionToiIterationCount + right.LastContinuousCollisionToiIterationCount == 0
                || left.LastContinuousCollisionToiIterationLimitReached || right.LastContinuousCollisionToiIterationLimitReached)
                throw new InvalidOperationException($"Planar CCD pair {i} did not stop at its shared contact.");
        }

        if (_rayHits.Count != 2 || _rayHits[0].Collider != _left[0].Collider
            || _rayHits[1].Collider != _right[0].Collider
            || (_rayHits[0].Distance - (Fixed64)3).Abs() > Fixed64.Epsilon
            || (_rayHits[1].Distance - (Fixed64)4).Abs() > Fixed64.Epsilon
            || (_rayHits[0].Point - new Vector2d(-1, 0)).Magnitude > Fixed64.Epsilon
            || _rayHits[1].Point.Magnitude > Fixed64.Epsilon
            || _overlapHits.Count != 2
            || _overlapHits[0].Collider != _left[0].Collider || _overlapHits[1].Collider != _right[0].Collider)
            throw new InvalidOperationException("Scheduled planar queries lost the designated pair or physical hit order.");
    }

    [GlobalCleanup]
    public void Cleanup() => _context?.Dispose();
}
