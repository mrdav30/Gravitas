using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedEmptyDimensionBenchmarks
{
    private const int PopulationSize = 64;
    private GravitasWorldContext _context;
    private SolidBody[] _bodies3D;
    private SolidBody2D[] _bodies2D;
    private LSCuboidCollider[] _floors3D;
    private LSAABBoxCollider2D[] _floors2D;
    private long[] _sleepHeights;

    [Params(false, true)]
    public bool PlanarPopulation { get; set; }

    [Params(false, true)]
    public bool MixedRuntime { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(80);
        _context.Settings.RuntimeMode = MixedRuntime ? PhysicsRuntimeMode.Mixed
            : PlanarPopulation ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        _context.SetFrameRate(16);
        _context.Environment.Gravity = Fixed64.One;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        _bodies3D = new SolidBody[PopulationSize];
        _bodies2D = new SolidBody2D[PopulationSize];
        _floors3D = new LSCuboidCollider[PopulationSize];
        _floors2D = new LSAABBoxCollider2D[PopulationSize];
        _sleepHeights = new long[PopulationSize];

        for (int i = 0; i < PopulationSize; i++)
        {
            Vector3d origin = new(i % 8 * 8, 0, i / 8 * 8);
            if (PlanarPopulation)
            {
                Vector3d floorCenter = origin + Vector3d.Backward * Fixed64.Half;
                var floor = new LSAABBoxCollider2D(new Vector2d(6, 1)) { Material = PhysicsMaterial.Frictionless };
                new SolidBody2D(new BenchmarkMatterAgent(_context, floorCenter), floor)
                    .Initialize(floorCenter.ToVector2d(), motionType: BodyMotionType.Static);
                _floors2D[i] = floor;
                var collider = new LSCircleCollider2D(Fixed64.Half)
                    { LocalOffset = Vector2d.Forward, Material = PhysicsMaterial.Frictionless };
                var body = new SolidBody2D(new BenchmarkMatterAgent(_context, origin), collider)
                    { Mass = Fixed64.One, SleepEnabled = false, Gravity = -Vector2d.Forward };
                body.Initialize(origin.ToVector2d());
                _bodies2D[i] = body;
            }
            else
            {
                Vector3d floorCenter = origin + Vector3d.Down * Fixed64.Half;
                var floor = new LSCuboidCollider { Size = new Vector3d(6, 1, 6), Material = PhysicsMaterial.Frictionless };
                new SolidBody(new BenchmarkMatterAgent(_context, floorCenter), floor)
                    .Initialize(floorCenter, FixedQuaternion.Identity, BodyMotionType.Static);
                _floors3D[i] = floor;
                var collider = new LSSphereCollider
                    { Radius = Fixed64.Half, LocalOffset = Vector3d.Up, Material = PhysicsMaterial.Frictionless };
                var body = new SolidBody(new BenchmarkMatterAgent(_context, origin), collider)
                    { Mass = Fixed64.One, SleepEnabled = false, FreezeAxes = BodyFreezeAxes3D.Rotation };
                body.Initialize(origin, FixedQuaternion.Identity);
                _bodies3D[i] = body;
            }
        }

        for (int frame = 0; frame < 16; frame++) Step();
        for (int i = 0; i < PopulationSize; i++)
        {
            if (PlanarPopulation) { _bodies2D[i].SleepEnabled = true; _bodies2D[i].Sleep(); }
            else { _bodies3D[i].SleepEnabled = true; _bodies3D[i].Sleep(); }
        }
        Step();
        for (int i = 0; i < PopulationSize; i++) _sleepHeights[i] = Height(i).m_rawValue;
        for (int frame = 0; frame < 32; frame++) Step();
        ValidateSupportedSleep();
    }

    [Benchmark]
    public long SupportedSleepStep()
    {
        Step();
        long checksum = 0;
        for (int i = 0; i < PopulationSize; i++)
        {
            bool sleeping = PlanarPopulation ? _bodies2D[i].IsSleeping : _bodies3D[i].IsSleeping;
            checksum += Height(i).m_rawValue ^ (sleeping ? 1L : 0L);
        }
        return checksum;
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        try { ValidateSupportedSleep(); }
        finally { _context.Dispose(); }
    }

    private Fixed64 Height(int i) => PlanarPopulation ? _bodies2D[i].Position.Y : _bodies3D[i].Position3d.Y;

    private void Step() { _context.Simulate(); _context.LateSimulate(); }

    private void ValidateSupportedSleep()
    {
        if (_context.Physics.ColliderCount != (PlanarPopulation ? 0 : PopulationSize * 2)
            || _context.Physics2D.ColliderCount != (PlanarPopulation ? PopulationSize * 2 : 0)
            || _context.MixedCollisions.LastBroadPhaseCandidateCount != 0)
            throw new InvalidOperationException("The opposite dimensional population must remain empty.");
        for (int i = 0; i < PopulationSize; i++)
        {
            bool valid = PlanarPopulation
                ? _bodies2D[i].IsSleeping && _bodies2D[i].IsGrounded && _bodies2D[i].LinearVelocity == Vector2d.Zero
                    && _bodies2D[i].Collider.Bounds.Min.Y > (Fixed64)(i / 8 * 8)
                : _bodies3D[i].IsSleeping && _bodies3D[i].IsGrounded && _bodies3D[i].LinearVelocity == Vector3d.Zero
                    && _bodies3D[i].Collider.Bounds.Min.Y > Fixed64.Zero;
            if (!valid || Height(i).m_rawValue != _sleepHeights[i])
                throw new InvalidOperationException("Query-only support must retain sleeping bodies above their floors.");
            if (PlanarPopulation)
            {
                if (_bodies2D[i].Collider.TryGetCollisionPair(_floors2D[i].Id, out var pair) && pair.Manifold.HasContact)
                    throw new InvalidOperationException("Query-only support acquired physical contact.");
            }
            else if (_bodies3D[i].Collider.TryGetCollisionPair(_floors3D[i].Id, out var pair) && pair.Manifold.HasContact)
                throw new InvalidOperationException("Query-only support acquired physical contact.");
        }
    }
}
