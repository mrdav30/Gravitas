using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using GridForge.Configuration;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Unchanged remote static population, separate from the moving body's local floor.</summary>
[MemoryDiagnoser]
public class StaticPopulationRefreshBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody2D _planarBody;
    private SolidBody _spatialBody;
    private LSCollider2D[] _planarFloors;
    private LSCollider[] _spatialFloors;
    private Fixed64 _direction;

    [Params("TwoD", "ThreeD", "Mixed")]
    public string Mode { get; set; } = "TwoD";

    [Params(false, true)]
    public bool ExplicitStatic { get; set; }

    [Params(1, 64)]
    public int StaticCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = Mode switch
        {
            "TwoD" => PhysicsRuntimeMode.TwoD,
            "ThreeD" => PhysicsRuntimeMode.ThreeD,
            "Mixed" => PhysicsRuntimeMode.Mixed,
            _ => throw new ArgumentException("Unknown population runtime mode.", nameof(Mode))
        };
        _context.SetFrameRate(16);
        if (!_context.World.TryAddGrid(new GridConfiguration(new(-16, -4, -16), new(288, 4, 16)), out _))
            throw new InvalidOperationException("Unable to load the population world.");
        _context.Environment.Gravity = Mode == "Mixed" ? Fixed64.Zero : Fixed64.One;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        _planarFloors = Mode == "ThreeD" ? Array.Empty<LSCollider2D>() : new LSCollider2D[StaticCount];
        _spatialFloors = Mode == "ThreeD" ? new LSCollider[StaticCount] : Array.Empty<LSCollider>();
        for (int i = 0; i < StaticCount; i++)
        {
            Fixed64 x = i == 0 ? Fixed64.Zero : (Fixed64)(32 + 4 * (i - 1));
            Fixed64 width = i == 0 ? (Fixed64)16 : Fixed64.Two;
            if (Mode == "ThreeD")
            {
                var floor = new LSCuboidCollider { Size = new(width, Fixed64.Two, (Fixed64)4), Material = PhysicsMaterial.Frictionless };
                var agent = new BenchmarkMatterAgent(_context, new(x, -Fixed64.One, Fixed64.Zero));
                if (ExplicitStatic) new SolidBody(agent, floor).Initialize(agent.Transform.WorldPosition, FixedQuaternion.Identity, motionType: BodyMotionType.Static);
                else floor.InitializeWithNoBody(agent);
                _spatialFloors[i] = floor;
                if (!floor.IsPartitioned) throw new InvalidOperationException("A spatial population member is not loaded.");
            }
            else
            {
                var floor = new LSAABBoxCollider2D(new Vector2d(width, Fixed64.Two))
                { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
                var agent = new BenchmarkMatterAgent(_context, new(x, Fixed64.Zero, -Fixed64.One));
                if (ExplicitStatic) new SolidBody2D(agent, floor).Initialize(agent.Transform.WorldPositionXZ, motionType: BodyMotionType.Static);
                else floor.InitializeWithNoBody(agent);
                _planarFloors[i] = floor;
                if (!floor.IsPartitioned) throw new InvalidOperationException("A planar population member is not loaded.");
            }
        }
        if (Mode == "TwoD")
        {
            _planarBody = new SolidBody2D(new BenchmarkMatterAgent(_context, Vector3d.Zero),
                new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless })
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
            _planarBody.Initialize(new(-Fixed64.Two, Fixed64.Half));
            _planarBody.UseManualGrounding();
        }
        else
        {
            Vector3d position = Mode == "Mixed" ? new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half) : new(-Fixed64.Two, Fixed64.Half, Fixed64.Zero);
            _spatialBody = new SolidBody(new BenchmarkMatterAgent(_context, position),
                new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless })
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation | (Mode == "Mixed" ? BodyFreezeAxes3D.PositionY : BodyFreezeAxes3D.PositionZ), SleepEnabled = false };
            _spatialBody.Initialize(position, FixedQuaternion.Identity);
            _spatialBody.UseManualGrounding();
        }
        _direction = Fixed64.One;
        for (int frame = 0; frame < 32; frame++) { _context.Simulate(); _context.LateSimulate(); }
        for (int batch = 0; batch < 4; batch++) FullStep();
    }

    [Benchmark(OperationsPerInvoke = 64)]
    public void FullStep()
    {
        for (int frame = 0; frame < 64; frame++)
        {
            Fixed64 x = _planarBody?.Position.X ?? _spatialBody.Position3d.X;
            if (x >= Fixed64.Two) _direction = -Fixed64.One;
            else if (x <= -Fixed64.Two) _direction = Fixed64.One;
            if (_planarBody != null)
                _planarBody.AddLinearImpulse(Vector2d.Right * (_direction - _planarBody.LinearVelocity.X));
            else
            {
                _spatialBody.AddLinearImpulse(Vector3d.Right * (_direction - _spatialBody.LinearVelocity.X));
                if (Mode == "Mixed") _spatialBody.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
            }
            _context.Simulate();
            _context.LateSimulate();
            Fixed64 speed = _planarBody?.LinearVelocity.X ?? _spatialBody.LinearVelocity.X;
            Fixed64 height = _planarBody?.Position.Y ?? (Mode == "Mixed" ? _spatialBody.Position3d.Z : _spatialBody.Position3d.Y);
            if (speed != _direction || height <= Fixed64.Zero)
                throw new InvalidOperationException("A frictionless population floor must preserve tangential motion and support the moving body.");
        }
    }

    [Benchmark]
    public void RefreshStaticShapes()
    {
        foreach (LSCollider2D floor in _planarFloors) floor.Simulate();
        foreach (LSCollider floor in _spatialFloors) floor.Simulate();
    }

    [GlobalCleanup]
    public void Cleanup() => _context?.Dispose();
}
