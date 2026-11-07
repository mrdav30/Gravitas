using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;

namespace Gravitas.Benchmarks;

/// <summary>Measures native wall response with retained and regenerated contact geometry.</summary>
[MemoryDiagnoser]
public class NativeMeshSimulationBenchmarks
{
    private GravitasWorldContext _context;
    private SolidBody _body;

    [Params("ThreeD", "Mixed")]
    public string Mode { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateMixedContext(16, 8);
        _context.Settings.RuntimeMode = Mode == "Mixed" ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD;
        _context.SetFrameRate(16);
        _context.Environment.Gravity = Fixed64.Zero;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        if (Mode == "Mixed")
        {
            for (int index = 0; index < 2; index++)
            {
                var target = new LSPolygonCollider2D(new(-4, -1), new(4, -1), new(4, 1), new(-4, 1))
                { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
                target.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new(index == 0 ? -4 : 4, 0, -1)));
            }
        }
        else
        {
            for (int index = 0; index < 2; index++)
            {
                var target = new LSMeshCollider(new[] { new Vector3d(-4, -2, 0), new(4, -2, 0),
                    new(4, 2, 0), new(-4, 2, 0) }, new[] { 0, 1, 2, 0, 2, 3 },
                    MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
                { Material = PhysicsMaterial.Frictionless };
                target.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new(index == 0 ? -4 : 4, 0, 0)));
            }
        }
        LSMeshCollider hull = BenchmarkPhysicsScene.CreateDynamicConvexCube(_context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half));
        hull.Material = PhysicsMaterial.Frictionless;
        _body = hull.Body!;
        _body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        _body.SleepEnabled = false;
        _body.UseManualGrounding();
        for (int frame = 0; frame < 32; frame++) FullStep();
        if (_body.Position3d.Z <= Fixed64.Zero || _body.LinearVelocity != Vector3d.Zero)
            throw new InvalidOperationException("The native hull must retain a settled physical wall response.");
    }

    [Benchmark]
    public long FullStep()
    {
        _body.AddForce(Vector3d.Backward);
        _context.Simulate();
        _context.LateSimulate();
        return _body.Position3d.Z.m_rawValue;
    }

    [Benchmark]
    public long HostMoveAndFullStep()
    {
        Vector3d position = _body.Position3d;
        position.X = position.X == Fixed64.Zero ? Fixed64.Quarter : Fixed64.Zero;
        _body.ResetPosition(position, FixedQuaternion.Identity);
        return FullStep();
    }

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
