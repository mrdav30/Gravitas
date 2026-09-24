using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class SupportQuery2DBenchmarks
{
    private GravitasWorldContext _context;
    private Physics2DSupportQuery[] _probes;

    [Params(1, 64)]
    public int AgentCount { get; set; }

    [Params(false, true)]
    public bool CompoundFloor { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        if (!_context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-2, 0, -2), new Vector3d(AgentCount * 4, 0, 4)), out _))
            throw new InvalidOperationException("Support benchmark grid was not admitted.");
        _probes = new Physics2DSupportQuery[AgentCount];
        for (int i = 0; i < AgentCount; i++)
        {
            Vector2d origin = Vector2d.Right * (Fixed64)(i * 4);
            LSCollider2D floor = CompoundFloor
                ? new LSCompoundCollider2D(
                    CompoundColliderPart2D.AABBox(new Vector2d(1, 3), new Vector2d(-Fixed64.Half, Fixed64.Zero)),
                    CompoundColliderPart2D.AABBox(new Vector2d(3, 1), new Vector2d(Fixed64.Zero, -Fixed64.Half)))
                : new LSAABBoxCollider2D(new Vector2d(3, 1)) { LocalOffset = new Vector2d(Fixed64.Zero, -Fixed64.Half) };
            floor.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new Vector3d(origin.X, Fixed64.Zero, origin.Y)));
            Vector2d center = origin + new Vector2d(Fixed64.Half, Fixed64.Half);
            var body = new SolidBody2D(new BenchmarkMatterAgent(_context,
                new Vector3d(center.X, Fixed64.Zero, center.Y)), new LSCircleCollider2D(Fixed64.Half));
            body.Initialize(center, motionType: BodyMotionType.Kinematic);
            body.UseManualGrounding();
            _probes[i] = new Physics2DSupportQuery(body.Collider, center, Fixed64.Half,
                Vector2d.Forward, Fixed64.Half, Fixed64.Half, PhysicsLayerMask.All);
        }
        for (int frame = 0; frame < 8; frame++)
        {
            if (CompletePhysicsAndSupportFrame() != AgentCount)
                throw new InvalidOperationException("Support benchmark did not find every floor.");
        }
    }

    [Benchmark]
    public int SupportQueries()
    {
        int found = 0;
        for (int i = 0; i < _probes.Length; i++)
        {
            if (_context.Query2D.QuerySupport(_probes[i], out var support) == Physics2DSupportQueryStatus.Found
                && support.Hit.Normal == Vector2d.Forward)
                found++;
        }
        return found;
    }

    [Benchmark]
    public int CompletePhysicsAndSupportFrame()
    {
        _context.Simulate();
        _context.LateSimulate();
        return SupportQueries();
    }

    [GlobalCleanup]
    public void Cleanup() => _context?.Dispose();
}
