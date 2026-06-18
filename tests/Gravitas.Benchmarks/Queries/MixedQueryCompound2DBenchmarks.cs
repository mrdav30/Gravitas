using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedQueryCompound2DBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _context;
    private SwiftList<PhysicsMixedHit> _hits;
    private Vector3d _end;

    [Params(64, 1024)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extentX = 16 + (ColliderCount * 3);
        _context = CreateMixedContext(extentX, 16);
        _hits = new SwiftList<PhysicsMixedHit>(ColliderCount);

        for (int i = 0; i < ColliderCount; i++)
            _ = CreateCompound2D(_context, new Vector3d((Fixed64)(i * 3), Fixed64.Zero, Fixed64.Zero));

        _context.Simulate();
        _end = new Vector3d((Fixed64)(ColliderCount * 3), Fixed64.Zero, Fixed64.Zero);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context?.Dispose();
        _context = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepSphereAgainst2DAll_Compound2DTargets()
    {
        return _context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            _end,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    private static GravitasWorldContext CreateMixedContext(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        if (!context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)extentX, (Fixed64)4, (Fixed64)extentZ)),
            out _))
        {
            throw new InvalidOperationException("Unable to create mixed compound query benchmark grid.");
        }

        return context;
    }

    private static StiffBody2D CreateCompound2D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Half, Fixed64.Zero)));
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(new Vector2d(position.X, position.Z));
        return body;
    }
}
