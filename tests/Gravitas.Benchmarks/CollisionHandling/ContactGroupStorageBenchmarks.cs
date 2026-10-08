using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

/// <summary>
/// Separates first-ever pair/group allocations from regeneration after the
/// same pair has retained its group high-water capacity. Geometry and response
/// cost belong to their existing benchmarks; these are prepared storage inputs.
/// </summary>
[MemoryDiagnoser]
public class ContactGroupStorageBenchmarks
{
    private GravitasWorldContext _context;
    private LSCuboidCollider _first, _second;
    private CollisionPair _retainedPair;
    private ContactGroupKey[] _groups;
    private ManifoldContact[] _contacts;

    [Params(1, 9)]
    public int GroupCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(4, clearAllPools: true);
        _first = BenchmarkPhysicsScene.CreateDynamicCuboid(_context, Vector3d.Zero);
        _second = BenchmarkPhysicsScene.CreateDynamicCuboid(_context, Vector3d.Up);
        _groups = new ContactGroupKey[GroupCount];
        for (int group = 0; group < GroupCount; group++)
            _groups[group] = new ContactGroupKey(0, 0, surfaceA: group);
        _contacts = new ManifoldContact[ContactManifold.MaxContactsPerGroup];
        for (int point = 0; point < _contacts.Length; point++)
        {
            Vector3d anchor = new(point < 2 ? -Fixed64.Quarter : Fixed64.Quarter,
                Fixed64.Half, (point & 1) == 0 ? -Fixed64.Quarter : Fixed64.Quarter);
            _contacts[point] = new ManifoldContact((ulong)(point + 1), anchor, anchor,
                Fixed64.Zero, Vector3d.Up);
        }

        _retainedPair = (CollisionPair)ColdPairWithGroups();
        ReuseRetainedGroups();
        if (_retainedPair.Manifold.GroupCount != GroupCount
            || _retainedPair.Manifold.Count != GroupCount * _contacts.Length)
            throw new InvalidOperationException("Prepared storage lost an independent contact group.");
        for (int group = 0; group < GroupCount; group++)
            foreach (ManifoldContact contact in _contacts)
                if (!_retainedPair.TryGetWarmStartImpulse(_groups[group], contact.ContactId, out var impulse)
                    || impulse.NormalImpulse != Fixed64.One)
                    throw new InvalidOperationException("Prepared storage lost a grouped warm-start impulse.");
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _retainedPair = null;
        _groups = null;
        _contacts = null;
        _first = _second = null;
        _context.Dispose();
        _context = null;
    }

    [Benchmark]
    public object ColdPairWithGroups()
    {
        var pair = new CollisionPair(_first, _second);
        PopulateGroups(pair);
        return pair;
    }

    [Benchmark]
    public int ReuseRetainedGroups()
    {
        _retainedPair.Reset();
        PopulateGroups(_retainedPair);
        return _retainedPair.Manifold.Count;
    }

    private void PopulateGroups(CollisionPair pair)
    {
        pair.Manifold.BeginUpdate(0);
        for (int group = 0; group < _groups.Length; group++)
            foreach (ManifoldContact contact in _contacts)
            {
                pair.Manifold.AddContact(_groups[group], contact);
                pair.StoreWarmStartImpulse(_groups[group], contact.ContactId,
                    Vector3d.Up, Fixed64.One, Fixed64.Half, Fixed64.Quarter);
            }
    }
}
