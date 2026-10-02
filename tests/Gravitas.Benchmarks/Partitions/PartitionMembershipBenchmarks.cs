using BenchmarkDotNet.Attributes;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Compare existing owners for small partition-local sets of global collider IDs.</summary>
[MemoryDiagnoser]
public class PartitionMembershipBenchmarks
{
    private readonly SwiftSparseSet _sparse = new();
    private readonly SwiftHashSet<int> _hash = new();
    private readonly SwiftPackedSet<int> _packed = new();
    private readonly SwiftList<int> _ids = new();
    private const int FirstId = 65536;

    [Params(1, 8, 64)]
    public int Count { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        for (int i = 0; i < Count; i++)
        {
            _sparse.Add(FirstId + i);
            _hash.Add(FirstId + i);
            _packed.Add(FirstId + i);
        }
        _ids.EnsureCapacity(Count + 1);
        if (SparseMembership() != FirstId + Count - 1
            || HashMembership() != FirstId + Count - 1
            || PackedMembership() != FirstId + Count - 1)
            throw new InvalidOperationException("Incorrect sorted membership.");
    }

    [Benchmark(Baseline = true)]
    public int SparseMembership()
    {
        _sparse.Add(FirstId + Count);
        _sparse.Remove(FirstId + Count);
        _sparse.CopySortedKeysTo(_ids);
        return _ids[Count - 1];
    }

    [Benchmark]
    public int HashMembership()
    {
        _hash.Add(FirstId + Count);
        _hash.Remove(FirstId + Count);
        _ids.FastClear();
        foreach (int id in _hash)
            _ids.Add(id);
        _ids.SortInPlace();
        return _ids[Count - 1];
    }

    [Benchmark]
    public int PackedMembership()
    {
        _packed.Add(FirstId + Count);
        _packed.Remove(FirstId + Count);
        _ids.FastClear();
        for (int i = 0; i < _packed.Count; i++)
            _ids.Add(_packed.Dense[i]);
        _ids.SortInPlace();
        return _ids[Count - 1];
    }
}
