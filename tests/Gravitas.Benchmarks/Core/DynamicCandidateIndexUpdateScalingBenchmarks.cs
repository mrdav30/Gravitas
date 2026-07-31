using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Benchmarks;

/// <summary>
/// Measures translation-only updates of the unique widest candidate.
/// </summary>
[MemoryDiagnoser]
public class DynamicCandidateIndexUpdateScalingBenchmarks
{
    private DynamicCcdCandidateIndex _index3D;
    private DynamicCcdCandidateIndex2D _index2D;
    private SwiftList<int> _results3D;
    private SwiftList<int> _results2D;
    private bool _alternate3D;
    private bool _alternate2D;

    [Params(64, 1024, 16384)]
    public int CandidateCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _index3D = new DynamicCcdCandidateIndex(CandidateCount, supportsUpdates: true);
        _index2D = new DynamicCcdCandidateIndex2D(CandidateCount, supportsUpdates: true);
        _results3D = new SwiftList<int>(1);
        _results2D = new SwiftList<int>(1);
        _index3D.Add(0, CreateBounds3D(Fixed64.Zero, Fixed64.Two));
        _index2D.Add(0, CreateBounds2D(Fixed64.Zero, Fixed64.Two));

        for (int i = 1; i < CandidateCount; i++)
        {
            Fixed64 center = (Fixed64)(i + 8);
            _index3D.Add(i, CreateBounds3D(center, Fixed64.Half));
            _index2D.Add(i, CreateBounds2D(center, Fixed64.Half));
        }
    }

    [Benchmark]
    public int TranslateUniqueWidest3D()
    {
        _alternate3D = !_alternate3D;
        _index3D.AddOrUpdate(0, CreateBounds3D(_alternate3D ? Fixed64.One : Fixed64.Zero, Fixed64.Two));
        return _index3D.Count;
    }

    [Benchmark]
    public int TranslateUniqueWidest2D()
    {
        _alternate2D = !_alternate2D;
        _index2D.AddOrUpdate(0, CreateBounds2D(_alternate2D ? Fixed64.One : Fixed64.Zero, Fixed64.Two));
        return _index2D.Count;
    }

    [Benchmark]
    public int TranslateUniqueWidestThenQuery3D()
    {
        _alternate3D = !_alternate3D;
        FixedBoundVolume bounds = CreateBounds3D(_alternate3D ? Fixed64.One : Fixed64.Zero, Fixed64.Two);
        _index3D.AddOrUpdate(0, bounds);
        _index3D.Query(bounds, _results3D);
        return _results3D.Count;
    }

    [Benchmark]
    public int TranslateUniqueWidestThenQuery2D()
    {
        _alternate2D = !_alternate2D;
        DynamicCcdPlanarBounds bounds = CreateBounds2D(_alternate2D ? Fixed64.One : Fixed64.Zero, Fixed64.Two);
        _index2D.AddOrUpdate(0, bounds);
        _index2D.Query(bounds, _results2D);
        return _results2D.Count;
    }

    private static FixedBoundVolume CreateBounds3D(Fixed64 centerX, Fixed64 extentX) =>
        new(
            new Vector3d(centerX - extentX, -Fixed64.Half, -Fixed64.Half),
            new Vector3d(centerX + extentX, Fixed64.Half, Fixed64.Half));

    private static DynamicCcdPlanarBounds CreateBounds2D(Fixed64 centerX, Fixed64 extentX) =>
        new(centerX - extentX, -Fixed64.Half, centerX + extentX, Fixed64.Half);
}
