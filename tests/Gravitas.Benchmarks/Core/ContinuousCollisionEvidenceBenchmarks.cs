using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class ContinuousCollisionEvidenceBenchmarks
{
    private const int EvidenceFrames = 64;

    private ContinuousCollisionBenchmarkFixture _fixture;

    [Params(256, 1024)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _fixture = new ContinuousCollisionBenchmarkFixture(BodyCount);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _fixture.Dispose();
        _fixture = null;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector3d Pure3DFullRuntimeNoHitDynamicCcdEvidence()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, pairedDirections: false);
            _fixture.Sparse3DContext.LateSimulate();
            total += Sum3D(_fixture.Sparse3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector3d Pure3DFullRuntimeDenseHitDynamicCcdEvidence()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
            _fixture.Dense3DContext.LateSimulate();
            total += Sum3D(_fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector2d Pure2DFullRuntimeNoHitDynamicCcdEvidence()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, pairedDirections: false);
            _fixture.Sparse2DContext.LateSimulate();
            total += Sum2D(_fixture.Sparse2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector2d Pure2DFullRuntimeDenseHitDynamicCcdEvidence()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
            _fixture.Dense2DContext.LateSimulate();
            total += Sum2D(_fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector3d MixedFullRuntimeNoHitDynamicCcdEvidence()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.SparseMixed3DBodies, _fixture.SparseMixed3DPositions, pairedDirections: false);
            Reset2DBodies(_fixture.SparseMixed2DBodies, _fixture.SparseMixed2DPositions, pairedDirections: false);
            _fixture.SparseMixedContext.LateSimulate();
            total += Sum3D(_fixture.SparseMixed3DBodies) + Sum2D(_fixture.SparseMixed2DBodies).ToVector3d(Fixed64.Zero);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector3d MixedFullRuntimeDenseHitDynamicCcdEvidence()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.DenseMixed3DBodies, _fixture.DenseMixed3DPositions, pairedDirections: false);
            Reset2DBodies(_fixture.DenseMixed2DBodies, _fixture.DenseMixed2DPositions, pairedDirections: true);
            _fixture.DenseMixedContext.LateSimulate();
            total += Sum3D(_fixture.DenseMixed3DBodies) + Sum2D(_fixture.DenseMixed2DBodies).ToVector3d(Fixed64.Zero);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public int Pure3DStaticQueryAttributionEvidence()
    {
        int total = 0;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodyPositions(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions);
            total += SweepPure3DStaticQueries(
                _fixture.Sparse3DContext,
                _fixture.Sparse3DBodies,
                _fixture.Sparse3DPositions,
                pairedDirections: false,
                _fixture.Query3DHits);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public int Pure2DStaticQueryAttributionEvidence()
    {
        int total = 0;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DBodyPositions(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions);
            total += SweepPure2DStaticQueries(
                _fixture.Sparse2DContext,
                _fixture.Sparse2DBodies,
                _fixture.Sparse2DPositions,
                pairedDirections: false,
                _fixture.Query2DHits);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public int Pure3DDynamicCandidateIndexAttributionEvidence()
    {
        int total = 0;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
            _fixture.Dense3DContext.AdvanceLateSimulateToken();
            total += QueryPure3DDynamicCandidates(_fixture.Dense3DContext, _fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public int Pure2DDynamicCandidateIndexAttributionEvidence()
    {
        int total = 0;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
            _fixture.Dense2DContext.AdvanceLateSimulateToken();
            total += QueryPure2DDynamicCandidates(_fixture.Dense2DContext, _fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public int Pure3DDynamicRelativeSweepAttributionEvidence()
    {
        int total = 0;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
            _fixture.Dense3DContext.AdvanceLateSimulateToken();
            total += SweepPure3DDynamicRelativeTargets(_fixture.Dense3DContext, _fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public int Pure2DDynamicRelativeSweepAttributionEvidence()
    {
        int total = 0;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
            _fixture.Dense2DContext.AdvanceLateSimulateToken();
            total += SweepPure2DDynamicRelativeTargets(_fixture.Dense2DContext, _fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector3d Pure3DFullRuntimeShapeExactFalsePositiveEvidence()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DBodies(_fixture.ShapeExact3DBodies, _fixture.ShapeExact3DPositions, pairedDirections: false);
            _fixture.ShapeExact3DContext.LateSimulate();
            total += Sum3D(_fixture.ShapeExact3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector2d Pure2DFullRuntimeShapeExactFalsePositiveEvidence()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DBodies(_fixture.ShapeExact2DBodies, _fixture.ShapeExact2DPositions, pairedDirections: false);
            _fixture.ShapeExact2DContext.LateSimulate();
            total += Sum2D(_fixture.ShapeExact2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector3d Pure3DFullRuntimeDynamicShapeExactFalsePositiveEvidence()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset3DDynamicShapeExactBodies(_fixture.DynamicShapeExact3DBodies, _fixture.DynamicShapeExact3DPositions);
            _fixture.DynamicShapeExact3DContext.LateSimulate();
            total += Sum3D(_fixture.DynamicShapeExact3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = EvidenceFrames)]
    public Vector2d Pure2DFullRuntimeDynamicShapeExactFalsePositiveEvidence()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < EvidenceFrames; i++)
        {
            Reset2DDynamicShapeExactBodies(_fixture.DynamicShapeExact2DBodies, _fixture.DynamicShapeExact2DPositions);
            _fixture.DynamicShapeExact2DContext.LateSimulate();
            total += Sum2D(_fixture.DynamicShapeExact2DBodies);
        }

        return total;
    }
}
