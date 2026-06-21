using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
[InvocationCount(1)]
public class DynamicCcdScalingBenchmarks
{
    private const int PureBatchFrames = 8;
    private const int MixedBatchFrames = 8;

    private ContinuousCollisionBenchmarkFixture _fixture;

    [Params(64, 256)]
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

    [Benchmark]
    public Vector3d Sparse3DDynamicCcd()
    {
        Reset3DBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, pairedDirections: false);
        _fixture.Sparse3DContext.LateSimulate();
        return Sum3D(_fixture.Sparse3DBodies);
    }

    [Benchmark]
    public Vector3d Dense3DDynamicCcd()
    {
        Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
        _fixture.Dense3DContext.LateSimulate();
        return Sum3D(_fixture.Dense3DBodies);
    }

    [Benchmark]
    public Vector2d Sparse2DDynamicCcd()
    {
        Reset2DBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, pairedDirections: false);
        _fixture.Sparse2DContext.LateSimulate();
        return Sum2D(_fixture.Sparse2DBodies);
    }

    [Benchmark]
    public Vector2d Dense2DDynamicCcd()
    {
        Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
        _fixture.Dense2DContext.LateSimulate();
        return Sum2D(_fixture.Dense2DBodies);
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector3d SparsePure3DDynamicCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, pairedDirections: false);
            _fixture.Sparse3DContext.LateSimulate();
            total += Sum3D(_fixture.Sparse3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector3d DensePure3DDynamicCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
            _fixture.Dense3DContext.LateSimulate();
            total += Sum3D(_fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector2d SparsePure2DDynamicCcdBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, pairedDirections: false);
            _fixture.Sparse2DContext.LateSimulate();
            total += Sum2D(_fixture.Sparse2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector2d DensePure2DDynamicCcdBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
            _fixture.Dense2DContext.LateSimulate();
            total += Sum2D(_fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector3d SparsePure3DAngularCcdNoAngularMotionBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DAngularBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, angularMotion: false);
            _fixture.Sparse3DContext.LateSimulate();
            total += Sum3D(_fixture.Sparse3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector3d SparsePure3DAngularCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DAngularBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, angularMotion: true);
            _fixture.Sparse3DContext.LateSimulate();
            total += Sum3D(_fixture.Sparse3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector3d DensePure3DAngularCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DAngularBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, angularMotion: true);
            _fixture.Dense3DContext.LateSimulate();
            total += Sum3D(_fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector2d SparsePure2DAngularCcdNoAngularMotionBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DAngularBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, angularMotion: false);
            _fixture.Sparse2DContext.LateSimulate();
            total += Sum2D(_fixture.Sparse2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector2d SparsePure2DAngularCcdBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DAngularBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, angularMotion: true);
            _fixture.Sparse2DContext.LateSimulate();
            total += Sum2D(_fixture.Sparse2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector2d DensePure2DAngularCcdBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DAngularBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, angularMotion: true);
            _fixture.Dense2DContext.LateSimulate();
            total += Sum2D(_fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector3d SparsePure3DShapeExactCcdFalsePositiveBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.ShapeExact3DBodies, _fixture.ShapeExact3DPositions, pairedDirections: false);
            _fixture.ShapeExact3DContext.LateSimulate();
            total += Sum3D(_fixture.ShapeExact3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public Vector2d SparsePure2DShapeExactCcdFalsePositiveBatch8()
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.ShapeExact2DBodies, _fixture.ShapeExact2DPositions, pairedDirections: false);
            _fixture.ShapeExact2DContext.LateSimulate();
            total += Sum2D(_fixture.ShapeExact2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int SparsePure3DStaticQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodyPositions(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions);
            total += SweepPure3DStaticQueries(_fixture.Sparse3DContext, _fixture.Sparse3DBodies, _fixture.Sparse3DPositions, pairedDirections: false, _fixture.Query3DHits);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int DensePure3DStaticQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodyPositions(_fixture.Dense3DBodies, _fixture.Dense3DPositions);
            total += SweepPure3DStaticQueries(_fixture.Dense3DContext, _fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true, _fixture.Query3DHits);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int SparsePure2DStaticQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodyPositions(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions);
            total += SweepPure2DStaticQueries(_fixture.Sparse2DContext, _fixture.Sparse2DBodies, _fixture.Sparse2DPositions, pairedDirections: false, _fixture.Query2DHits);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int DensePure2DStaticQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodyPositions(_fixture.Dense2DBodies, _fixture.Dense2DPositions);
            total += SweepPure2DStaticQueries(_fixture.Dense2DContext, _fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true, _fixture.Query2DHits);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int SparsePure3DDynamicCandidateQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, pairedDirections: false);
            _fixture.Sparse3DContext.AdvanceLateSimulateToken();
            total += QueryPure3DDynamicCandidates(_fixture.Sparse3DContext, _fixture.Sparse3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int DensePure3DDynamicCandidateQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
            _fixture.Dense3DContext.AdvanceLateSimulateToken();
            total += QueryPure3DDynamicCandidates(_fixture.Dense3DContext, _fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int SparsePure2DDynamicCandidateQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, pairedDirections: false);
            _fixture.Sparse2DContext.AdvanceLateSimulateToken();
            total += QueryPure2DDynamicCandidates(_fixture.Sparse2DContext, _fixture.Sparse2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int DensePure2DDynamicCandidateQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
            _fixture.Dense2DContext.AdvanceLateSimulateToken();
            total += QueryPure2DDynamicCandidates(_fixture.Dense2DContext, _fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int SparsePure3DDynamicRelativeSweepBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.Sparse3DBodies, _fixture.Sparse3DPositions, pairedDirections: false);
            _fixture.Sparse3DContext.AdvanceLateSimulateToken();
            total += SweepPure3DDynamicRelativeTargets(_fixture.Sparse3DContext, _fixture.Sparse3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int DensePure3DDynamicRelativeSweepBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset3DBodies(_fixture.Dense3DBodies, _fixture.Dense3DPositions, pairedDirections: true);
            _fixture.Dense3DContext.AdvanceLateSimulateToken();
            total += SweepPure3DDynamicRelativeTargets(_fixture.Dense3DContext, _fixture.Dense3DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int SparsePure2DDynamicRelativeSweepBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.Sparse2DBodies, _fixture.Sparse2DPositions, pairedDirections: false);
            _fixture.Sparse2DContext.AdvanceLateSimulateToken();
            total += SweepPure2DDynamicRelativeTargets(_fixture.Sparse2DContext, _fixture.Sparse2DBodies);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = PureBatchFrames)]
    public int DensePure2DDynamicRelativeSweepBatch8()
    {
        int total = 0;
        for (int i = 0; i < PureBatchFrames; i++)
        {
            Reset2DBodies(_fixture.Dense2DBodies, _fixture.Dense2DPositions, pairedDirections: true);
            _fixture.Dense2DContext.AdvanceLateSimulateToken();
            total += SweepPure2DDynamicRelativeTargets(_fixture.Dense2DContext, _fixture.Dense2DBodies);
        }

        return total;
    }

    [Benchmark]
    public Vector3d SparseMixedDynamicCcd()
    {
        Reset3DBodies(_fixture.SparseMixed3DBodies, _fixture.SparseMixed3DPositions, pairedDirections: false);
        Reset2DBodies(_fixture.SparseMixed2DBodies, _fixture.SparseMixed2DPositions, pairedDirections: false);
        _fixture.SparseMixedContext.LateSimulate();
        return Sum3D(_fixture.SparseMixed3DBodies) + Sum2D(_fixture.SparseMixed2DBodies).ToVector3d(Fixed64.Zero);
    }

    [Benchmark]
    public Vector3d DenseMixedDynamicCcd()
    {
        Reset3DBodies(_fixture.DenseMixed3DBodies, _fixture.DenseMixed3DPositions, pairedDirections: false);
        Reset2DBodies(_fixture.DenseMixed2DBodies, _fixture.DenseMixed2DPositions, pairedDirections: true);
        _fixture.DenseMixedContext.LateSimulate();
        return Sum3D(_fixture.DenseMixed3DBodies) + Sum2D(_fixture.DenseMixed2DBodies).ToVector3d(Fixed64.Zero);
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public Vector3d SparseMixedDynamicCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < MixedBatchFrames; i++)
        {
            Reset3DBodies(_fixture.SparseMixed3DBodies, _fixture.SparseMixed3DPositions, pairedDirections: false);
            Reset2DBodies(_fixture.SparseMixed2DBodies, _fixture.SparseMixed2DPositions, pairedDirections: false);
            _fixture.SparseMixedContext.LateSimulate();
            total += Sum3D(_fixture.SparseMixed3DBodies) + Sum2D(_fixture.SparseMixed2DBodies).ToVector3d(Fixed64.Zero);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public Vector3d DenseMixedDynamicCcdBatch8()
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < MixedBatchFrames; i++)
        {
            Reset3DBodies(_fixture.DenseMixed3DBodies, _fixture.DenseMixed3DPositions, pairedDirections: false);
            Reset2DBodies(_fixture.DenseMixed2DBodies, _fixture.DenseMixed2DPositions, pairedDirections: true);
            _fixture.DenseMixedContext.LateSimulate();
            total += Sum3D(_fixture.DenseMixed3DBodies) + Sum2D(_fixture.DenseMixed2DBodies).ToVector3d(Fixed64.Zero);
        }

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int SparseMixedStatic2DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepMixedStatic2DQueries(_fixture.SparseMixedContext, _fixture.SparseMixed3DBodies, _fixture.SparseMixed3DPositions, _fixture.MixedQueryHits);

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int DenseMixedStatic2DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepMixedStatic2DQueries(_fixture.DenseMixedContext, _fixture.DenseMixed3DBodies, _fixture.DenseMixed3DPositions, _fixture.MixedQueryHits);

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int SparseMixedStatic3DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepMixedStatic3DQueries(_fixture.SparseMixedContext, _fixture.SparseMixed2DBodies, _fixture.SparseMixed2DPositions, _fixture.MixedQueryHits);

        return total;
    }

    [Benchmark(OperationsPerInvoke = MixedBatchFrames)]
    public int DenseMixedStatic3DQueryBatch8()
    {
        int total = 0;
        for (int i = 0; i < MixedBatchFrames; i++)
            total += SweepMixedStatic3DQueries(_fixture.DenseMixedContext, _fixture.DenseMixed2DBodies, _fixture.DenseMixed2DPositions, _fixture.MixedQueryHits);

        return total;
    }

}
