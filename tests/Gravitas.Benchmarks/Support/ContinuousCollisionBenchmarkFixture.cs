using FixedMathSharp;
using Gravitas.Benchmarking;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

internal sealed class ContinuousCollisionBenchmarkFixture : IDisposable
{
    public ContinuousCollisionBenchmarkFixture(int bodyCount)
    {
        BodyCount = bodyCount;
        int mixedPerDimension = bodyCount / 2;

        Sparse3DContext = ContinuousCollisionBenchmarkSupport.CreateContext3D(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.SparseExtentZ(bodyCount));
        Dense3DContext = ContinuousCollisionBenchmarkSupport.CreateContext3D(
            ContinuousCollisionBenchmarkLayout.DenseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.DenseExtentZ(bodyCount));
        Sparse2DContext = ContinuousCollisionBenchmarkSupport.CreateContext2D(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.SparseExtentZ(bodyCount));
        Dense2DContext = ContinuousCollisionBenchmarkSupport.CreateContext2D(
            ContinuousCollisionBenchmarkLayout.DenseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.DenseExtentZ(bodyCount));
        ShapeExact3DContext = ContinuousCollisionBenchmarkSupport.CreateContext3D(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.SparseExtentZ(bodyCount) + 8);
        ShapeExact2DContext = ContinuousCollisionBenchmarkSupport.CreateContext2D(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.SparseExtentZ(bodyCount) + 8);
        DynamicShapeExact3DContext = ContinuousCollisionBenchmarkSupport.CreateContext3D(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.SparseExtentZ(bodyCount) + 8);
        DynamicShapeExact2DContext = ContinuousCollisionBenchmarkSupport.CreateContext2D(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(bodyCount),
            ContinuousCollisionBenchmarkLayout.SparseExtentZ(bodyCount) + 8);
        SparseMixedContext = ContinuousCollisionBenchmarkSupport.CreateMixedContext(
            ContinuousCollisionBenchmarkLayout.SparseExtentX(mixedPerDimension),
            ContinuousCollisionBenchmarkLayout.MixedSparseOffsetZ + ContinuousCollisionBenchmarkLayout.SparseExtentZ(mixedPerDimension));
        DenseMixedContext = ContinuousCollisionBenchmarkSupport.CreateMixedContext(
            ContinuousCollisionBenchmarkLayout.DenseExtentX(mixedPerDimension * 2),
            ContinuousCollisionBenchmarkLayout.DenseExtentZ(mixedPerDimension * 2));

        Sparse3DBodies = new SwiftList<StiffBody>(bodyCount);
        Dense3DBodies = new SwiftList<StiffBody>(bodyCount);
        Sparse2DBodies = new SwiftList<StiffBody2D>(bodyCount);
        Dense2DBodies = new SwiftList<StiffBody2D>(bodyCount);
        ShapeExact3DBodies = new SwiftList<StiffBody>(bodyCount);
        ShapeExact2DBodies = new SwiftList<StiffBody2D>(bodyCount);
        DynamicShapeExact3DBodies = new SwiftList<StiffBody>(bodyCount * 2);
        DynamicShapeExact2DBodies = new SwiftList<StiffBody2D>(bodyCount * 2);
        SparseMixed3DBodies = new SwiftList<StiffBody>(mixedPerDimension);
        SparseMixed2DBodies = new SwiftList<StiffBody2D>(mixedPerDimension);
        DenseMixed3DBodies = new SwiftList<StiffBody>(mixedPerDimension);
        DenseMixed2DBodies = new SwiftList<StiffBody2D>(mixedPerDimension);
        Query3DHits = new SwiftList<Physics3DHit>(bodyCount);
        Query2DHits = new SwiftList<Physics2DHit>(bodyCount);
        MixedQueryHits = new SwiftList<PhysicsMixedHit>(mixedPerDimension);

        Sparse3DPositions = new Vector3d[bodyCount];
        Dense3DPositions = new Vector3d[bodyCount];
        Sparse2DPositions = new Vector2d[bodyCount];
        Dense2DPositions = new Vector2d[bodyCount];
        ShapeExact3DPositions = new Vector3d[bodyCount];
        ShapeExact2DPositions = new Vector2d[bodyCount];
        DynamicShapeExact3DPositions = new Vector3d[bodyCount * 2];
        DynamicShapeExact2DPositions = new Vector2d[bodyCount * 2];
        SparseMixed3DPositions = new Vector3d[mixedPerDimension];
        SparseMixed2DPositions = new Vector2d[mixedPerDimension];
        DenseMixed3DPositions = new Vector3d[mixedPerDimension];
        DenseMixed2DPositions = new Vector2d[mixedPerDimension];

        CreatePureScenes(bodyCount);
        CreateMixedScenes(mixedPerDimension);
    }

    public int BodyCount { get; }

    public GravitasWorldContext Sparse3DContext { get; }

    public GravitasWorldContext Dense3DContext { get; }

    public GravitasWorldContext Sparse2DContext { get; }

    public GravitasWorldContext Dense2DContext { get; }

    public GravitasWorldContext ShapeExact3DContext { get; }

    public GravitasWorldContext ShapeExact2DContext { get; }

    public GravitasWorldContext DynamicShapeExact3DContext { get; }

    public GravitasWorldContext DynamicShapeExact2DContext { get; }

    public GravitasWorldContext SparseMixedContext { get; }

    public GravitasWorldContext DenseMixedContext { get; }

    public SwiftList<StiffBody> Sparse3DBodies { get; }

    public SwiftList<StiffBody> Dense3DBodies { get; }

    public SwiftList<StiffBody2D> Sparse2DBodies { get; }

    public SwiftList<StiffBody2D> Dense2DBodies { get; }

    public SwiftList<StiffBody> ShapeExact3DBodies { get; }

    public SwiftList<StiffBody2D> ShapeExact2DBodies { get; }

    public SwiftList<StiffBody> DynamicShapeExact3DBodies { get; }

    public SwiftList<StiffBody2D> DynamicShapeExact2DBodies { get; }

    public SwiftList<StiffBody> SparseMixed3DBodies { get; }

    public SwiftList<StiffBody2D> SparseMixed2DBodies { get; }

    public SwiftList<StiffBody> DenseMixed3DBodies { get; }

    public SwiftList<StiffBody2D> DenseMixed2DBodies { get; }

    public SwiftList<Physics3DHit> Query3DHits { get; }

    public SwiftList<Physics2DHit> Query2DHits { get; }

    public SwiftList<PhysicsMixedHit> MixedQueryHits { get; }

    public Vector3d[] Sparse3DPositions { get; }

    public Vector3d[] Dense3DPositions { get; }

    public Vector2d[] Sparse2DPositions { get; }

    public Vector2d[] Dense2DPositions { get; }

    public Vector3d[] ShapeExact3DPositions { get; }

    public Vector2d[] ShapeExact2DPositions { get; }

    public Vector3d[] DynamicShapeExact3DPositions { get; }

    public Vector2d[] DynamicShapeExact2DPositions { get; }

    public Vector3d[] SparseMixed3DPositions { get; }

    public Vector2d[] SparseMixed2DPositions { get; }

    public Vector3d[] DenseMixed3DPositions { get; }

    public Vector2d[] DenseMixed2DPositions { get; }

    public void Dispose()
    {
        Sparse3DContext.Dispose();
        Dense3DContext.Dispose();
        Sparse2DContext.Dispose();
        Dense2DContext.Dispose();
        ShapeExact3DContext.Dispose();
        ShapeExact2DContext.Dispose();
        DynamicShapeExact3DContext.Dispose();
        DynamicShapeExact2DContext.Dispose();
        SparseMixedContext.Dispose();
        DenseMixedContext.Dispose();
    }

    private void CreatePureScenes(int bodyCount)
    {
        for (int i = 0; i < bodyCount; i++)
        {
            Vector3d sparse3D = ContinuousCollisionBenchmarkLayout.Sparse3DPosition(i);
            Vector3d dense3D = ContinuousCollisionBenchmarkLayout.Dense3DPosition(i);
            Vector2d sparse2D = sparse3D.ToVector2d();
            Vector2d dense2D = dense3D.ToVector2d();
            Sparse3DPositions[i] = sparse3D;
            Dense3DPositions[i] = dense3D;
            Sparse2DPositions[i] = sparse2D;
            Dense2DPositions[i] = dense2D;
            Sparse3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateSphere3D(Sparse3DContext, sparse3D));
            Dense3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateSphere3D(Dense3DContext, dense3D));
            Sparse2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateCircle2D(Sparse2DContext, sparse2D));
            Dense2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateCircle2D(Dense2DContext, dense2D));

            ShapeExact3DPositions[i] = sparse3D;
            ShapeExact2DPositions[i] = sparse2D;
            ShapeExact3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateThinCuboid3D(ShapeExact3DContext, sparse3D));
            ShapeExact2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateThinPolygon2D(ShapeExact2DContext, sparse2D));
            ContinuousCollisionBenchmarkSupport.CreateStaticCuboid3D(
                ShapeExact3DContext,
                sparse3D + new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero),
                Vector3d.One);
            ContinuousCollisionBenchmarkSupport.CreateStaticCircle2D(
                ShapeExact2DContext,
                sparse2D + new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2)));

            int dynamicShapeSourceIndex = i * 2;
            int dynamicShapeTargetIndex = dynamicShapeSourceIndex + 1;
            Vector3d dynamicSource3D = sparse3D;
            Vector3d dynamicTarget3D = sparse3D + new Vector3d((Fixed64)4, Fixed64.FromFraction(5, 2), Fixed64.Zero);
            DynamicShapeExact3DPositions[dynamicShapeSourceIndex] = dynamicSource3D;
            DynamicShapeExact3DPositions[dynamicShapeTargetIndex] = dynamicTarget3D;
            DynamicShapeExact3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateThinCuboid3D(DynamicShapeExact3DContext, dynamicSource3D));
            DynamicShapeExact3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateSphere3D(DynamicShapeExact3DContext, dynamicTarget3D));

            Vector2d dynamicSource = sparse2D;
            Vector2d dynamicTarget = sparse2D + new Vector2d((Fixed64)4, Fixed64.FromFraction(5, 2));
            DynamicShapeExact2DPositions[dynamicShapeSourceIndex] = dynamicSource;
            DynamicShapeExact2DPositions[dynamicShapeTargetIndex] = dynamicTarget;
            DynamicShapeExact2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateThinPolygon2D(DynamicShapeExact2DContext, dynamicSource));
            DynamicShapeExact2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateCircle2D(DynamicShapeExact2DContext, dynamicTarget));
        }
    }

    private void CreateMixedScenes(int mixedPerDimension)
    {
        for (int i = 0; i < mixedPerDimension; i++)
        {
            Vector3d sparse3D = ContinuousCollisionBenchmarkLayout.Sparse3DPosition(i);
            Vector2d sparse2D = ContinuousCollisionBenchmarkLayout.SparseMixed2DPosition(i);
            Vector3d dense3D = ContinuousCollisionBenchmarkLayout.DenseMixed3DPosition(i);
            Vector2d dense2D = ContinuousCollisionBenchmarkLayout.DenseMixed2DPosition(i);
            SparseMixed3DPositions[i] = sparse3D;
            SparseMixed2DPositions[i] = sparse2D;
            DenseMixed3DPositions[i] = dense3D;
            DenseMixed2DPositions[i] = dense2D;
            SparseMixed3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateSphere3D(SparseMixedContext, sparse3D));
            SparseMixed2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateCircle2D(SparseMixedContext, sparse2D));
            DenseMixed3DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateSphere3D(DenseMixedContext, dense3D));
            DenseMixed2DBodies.Add(ContinuousCollisionBenchmarkSupport.CreateCircle2D(DenseMixedContext, dense2D));
        }
    }
}
