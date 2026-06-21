#nullable enable

using FixedMathSharp;
using Gravitas.Support;
using System;
using System.Collections.Generic;

namespace Gravitas.Benchmarking;

internal enum ContinuousCollisionBenchmarkSet : byte
{
    SparsePure3D,
    DensePure3D,
    SparsePure2D,
    DensePure2D,
    ShapeExactPure3D,
    ShapeExactPure2D,
    SparseMixed3D,
    SparseMixed2D,
    DenseMixed3D,
    DenseMixed2D
}

internal readonly struct ContinuousCollisionBenchmarkDescriptor : IEquatable<ContinuousCollisionBenchmarkDescriptor>
{
    public ContinuousCollisionBenchmarkDescriptor(
        ContinuousCollisionBenchmarkSet set,
        int index,
        int colliderOrdinal,
        PhysicsLayer layer,
        Vector3d position3D,
        Vector2d position2D)
    {
        Set = set;
        Index = index;
        ColliderOrdinal = colliderOrdinal;
        Layer = layer;
        Position3D = position3D;
        Position2D = position2D;
    }

    public ContinuousCollisionBenchmarkSet Set { get; }

    public int Index { get; }

    public int ColliderOrdinal { get; }

    public PhysicsLayer Layer { get; }

    public Vector3d Position3D { get; }

    public Vector2d Position2D { get; }

    public bool Equals(ContinuousCollisionBenchmarkDescriptor other) =>
        Set == other.Set
        && Index == other.Index
        && ColliderOrdinal == other.ColliderOrdinal
        && Layer == other.Layer
        && Position3D == other.Position3D
        && Position2D == other.Position2D;

    public override bool Equals(object? obj) =>
        obj is ContinuousCollisionBenchmarkDescriptor other && Equals(other);

    public override int GetHashCode() =>
        HashCode.Combine(Set, Index, ColliderOrdinal, Layer, Position3D, Position2D);
}

internal static class ContinuousCollisionBenchmarkLayout
{
    public const int SparseColumns = 64;
    public const int DensePairColumns = 32;
    public const int SparseSpacing = 8;
    public const int DensePairSpacing = 8;
    public const int MixedSparseOffsetZ = 4;

    public static IEnumerable<ContinuousCollisionBenchmarkDescriptor> CreateDescriptors(int bodyCount)
    {
        int mixedPerDimension = bodyCount / 2;
        int colliderOrdinal = 0;

        for (int i = 0; i < bodyCount; i++)
        {
            Vector3d sparse3D = Sparse3DPosition(i);
            Vector3d dense3D = Dense3DPosition(i);
            Vector2d sparse2D = sparse3D.ToVector2d();
            Vector2d dense2D = dense3D.ToVector2d();

            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.SparsePure3D, i, colliderOrdinal++, sparse3D, sparse2D);
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.DensePure3D, i, colliderOrdinal++, dense3D, dense2D);
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.SparsePure2D, i, colliderOrdinal++, sparse3D, sparse2D);
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.DensePure2D, i, colliderOrdinal++, dense3D, dense2D);
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.ShapeExactPure3D, i, colliderOrdinal++, sparse3D, sparse2D);
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.ShapeExactPure2D, i, colliderOrdinal++, sparse3D, sparse2D);
        }

        for (int i = 0; i < mixedPerDimension; i++)
        {
            Vector3d sparse3D = Sparse3DPosition(i);
            Vector2d sparse2D = SparseMixed2DPosition(i);
            Vector3d dense3D = DenseMixed3DPosition(i);
            Vector2d dense2D = DenseMixed2DPosition(i);

            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.SparseMixed3D, i, colliderOrdinal++, sparse3D, sparse3D.ToVector2d());
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.SparseMixed2D, i, colliderOrdinal++, sparse2D.ToVector3d(Fixed64.Zero), sparse2D);
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.DenseMixed3D, i, colliderOrdinal++, dense3D, dense3D.ToVector2d());
            yield return CreateDescriptor(ContinuousCollisionBenchmarkSet.DenseMixed2D, i, colliderOrdinal++, dense2D.ToVector3d(Fixed64.Zero), dense2D);
        }
    }

    public static Vector3d Sparse3DPosition(int index)
    {
        int x = index % SparseColumns;
        int z = index / SparseColumns;
        return new Vector3d((Fixed64)(x * SparseSpacing), Fixed64.Zero, (Fixed64)(z * SparseSpacing));
    }

    public static Vector2d Sparse2DPosition(int index)
    {
        Vector3d position = Sparse3DPosition(index);
        return new Vector2d(position.X, position.Z);
    }

    public static Vector2d SparseMixed2DPosition(int index) =>
        Sparse2DPosition(index) + new Vector2d(Fixed64.Zero, (Fixed64)MixedSparseOffsetZ);

    public static Vector3d Dense3DPosition(int index)
    {
        int pair = index / 2;
        int side = index & 1;
        int x = pair % DensePairColumns;
        int z = pair / DensePairColumns;
        Fixed64 centerX = (Fixed64)(x * DensePairSpacing);
        Fixed64 offsetX = side == 0 ? (Fixed64)(-2) : (Fixed64)2;
        return new Vector3d(centerX + offsetX, Fixed64.Zero, (Fixed64)(z * DensePairSpacing));
    }

    public static Vector2d Dense2DPosition(int index)
    {
        Vector3d position = Dense3DPosition(index);
        return new Vector2d(position.X, position.Z);
    }

    public static Vector3d DenseMixed3DPosition(int index)
    {
        int x = index % DensePairColumns;
        int z = index / DensePairColumns;
        return new Vector3d((Fixed64)(x * DensePairSpacing - 2), Fixed64.Zero, (Fixed64)(z * DensePairSpacing));
    }

    public static Vector2d DenseMixed2DPosition(int index)
    {
        int x = index % DensePairColumns;
        int z = index / DensePairColumns;
        return new Vector2d((Fixed64)(x * DensePairSpacing + 2), (Fixed64)(z * DensePairSpacing));
    }

    public static int SparseExtentX(int count) =>
        Math.Max(32, SparseColumns * SparseSpacing + 32);

    public static int SparseExtentZ(int count)
    {
        int rows = (count + SparseColumns - 1) / SparseColumns;
        return Math.Max(32, rows * SparseSpacing + 32);
    }

    public static int DenseExtentX(int count) =>
        Math.Max(32, DensePairColumns * DensePairSpacing + 32);

    public static int DenseExtentZ(int count)
    {
        int pairs = count / 2;
        int rows = (pairs + DensePairColumns - 1) / DensePairColumns;
        return Math.Max(32, rows * DensePairSpacing + 32);
    }

    private static ContinuousCollisionBenchmarkDescriptor CreateDescriptor(
        ContinuousCollisionBenchmarkSet set,
        int index,
        int colliderOrdinal,
        Vector3d position3D,
        Vector2d position2D) =>
        new(set, index, colliderOrdinal, default, position3D, position2D);
}
