#nullable enable

using System;
using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Benchmarking;

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
