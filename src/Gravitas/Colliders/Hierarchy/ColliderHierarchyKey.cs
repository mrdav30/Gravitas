//=======================================================================
// ColliderHierarchyKey.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal readonly struct ColliderHierarchyKey : IEquatable<ColliderHierarchyKey>
{
    private const byte NoneDimension = 0;
    private const byte ThreeDDimension = 1;
    private const byte TwoDDimension = 2;

    public static readonly ColliderHierarchyKey None = new(NoneDimension, -1);

    private readonly byte _dimension;
    private readonly int _id;

    private ColliderHierarchyKey(byte dimension, int id)
    {
        _dimension = dimension;
        _id = id;
    }

    public byte Dimension
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _dimension;
    }

    public int Id
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _id;
    }

    public ulong Packed
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => IsValid ? ((ulong)_dimension << 32) | (uint)_id : 0UL;
    }

    public bool IsValid
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _dimension != NoneDimension && _id >= 0;
    }

    public bool Is3D
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _dimension == ThreeDDimension;
    }

    public bool Is2D
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _dimension == TwoDDimension;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ColliderHierarchyKey Create3D(int id)
    {
        SwiftThrowHelper.ThrowIfNegative(id, nameof(id));
        return new ColliderHierarchyKey(ThreeDDimension, id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ColliderHierarchyKey Create2D(int id)
    {
        SwiftThrowHelper.ThrowIfNegative(id, nameof(id));
        return new ColliderHierarchyKey(TwoDDimension, id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ColliderHierarchyKey FromPacked(ulong packed)
    {
        if (packed == 0UL)
            return None;

        return new ColliderHierarchyKey((byte)(packed >> 32), (int)(uint)packed);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(ColliderHierarchyKey other) => _dimension == other._dimension && _id == other._id;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is ColliderHierarchyKey other && Equals(other);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => Packed.GetHashCode();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(ColliderHierarchyKey left, ColliderHierarchyKey right) => left.Equals(right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(ColliderHierarchyKey left, ColliderHierarchyKey right) => !left.Equals(right);
}
