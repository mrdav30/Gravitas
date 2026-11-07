//=======================================================================
// PhysicsLayerMask.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace Gravitas.Support;

/// <summary>
/// Represents an include mask for physics layer queries and filters.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct PhysicsLayerMask : IEquatable<PhysicsLayerMask>
{
    [JsonInclude]
    [MemoryPackInclude]
    private int _bits;

    /// <summary>
    /// Creates an include mask from its raw bit field.
    /// </summary>
    public PhysicsLayerMask(int bits)
    {
        _bits = bits;
    }

    /// <summary>
    /// Gets the raw include-mask bits.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int Bits
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _bits;
    }

    /// <summary>
    /// Gets a mask that includes no physics layers.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsLayerMask None => new(0);

    /// <summary>
    /// Gets a mask that includes every physics layer.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsLayerMask All => new(-1);

    /// <summary>
    /// Determines whether this mask includes the specified layer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Includes(PhysicsLayer layer) => (_bits & layer.MaskBit) != 0;

    /// <summary>
    /// Determines whether this mask includes the specified layer index.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Includes(int layerIndex) => Includes(new PhysicsLayer(layerIndex));

    /// <inheritdoc />
    public readonly bool Equals(PhysicsLayerMask other) => _bits == other._bits;

    /// <inheritdoc />
    public override readonly bool Equals(object? obj) => obj is PhysicsLayerMask other && Equals(other);

    /// <inheritdoc />
    public override readonly int GetHashCode() => _bits;

    /// <inheritdoc />
    public override readonly string ToString() => _bits.ToString();

    /// <summary>
    /// Creates an include mask containing one layer index.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PhysicsLayerMask FromLayer(int layerIndex) => FromLayer(new PhysicsLayer(layerIndex));

    /// <summary>
    /// Creates an include mask containing one layer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PhysicsLayerMask FromLayer(PhysicsLayer layer) => new(layer.MaskBit);

    /// <summary>
    /// Creates an include mask containing the specified layers.
    /// </summary>
    public static PhysicsLayerMask FromLayers(params PhysicsLayer[] layers)
    {
        int bits = 0;
        for (int i = 0; i < layers.Length; i++)
            bits |= layers[i].MaskBit;

        return new PhysicsLayerMask(bits);
    }

    /// <summary>
    /// Creates an include mask containing every layer except those specified.
    /// </summary>
    public static PhysicsLayerMask Excluding(params PhysicsLayer[] excludedLayers)
    {
        int bits = -1;
        for (int i = 0; i < excludedLayers.Length; i++)
            bits &= ~excludedLayers[i].MaskBit;

        return new PhysicsLayerMask(bits);
    }

    /// <summary>
    /// Determines whether two layer masks contain the same bits.
    /// </summary>
    public static bool operator ==(PhysicsLayerMask left, PhysicsLayerMask right) => left.Equals(right);

    /// <summary>
    /// Determines whether two layer masks contain different bits.
    /// </summary>
    public static bool operator !=(PhysicsLayerMask left, PhysicsLayerMask right) => !left.Equals(right);
}
