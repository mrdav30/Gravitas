//=======================================================================
// PhysicsLayer.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace Gravitas.Support;

/// <summary>
/// Identifies one physics layer by index.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct PhysicsLayer : IEquatable<PhysicsLayer>
{
    /// <summary>
    /// Minimum valid physics layer index.
    /// </summary>
    public const int MinIndex = 0;

    /// <summary>
    /// Maximum valid physics layer index.
    /// </summary>
    public const int MaxIndex = 31;

    [JsonInclude]
    [MemoryPackInclude]
    private int _index;

    /// <summary>
    /// Creates a physics layer and optionally registers its display name.
    /// </summary>
    public PhysicsLayer(int index, string? layerName = null)
    {
        ValidateIndex(index, nameof(index));
        _index = index;
        if (layerName != null)
            LayerNamesCache[index] = layerName;
    }

    /// <summary>
    /// Gets the layer index.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int Index
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _index;
    }

    /// <summary>
    /// Gets the single bit corresponding to this layer index.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int MaskBit
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => 1 << _index;
    }

    /// <summary>
    /// Replaces the layer index after validating its range.
    /// </summary>
    public void Set(int index)
    {
        ValidateIndex(index, nameof(index));
        _index = index;
    }

    /// <inheritdoc />
    public readonly bool Equals(PhysicsLayer other) => _index == other._index;

    /// <inheritdoc />
    public override readonly bool Equals(object? obj) => obj is PhysicsLayer other && Equals(other);

    /// <inheritdoc />
    public override readonly int GetHashCode() => _index;

    /// <inheritdoc />
    public override readonly string ToString() => _index.ToString();

    /// <summary>
    /// Determines whether two physics layers have the same index.
    /// </summary>
    public static bool operator ==(PhysicsLayer left, PhysicsLayer right) => left.Equals(right);

    /// <summary>
    /// Determines whether two physics layers have different indices.
    /// </summary>
    public static bool operator !=(PhysicsLayer left, PhysicsLayer right) => !left.Equals(right);

    /// <summary>
    /// Maps registered layer indices to their names.
    /// </summary>
    public static SwiftDictionary<int, string> LayerNamesCache = new();

    /// <summary>
    /// Given a layer number, returns the registered layer name.
    /// </summary>
    public static string? LayerToName(int layer)
    {
        if (LayerNamesCache.TryGetValue(layer, out string name))
            return name;
        return null;
    }

    /// <summary>
    /// Given a layer name, returns the registered layer index or -1 if the layer name is invalid.
    /// </summary>
    public static int NameToLayer(string layerName)
    {
        foreach (var kvp in LayerNamesCache)
        {
            if (kvp.Value == layerName)
                return kvp.Key;
        }

        return -1;
    }

    private static void ValidateIndex(int index, string paramName)
    {
        SwiftThrowHelper.ThrowIfArgumentOutOfRange(
            index < MinIndex || index > MaxIndex,
            index,
            paramName,
            "Physics layer index must be between 0 and 31 inclusive.");
    }
}

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
