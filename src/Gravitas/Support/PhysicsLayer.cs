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
    public const int MinIndex = 0;
    public const int MaxIndex = 31;

    [JsonInclude]
    [MemoryPackInclude]
    private int _index;

    public PhysicsLayer(int index, string? layerName = null)
    {
        ValidateIndex(index, nameof(index));
        _index = index;
        if (layerName != null)
            LayerNamesCache[index] = layerName;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int Index
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _index;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int MaskBit
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => 1 << _index;
    }

    public void Set(int index)
    {
        ValidateIndex(index, nameof(index));
        _index = index;
    }

    public readonly bool Equals(PhysicsLayer other) => _index == other._index;

    public override readonly bool Equals(object? obj) => obj is PhysicsLayer other && Equals(other);

    public override readonly int GetHashCode() => _index;

    public override readonly string ToString() => _index.ToString();

    public static bool operator ==(PhysicsLayer left, PhysicsLayer right) => left.Equals(right);

    public static bool operator !=(PhysicsLayer left, PhysicsLayer right) => !left.Equals(right);

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

    public PhysicsLayerMask(int bits)
    {
        _bits = bits;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int Bits
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _bits;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsLayerMask None => new(0);

    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsLayerMask All => new(-1);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Includes(PhysicsLayer layer) => (_bits & layer.MaskBit) != 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Includes(int layerIndex) => Includes(new PhysicsLayer(layerIndex));

    public readonly bool Equals(PhysicsLayerMask other) => _bits == other._bits;

    public override readonly bool Equals(object? obj) => obj is PhysicsLayerMask other && Equals(other);

    public override readonly int GetHashCode() => _bits;

    public override readonly string ToString() => _bits.ToString();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PhysicsLayerMask FromLayer(int layerIndex) => FromLayer(new PhysicsLayer(layerIndex));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PhysicsLayerMask FromLayer(PhysicsLayer layer) => new(layer.MaskBit);

    public static PhysicsLayerMask FromLayers(params PhysicsLayer[] layers)
    {
        int bits = 0;
        for (int i = 0; i < layers.Length; i++)
            bits |= layers[i].MaskBit;

        return new PhysicsLayerMask(bits);
    }

    public static PhysicsLayerMask Excluding(params PhysicsLayer[] excludedLayers)
    {
        int bits = -1;
        for (int i = 0; i < excludedLayers.Length; i++)
            bits &= ~excludedLayers[i].MaskBit;

        return new PhysicsLayerMask(bits);
    }

    public static bool operator ==(PhysicsLayerMask left, PhysicsLayerMask right) => left.Equals(right);

    public static bool operator !=(PhysicsLayerMask left, PhysicsLayerMask right) => !left.Equals(right);
}
