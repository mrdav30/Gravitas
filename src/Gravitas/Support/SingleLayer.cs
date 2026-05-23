using MemoryPack;
using SwiftCollections;
using System;
using System.Text.Json.Serialization;

namespace Gravitas.Support;

[Serializable]
[MemoryPackable]
public partial struct SingleLayer
{
    [JsonInclude]
    [MemoryPackInclude]
    private int m_LayerIndex = 0;

    public SingleLayer(int layerIndex, string? layerName = null)
    {
        m_LayerIndex = layerIndex;
        if (layerName != null)
            LayerNamesCache[layerIndex] = layerName;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public int LayerIndex => m_LayerIndex;

    [JsonIgnore]
    [MemoryPackIgnore]
    public int Mask => 1 << m_LayerIndex;

    public void Set(int _layerIndex)
    {
        if (_layerIndex > 0 && _layerIndex < 32)
            m_LayerIndex = _layerIndex;
    }

    public static implicit operator int(SingleLayer mask)
    {
        return mask.Mask;
    }

    public static implicit operator SingleLayer(int intVal)
    {
        SingleLayer result = new()
        {
            m_LayerIndex = intVal
        };
        return result;
    }

    public static SwiftDictionary<int, string> LayerNamesCache = new();

    /// <summary>
    /// Given a layer number, returns the name of the layer as defined in either a Builtin or a User Layer in the Unity Editor.
    /// </summary>
    public static string? LayerToName(int layer)
    {
        if (LayerNamesCache.TryGetValue(layer, out string name))
            return name;
        return null;
    }

    /// <summary>
    /// Given a layer name, returns the layer index as defined by either a Builtin or a User Layer in the Unity Editor. Returns -1 if the layer name is invalid.
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
}
