using Chronicler;
using System;
using System.Collections.Generic;

namespace Gravitas.Tests.Constraints;

internal sealed class InvalidRecordPayloadChronicler : IChronicler
{
    private readonly IReadOnlyDictionary<string, object> _values;

    public InvalidRecordPayloadChronicler(IReadOnlyDictionary<string, object> values)
    {
        _values = values;
        Context = new ChronicleContext();
    }

    public ChronicleContext Context { get; }

    public SerializationMode Mode => SerializationMode.Loading;

    public void LookValue<T>(ref T value, string name, T? defaultValue = default)
    {
        if (_values.TryGetValue(name, out object? loadedValue))
            value = (T)loadedValue;
    }

    public void LookDeep<T>(ref T value, string name) where T : class, IRecordable
    {
    }

    public void LookDeepStruct<T>(ref T value, string name) where T : struct, IRecordable
    {
    }

    public void LookNullableDeep<T>(ref T? value, string name) where T : struct, IRecordable
    {
    }

    public void LookLink<T>(
        ref T value,
        string name,
        string? slot = null,
        RecordLinkResolveMode resolveMode = RecordLinkResolveMode.Immediate,
        Action<T>? assignLoadedValue = null)
    {
    }
}
