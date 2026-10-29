using Chronicler;
using Chronicler.Serialization;
using Xunit;

namespace Gravitas.Tests.Serialization;

public enum GravitasSerializationTransport
{
    Json,
#if !GRAVITAS_DISABLE_MEMORYPACK
    MemoryPack
#endif
}

internal static class GravitasSerializationTransportCases
{
    public static TheoryData<GravitasSerializationTransport> All()
    {
        TheoryData<GravitasSerializationTransport> transports = new()
        {
            GravitasSerializationTransport.Json
        };

#if !GRAVITAS_DISABLE_MEMORYPACK
        transports.Add(GravitasSerializationTransport.MemoryPack);
#endif

        return transports;
    }
}

internal static class GravitasSerializationHarness
{
    public static object Serialize(IRecordable target, GravitasSerializationTransport transport, ChronicleContext? context = null)
    {
        return transport switch
        {
            GravitasSerializationTransport.Json => JsonRecordSerializer.Serialize(target, context, writeIndented: true),
#if !GRAVITAS_DISABLE_MEMORYPACK
            GravitasSerializationTransport.MemoryPack => MemoryPackRecordSerializer.Serialize(target, context),
#endif
            _ => throw new System.ArgumentOutOfRangeException(nameof(transport), transport, null)
        };
    }

    public static void Populate(IRecordable target, object payload, GravitasSerializationTransport transport, ChronicleContext? context = null)
    {
        switch (transport)
        {
            case GravitasSerializationTransport.Json:
                JsonRecordSerializer.Populate(target, (string)payload, context);
                break;
#if !GRAVITAS_DISABLE_MEMORYPACK
            case GravitasSerializationTransport.MemoryPack:
                MemoryPackRecordSerializer.Populate(target, (byte[])payload, context);
                break;
#endif
            default:
                throw new System.ArgumentOutOfRangeException(nameof(transport), transport, null);
        }
    }
}
