using Chronicler;
using Chronicler.Serialization;

namespace Gravitas.Tests.Serialization;

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
