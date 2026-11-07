using Xunit;

namespace Gravitas.Tests.Serialization;

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
