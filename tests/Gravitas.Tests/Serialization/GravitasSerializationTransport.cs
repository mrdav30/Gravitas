namespace Gravitas.Tests.Serialization;

public enum GravitasSerializationTransport
{
    Json,
#if !GRAVITAS_DISABLE_MEMORYPACK
    MemoryPack
#endif
}
