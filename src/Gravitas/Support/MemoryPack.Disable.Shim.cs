// This shim allows consumers to disable MemoryPack entirely, and use their own serialization solution instead.
#if GRAVITAS_DISABLE_MEMORYPACK
using System;

namespace MemoryPack
{
    [AttributeUsage(AttributeTargets.Class | AttributeTargets.Struct)]
    internal sealed class MemoryPackableAttribute : Attribute
    {
    }

    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property)]
    internal sealed class MemoryPackIncludeAttribute : Attribute
    {
    }

    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property)]
    internal sealed class MemoryPackIgnoreAttribute : Attribute
    {
    }

    [AttributeUsage(AttributeTargets.Constructor)]
    internal sealed class MemoryPackConstructorAttribute : Attribute
    {
    }

    [AttributeUsage(AttributeTargets.Class | AttributeTargets.Struct)]
    internal sealed class MemoryPackAllowSerializeAttribute : Attribute
    {
    }

    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property)]
    internal sealed class MemoryPackOrderAttribute : Attribute
    {
#pragma warning disable IDE0060 // Remove unused parameter
        public MemoryPackOrderAttribute(ushort order)
#pragma warning restore IDE0060 // Remove unused parameter
        {
        }

#pragma warning disable IDE0060 // Remove unused parameter
        public MemoryPackOrderAttribute(int order)
#pragma warning restore IDE0060 // Remove unused parameter
        {
        }
    }
}

#endif