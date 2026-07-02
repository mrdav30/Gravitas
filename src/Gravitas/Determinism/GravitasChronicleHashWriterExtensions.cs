//=======================================================================
// GravitasChronicleHashWriterExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using Gravitas.Support;

namespace Gravitas;

/// <summary>
/// Provides Gravitas-domain replay hash writer extensions.
/// </summary>
internal static class GravitasChronicleHashWriterExtensions
{
    /// <summary>
    /// Writes a physics layer by its deterministic layer index.
    /// </summary>
    public static void WritePhysicsLayer(this ref ChronicleHashWriter writer, PhysicsLayer value)
    {
        writer.WriteInt32(value.Index);
    }

    /// <summary>
    /// Writes a physics layer mask by its deterministic bit payload.
    /// </summary>
    public static void WritePhysicsLayerMask(this ref ChronicleHashWriter writer, PhysicsLayerMask value)
    {
        writer.WriteInt32(value.Bits);
    }
}
