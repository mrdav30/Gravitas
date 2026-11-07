//=======================================================================
// CollisionResponseDispatchMode.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>
/// Selects immediate 3D pair response or queues the pair for the discrete solver.
/// </summary>
internal enum CollisionResponseDispatchMode : byte
{
    Immediate,
    Deferred
}
