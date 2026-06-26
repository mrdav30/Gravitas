//=======================================================================
// PhysicsQueryReducerKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <summary>
/// Describes whether a query hit came from an exact shape reducer or a conservative fallback reducer.
/// </summary>
public enum PhysicsQueryReducerKind : byte
{
    /// <summary>
    /// The hit was accepted by a reducer that matches the documented source and target shape semantics.
    /// </summary>
    Exact = 0,

    /// <summary>
    /// The hit was accepted by a conservative fallback that should not create false negatives but may report early or extra hits.
    /// </summary>
    ConservativeFallback = 1
}
