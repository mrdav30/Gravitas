//=======================================================================
// PhysicsQueryHitRange.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <summary>
/// Identifies the contiguous hits produced by one request inside a shared batch hit buffer.
/// </summary>
public readonly struct PhysicsQueryHitRange
{
    /// <summary>
    /// Creates a range within a shared batch hit buffer.
    /// </summary>
    public PhysicsQueryHitRange(int start, int count)
    {
        Start = start;
        Count = count;
    }

    /// <summary>
    /// Gets the first hit index in the shared hit buffer.
    /// </summary>
    public int Start { get; }

    /// <summary>
    /// Gets the number of hits written for the request.
    /// </summary>
    public int Count { get; }

    /// <summary>
    /// Gets the exclusive end index in the shared hit buffer.
    /// </summary>
    public int End => Start + Count;

    /// <summary>
    /// Gets whether the request produced at least one hit.
    /// </summary>
    public bool HasHits => Count > 0;
}
