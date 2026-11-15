//=======================================================================
// PhysicsQueryWork.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <summary>Reports the work charged by a bounded physics query.</summary>
public readonly struct PhysicsQueryWork
{
    /// <summary>Creates a record of charged discovery, candidate, and geometry work.</summary>
    public PhysicsQueryWork(ulong discovery, ulong candidates, ulong geometry)
    {
        Discovery = discovery;
        Candidates = candidates;
        Geometry = geometry;
    }

    /// <summary>Gets the discovery work charged.</summary>
    public ulong Discovery { get; }

    /// <summary>Gets the candidate work charged.</summary>
    public ulong Candidates { get; }

    /// <summary>Gets the geometry work charged.</summary>
    public ulong Geometry { get; }
}
