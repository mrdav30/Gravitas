//=======================================================================
// PhysicsQueryBudget.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <summary>Specifies independent deterministic work limits for a bounded physics query.</summary>
public readonly struct PhysicsQueryBudget
{
    /// <summary>Creates a budget from discovery, candidate, and geometry allowances.</summary>
    public PhysicsQueryBudget(ulong discoveryLimit, ulong candidateLimit, ulong geometryLimit)
    {
        DiscoveryLimit = discoveryLimit;
        CandidateLimit = candidateLimit;
        GeometryLimit = geometryLimit;
    }

    /// <summary>Gets the allowance for grid, voxel, and partition discovery.</summary>
    public ulong DiscoveryLimit { get; }

    /// <summary>Gets the allowance for membership and candidate identity inspections.</summary>
    public ulong CandidateLimit { get; }

    /// <summary>Gets the allowance for bounded geometry stages.</summary>
    public ulong GeometryLimit { get; }
}
