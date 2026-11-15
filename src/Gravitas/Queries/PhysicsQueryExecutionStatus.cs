//=======================================================================
// PhysicsQueryExecutionStatus.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <summary>Distinguishes complete bounded query results from incomplete attempts.</summary>
public enum PhysicsQueryExecutionStatus : byte
{
    /// <summary>The query could not acquire a ready world view.</summary>
    WorldNotReady,
    /// <summary>The query completed and its result may be used.</summary>
    Completed,
    /// <summary>The next operation exceeded one of the supplied work limits.</summary>
    WorkBudgetExceeded,
    /// <summary>The caller's scratch could not retain the next result.</summary>
    ScratchCapacityExceeded,
    /// <summary>The acquired world view changed before the query completed.</summary>
    Stale,
    /// <summary>The query geometry or bounds could not be represented.</summary>
    Unrepresentable,
    /// <summary>The query requires its dimension's pure runtime mode.</summary>
    UnsupportedRuntimeMode,
    /// <summary>The query encountered a grid outside its supported topology or storage.</summary>
    UnsupportedTopology,
    /// <summary>The query encountered eligible geometry outside its supported shapes.</summary>
    UnsupportedGeometry
}
