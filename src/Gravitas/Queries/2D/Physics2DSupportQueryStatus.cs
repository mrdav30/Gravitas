namespace Gravitas.Queries;

/// <summary>Distinguishes completed support queries from queries that could not be executed.</summary>
public enum Physics2DSupportQueryStatus
{
    /// <summary>A qualifying surface was found.</summary>
    Found,
    /// <summary>The complete probe found no qualifying surface.</summary>
    NoSupport,
    /// <summary>The context is not at a stable between-frame query boundary.</summary>
    WorldNotReady,
    /// <summary>The probe endpoint or padded bounds cannot be represented.</summary>
    Unrepresentable
}
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
