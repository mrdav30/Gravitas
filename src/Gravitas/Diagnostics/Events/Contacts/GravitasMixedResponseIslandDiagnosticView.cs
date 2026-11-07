//=======================================================================
// GravitasMixedResponseIslandDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a mixed 3D/2D response-island diagnostic event.
/// </summary>
public readonly struct GravitasMixedResponseIslandDiagnosticView
{
    internal GravitasMixedResponseIslandDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the island result was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the deterministic root key of the response island.</summary>
    public int RootKey => Event.BodyId;

    /// <summary>Gets the number of constraints in the island.</summary>
    public int ConstraintCount => Event.DataA;

    /// <summary>Gets the number of response iterations performed.</summary>
    public int IterationCount => Event.DataB;

    /// <summary>Gets whether the response reached its iteration limit.</summary>
    public bool ReachedIterationLimit => Event.Hit;
}
