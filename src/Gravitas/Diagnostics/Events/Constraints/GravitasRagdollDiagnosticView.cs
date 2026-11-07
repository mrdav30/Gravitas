//=======================================================================
// GravitasRagdollDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a ragdoll active-state diagnostic event.
/// </summary>
public readonly struct GravitasRagdollDiagnosticView
{
    internal GravitasRagdollDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the state change was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the context-local ragdoll identifier.</summary>
    public int RagdollId => Event.BodyId;

    /// <summary>Gets the number of ragdoll links.</summary>
    public int LinkCount => Event.DataA;

    /// <summary>Gets the number of ragdoll joints.</summary>
    public int JointCount => Event.DataB;

    /// <summary>Gets whether the ragdoll is active.</summary>
    public bool IsActive => Event.Hit;
}
