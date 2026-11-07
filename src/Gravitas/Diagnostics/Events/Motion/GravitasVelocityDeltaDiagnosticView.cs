//=======================================================================
// GravitasVelocityDeltaDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over linear or angular velocity diagnostic events.
/// </summary>
public readonly struct GravitasVelocityDeltaDiagnosticView
{
    internal GravitasVelocityDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the velocity change was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets whether the event describes linear or angular velocity.</summary>
    public GravitasDiagnosticEventKind Kind => Event.Kind;

    /// <summary>Gets the context-local body identifier.</summary>
    public int BodyId => Event.BodyId;

    /// <summary>Gets the body's context-local collider identifier.</summary>
    public int ColliderId => Event.ColliderAId;

    /// <summary>Gets the body's collider type.</summary>
    public ColliderType ColliderType => Event.ColliderAType;

    /// <summary>Gets the velocity before the change.</summary>
    public Vector3d Before => Event.Start;

    /// <summary>Gets the velocity after the change.</summary>
    public Vector3d After => Event.End;

    /// <summary>Gets the velocity delta.</summary>
    public Vector3d Delta => Event.Vector;

    /// <summary>Gets the magnitude of the resulting velocity.</summary>
    public Fixed64 ResultSpeed => Event.ScalarA;
}
