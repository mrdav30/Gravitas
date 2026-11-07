//=======================================================================
// GravitasTorqueDeltaDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a torque-delta diagnostic event.
/// </summary>
public readonly struct GravitasTorqueDeltaDiagnosticView
{
    internal GravitasTorqueDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the torque was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the context-local body identifier.</summary>
    public int BodyId => Event.BodyId;

    /// <summary>Gets the body's context-local collider identifier.</summary>
    public int ColliderId => Event.ColliderAId;

    /// <summary>Gets the body's collider type.</summary>
    public ColliderType ColliderType => Event.ColliderAType;

    /// <summary>Gets the applied torque.</summary>
    public Vector3d Torque => Event.Vector;

    /// <summary>Gets the magnitude of the applied torque.</summary>
    public Fixed64 TorqueMagnitude => Event.ScalarA;
}
