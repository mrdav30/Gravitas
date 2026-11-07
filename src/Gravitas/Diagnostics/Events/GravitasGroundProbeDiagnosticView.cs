//=======================================================================
// GravitasGroundProbeDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a ground-probe diagnostic event.
/// </summary>
public readonly struct GravitasGroundProbeDiagnosticView
{
    internal GravitasGroundProbeDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the probe was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the context-local body identifier.</summary>
    public int BodyId => Event.BodyId;

    /// <summary>Gets the probing body's context-local collider identifier.</summary>
    public int ColliderId => Event.ColliderAId;

    /// <summary>Gets the hit collider identifier, or <c>-1</c> when no collider was hit.</summary>
    public int HitColliderId => Event.ColliderBId;

    /// <summary>Gets the probing collider's dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Event.ColliderADimension;

    /// <summary>Gets the hit collider's dimension.</summary>
    public GravitasColliderDimension HitColliderDimension => Event.ColliderBDimension;

    /// <summary>Gets the probing collider's 3D type.</summary>
    public ColliderType ColliderType => Event.ColliderAType;

    /// <summary>Gets the hit collider's 3D type.</summary>
    public ColliderType HitColliderType => Event.ColliderBType;

    /// <summary>Gets the probing collider's 2D type.</summary>
    public ColliderType2D Collider2DType => Event.ColliderA2DType;

    /// <summary>Gets the hit collider's 2D type.</summary>
    public ColliderType2D HitCollider2DType => Event.ColliderB2DType;

    /// <summary>Gets the probe start point.</summary>
    public Vector3d Start => Event.Start;

    /// <summary>Gets the probe endpoint.</summary>
    public Vector3d End => Event.End;

    /// <summary>Gets the materialized hit point, or zero when unavailable.</summary>
    public Vector3d HitPoint => Event.PointA;

    /// <summary>Gets the hit normal.</summary>
    public Vector3d Normal => Event.Vector;

    /// <summary>Gets the probe radius.</summary>
    public Fixed64 Radius => Event.ScalarA;

    /// <summary>Gets the distance to the hit.</summary>
    public Fixed64 Distance => Event.ScalarB;

    /// <summary>Gets the probe mode for a 3D event.</summary>
    public GroundProbeMode Mode => (GroundProbeMode)Event.DataA;

    /// <summary>Gets the probe mode for a 2D event.</summary>
    public GroundProbeMode2D Mode2D => (GroundProbeMode2D)Event.DataA;

    /// <summary>Gets whether the probe hit a collider.</summary>
    public bool Hit => Event.Hit;
}
