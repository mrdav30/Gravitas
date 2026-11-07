//=======================================================================
// GravitasRayQueryDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a 3D ray or swept-sphere query diagnostic event.
/// </summary>
public readonly struct GravitasRayQueryDiagnosticView
{
    internal GravitasRayQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the query was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the hit collider identifier, or <c>-1</c> when no collider was hit.</summary>
    public int HitColliderId => Event.ColliderAId;

    /// <summary>Gets the hit collider type.</summary>
    public ColliderType HitColliderType => Event.ColliderAType;

    /// <summary>Gets the query start point.</summary>
    public Vector3d Start => Event.Start;

    /// <summary>Gets the query endpoint.</summary>
    public Vector3d End => Event.End;

    /// <summary>Gets the materialized hit point, or zero when unavailable.</summary>
    public Vector3d HitPoint => Event.PointA;

    /// <summary>Gets the hit normal.</summary>
    public Vector3d Normal => Event.Vector;

    /// <summary>Gets the sweep radius, or zero for a raycast.</summary>
    public Fixed64 SweepRadius => Event.ScalarA;

    /// <summary>Gets the distance to the nearest hit.</summary>
    public Fixed64 Distance => Event.ScalarB;

    /// <summary>Gets the query layer-mask bits.</summary>
    public int LayerMaskBits => Event.DataA;

    /// <summary>Gets the number of hits reported by the query.</summary>
    public int HitCount => Event.DataB;

    /// <summary>Gets whether the query hit a collider.</summary>
    public bool Hit => Event.Hit;
}
