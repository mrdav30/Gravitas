//=======================================================================
// GravitasCircleQueryDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a 3D X/Z circle-query diagnostic event.
/// </summary>
public readonly struct GravitasCircleQueryDiagnosticView
{
    internal GravitasCircleQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the query was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the nearest hit collider identifier, or <c>-1</c> when none was hit.</summary>
    public int HitColliderId => Event.ColliderAId;

    /// <summary>Gets the nearest hit collider type.</summary>
    public ColliderType HitColliderType => Event.ColliderAType;

    /// <summary>Gets the X/Z query-circle center.</summary>
    public Vector3d Center => Event.Start;

    /// <summary>Gets the endpoint of the directional-filter visualization.</summary>
    public Vector3d End => Event.End;

    /// <summary>Gets the materialized nearest hit point, or zero when unavailable.</summary>
    public Vector3d HitPoint => Event.PointA;

    /// <summary>Gets the normalized directional filter, or zero for an undirected query.</summary>
    public Vector3d Direction => Event.Vector;

    /// <summary>Gets the query-circle radius.</summary>
    public Fixed64 Radius => Event.ScalarA;

    /// <summary>Gets the nearest hit separation distance.</summary>
    public Fixed64 Distance => Event.ScalarB;

    /// <summary>Gets the query layer-mask bits.</summary>
    public int LayerMaskBits => Event.DataA;

    /// <summary>Gets the number of hits reported by the query.</summary>
    public int HitCount => Event.DataB;

    /// <summary>Gets whether the query found an overlap.</summary>
    public bool Hit => Event.Hit;
}
