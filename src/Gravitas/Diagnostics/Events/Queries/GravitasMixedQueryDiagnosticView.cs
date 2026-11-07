//=======================================================================
// GravitasMixedQueryDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a mixed 3D/2D query diagnostic event.
/// </summary>
public readonly struct GravitasMixedQueryDiagnosticView
{
    internal GravitasMixedQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the query was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the participating 3D collider identifier, or <c>-1</c> when absent.</summary>
    public int Collider3DId => Event.ColliderAId;

    /// <summary>Gets the participating 2D collider identifier, or <c>-1</c> when absent.</summary>
    public int Collider2DId => Event.ColliderBId;

    /// <summary>Gets the 3D collider's dimension tag.</summary>
    public GravitasColliderDimension Collider3DDimension => Event.ColliderADimension;

    /// <summary>Gets the 2D collider's dimension tag.</summary>
    public GravitasColliderDimension Collider2DDimension => Event.ColliderBDimension;

    /// <summary>Gets the participating 3D collider type.</summary>
    public ColliderType Collider3DType => Event.ColliderAType;

    /// <summary>Gets the participating 2D collider type.</summary>
    public ColliderType2D Collider2DType => Event.ColliderB2DType;

    /// <summary>Gets the query start point.</summary>
    public Vector3d Start => Event.Start;

    /// <summary>Gets the query endpoint.</summary>
    public Vector3d End => Event.End;

    /// <summary>Gets the materialized hit point on the 3D collider, or zero when unavailable.</summary>
    public Vector3d Point3D => Event.PointA;

    /// <summary>Gets the materialized hit point on the 2D collider, or zero when unavailable.</summary>
    public Vector3d Point2D => Event.PointB;

    /// <summary>Gets the hit normal directed from the 3D collider toward the 2D collider.</summary>
    public Vector3d Normal3DTo2D => Event.Vector;

    /// <summary>Gets the query radius.</summary>
    public Fixed64 Radius => Event.ScalarA;

    /// <summary>Gets the distance to the nearest hit.</summary>
    public Fixed64 Distance => Event.ScalarB;

    /// <summary>Gets the query layer-mask bits.</summary>
    public int LayerMaskBits => Event.DataA;

    /// <summary>Gets the number of hits reported by the query.</summary>
    public int HitCount => Event.DataB;

    /// <summary>Gets whether the query found a target.</summary>
    public bool Hit => Event.Hit;
}
