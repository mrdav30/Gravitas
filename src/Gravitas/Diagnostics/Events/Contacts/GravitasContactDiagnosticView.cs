//=======================================================================
// GravitasContactDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a 3D contact diagnostic event.
/// </summary>
public readonly struct GravitasContactDiagnosticView
{
    internal GravitasContactDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the contact was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the first context-local collider identifier.</summary>
    public int ColliderAId => Event.ColliderAId;

    /// <summary>Gets the second context-local collider identifier.</summary>
    public int ColliderBId => Event.ColliderBId;

    /// <summary>Gets the first collider type.</summary>
    public ColliderType ColliderAType => Event.ColliderAType;

    /// <summary>Gets the second collider type.</summary>
    public ColliderType ColliderBType => Event.ColliderBType;

    /// <summary>Gets the materialized contact point on the first collider.</summary>
    public Vector3d PointA => Event.PointA;

    /// <summary>Gets the materialized contact point on the second collider.</summary>
    public Vector3d PointB => Event.PointB;

    /// <summary>Gets whether <see cref="PointA"/> contains a materialized point.</summary>
    public bool HasPointA => Event.HasPointA;

    /// <summary>Gets whether <see cref="PointB"/> contains a materialized point.</summary>
    public bool HasPointB => Event.HasPointB;

    /// <summary>Gets the contact normal.</summary>
    public Vector3d Normal => Event.Vector;

    /// <summary>Gets the contact penetration depth.</summary>
    public Fixed64 Depth => Event.ScalarA;

    /// <summary>Gets the number of contacts in the manifold.</summary>
    public int ContactCount => Event.DataA;

    /// <summary>Gets whether the reported penetration depth was clamped.</summary>
    public bool DepthIsClamped => Event.DataB != 0;

    /// <summary>Gets whether the collision produced a contact.</summary>
    public bool HasContact => Event.Hit;
}
