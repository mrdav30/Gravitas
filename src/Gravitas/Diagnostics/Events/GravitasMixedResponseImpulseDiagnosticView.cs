//=======================================================================
// GravitasMixedResponseImpulseDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a mixed 3D/2D response-impulse diagnostic event.
/// </summary>
public readonly struct GravitasMixedResponseImpulseDiagnosticView
{
    internal GravitasMixedResponseImpulseDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the impulse was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the context-local 3D collider identifier.</summary>
    public int Collider3DId => Event.ColliderAId;

    /// <summary>Gets the context-local 2D collider identifier.</summary>
    public int Collider2DId => Event.ColliderBId;

    /// <summary>Gets the 3D collider's dimension tag.</summary>
    public GravitasColliderDimension Collider3DDimension => Event.ColliderADimension;

    /// <summary>Gets the 2D collider's dimension tag.</summary>
    public GravitasColliderDimension Collider2DDimension => Event.ColliderBDimension;

    /// <summary>Gets the 3D collider type.</summary>
    public ColliderType Collider3DType => Event.ColliderAType;

    /// <summary>Gets the 2D collider type.</summary>
    public ColliderType2D Collider2DType => Event.ColliderB2DType;

    /// <summary>Gets the materialized impulse point on the 3D collider.</summary>
    public Vector3d Point3D => Event.PointA;

    /// <summary>Gets the materialized impulse point on the 2D collider.</summary>
    public Vector3d Point2D => Event.PointB;

    /// <summary>Gets whether <see cref="Point3D"/> contains a materialized point.</summary>
    public bool HasPoint3D => Event.HasPointA;

    /// <summary>Gets whether <see cref="Point2D"/> contains a materialized point.</summary>
    public bool HasPoint2D => Event.HasPointB;

    /// <summary>Gets the applied impulse.</summary>
    public Vector3d Impulse => Event.Vector;

    /// <summary>Gets the magnitude of the applied impulse.</summary>
    public Fixed64 ImpulseMagnitude => Event.ScalarA;

    /// <summary>Gets the relative normal velocity used by the response.</summary>
    public Fixed64 NormalVelocity => Event.ScalarB;

    /// <summary>Gets the response iteration that applied the impulse.</summary>
    public int Iteration => Event.DataA;

    /// <summary>Gets the configured response iteration limit.</summary>
    public int IterationLimit => Event.DataB;
}
