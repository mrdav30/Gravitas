//=======================================================================
// GravitasJointDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a joint diagnostic event.
/// </summary>
public readonly struct GravitasJointDiagnosticView
{
    internal GravitasJointDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the joint event was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the joint event kind.</summary>
    public GravitasDiagnosticEventKind Kind => Event.Kind;

    /// <summary>Gets the context-local joint identifier.</summary>
    public int JointId => Event.JointId;

    /// <summary>Gets the first endpoint's context-local collider identifier.</summary>
    public int ColliderAId => Event.ColliderAId;

    /// <summary>Gets the second endpoint's context-local collider identifier.</summary>
    public int ColliderBId => Event.ColliderBId;

    /// <summary>Gets the first endpoint collider's dimension.</summary>
    public GravitasColliderDimension ColliderADimension => Event.ColliderADimension;

    /// <summary>Gets the second endpoint collider's dimension.</summary>
    public GravitasColliderDimension ColliderBDimension => Event.ColliderBDimension;

    /// <summary>Gets the first endpoint's 3D collider type.</summary>
    public ColliderType ColliderAType => Event.ColliderAType;

    /// <summary>Gets the second endpoint's 3D collider type.</summary>
    public ColliderType ColliderBType => Event.ColliderBType;

    /// <summary>Gets the first endpoint's 2D collider type.</summary>
    public ColliderType2D ColliderA2DType => Event.ColliderA2DType;

    /// <summary>Gets the second endpoint's 2D collider type.</summary>
    public ColliderType2D ColliderB2DType => Event.ColliderB2DType;

    /// <summary>Gets the accumulated impulse magnitude for an impulse event.</summary>
    public Fixed64 ImpulseMagnitude => Event.ScalarA;

    /// <summary>Gets the limit error reported by an impulse or limit event.</summary>
    public Fixed64 LimitError => Kind == GravitasDiagnosticEventKind.JointLimitReached
        ? Event.ScalarB
        : LimitErrorMagnitude;

    /// <summary>Gets the linear anchor-error magnitude for an impulse event.</summary>
    public Fixed64 LinearAnchorErrorMagnitude => Event.ScalarB;

    /// <summary>Gets the motor impulse magnitude for an impulse event.</summary>
    public Fixed64 MotorImpulseMagnitude => Event.Vector.X;

    /// <summary>Gets the motor error magnitude for an impulse event.</summary>
    public Fixed64 MotorErrorMagnitude => Event.Vector.Y;

    /// <summary>Gets the limit-error magnitude stored by an impulse event.</summary>
    public Fixed64 LimitErrorMagnitude => Event.Vector.Z;

    /// <summary>Gets the prepared solver-row count for an impulse event.</summary>
    public int RowCount => Event.DataA;

    /// <summary>Gets the clamped solver-row count for an impulse event.</summary>
    public int ClampedRowCount => Event.DataB;
}
