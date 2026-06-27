//=======================================================================
// GravitasDiagnosticEventViews.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a force diagnostic event.
/// </summary>
public readonly struct GravitasForceDeltaDiagnosticView
{
    internal GravitasForceDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int BodyId => Event.BodyId;

    public int ColliderId => Event.ColliderAId;

    public ColliderType ColliderType => Event.ColliderAType;

    public Vector3d Force => Event.Vector;

    public Vector3d AccelerationDelta => Event.PointA;

    public Fixed64 ForceMagnitude => Event.ScalarA;
}

/// <summary>
/// Typed read-only view over a torque diagnostic event.
/// </summary>
public readonly struct GravitasTorqueDeltaDiagnosticView
{
    internal GravitasTorqueDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int BodyId => Event.BodyId;

    public int ColliderId => Event.ColliderAId;

    public ColliderType ColliderType => Event.ColliderAType;

    public Vector3d Torque => Event.Vector;

    public Fixed64 TorqueMagnitude => Event.ScalarA;
}

/// <summary>
/// Typed read-only view over linear or angular velocity diagnostic events.
/// </summary>
public readonly struct GravitasVelocityDeltaDiagnosticView
{
    internal GravitasVelocityDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public GravitasDiagnosticEventKind Kind => Event.Kind;

    public int BodyId => Event.BodyId;

    public int ColliderId => Event.ColliderAId;

    public ColliderType ColliderType => Event.ColliderAType;

    public Vector3d Before => Event.Start;

    public Vector3d After => Event.End;

    public Vector3d Delta => Event.Vector;

    public Fixed64 ResultSpeed => Event.ScalarA;
}

/// <summary>
/// Typed read-only view over a ground-probe diagnostic event.
/// </summary>
public readonly struct GravitasGroundProbeDiagnosticView
{
    internal GravitasGroundProbeDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int BodyId => Event.BodyId;

    public int ColliderId => Event.ColliderAId;

    public int HitColliderId => Event.ColliderBId;

    public GravitasColliderDimension ColliderDimension => Event.ColliderADimension;

    public GravitasColliderDimension HitColliderDimension => Event.ColliderBDimension;

    public ColliderType ColliderType => Event.ColliderAType;

    public ColliderType HitColliderType => Event.ColliderBType;

    public ColliderType2D Collider2DType => Event.ColliderA2DType;

    public ColliderType2D HitCollider2DType => Event.ColliderB2DType;

    public Vector3d Start => Event.Start;

    public Vector3d End => Event.End;

    public Vector3d HitPoint => Event.PointA;

    public Vector3d Normal => Event.Vector;

    public Fixed64 Radius => Event.ScalarA;

    public Fixed64 Distance => Event.ScalarB;

    public GroundProbeMode Mode => (GroundProbeMode)Event.DataA;

    public GroundProbeMode2D Mode2D => (GroundProbeMode2D)Event.DataA;

    public bool Hit => Event.Hit;
}

/// <summary>
/// Typed read-only view over a 3D ray or swept-sphere query diagnostic event.
/// </summary>
public readonly struct GravitasRayQueryDiagnosticView
{
    internal GravitasRayQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int HitColliderId => Event.ColliderAId;

    public ColliderType HitColliderType => Event.ColliderAType;

    public Vector3d Start => Event.Start;

    public Vector3d End => Event.End;

    public Vector3d HitPoint => Event.PointA;

    public Vector3d Normal => Event.Vector;

    public Fixed64 SweepRadius => Event.ScalarA;

    public Fixed64 Distance => Event.ScalarB;

    public int LayerMaskBits => Event.DataA;

    public int HitCount => Event.DataB;

    public bool Hit => Event.Hit;
}

/// <summary>
/// Typed read-only view over a 3D X/Z circle-query diagnostic event.
/// </summary>
public readonly struct GravitasCircleQueryDiagnosticView
{
    internal GravitasCircleQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int HitColliderId => Event.ColliderAId;

    public ColliderType HitColliderType => Event.ColliderAType;

    public Vector3d Center => Event.Start;

    public Vector3d End => Event.End;

    public Vector3d HitPoint => Event.PointA;

    public Vector3d Direction => Event.Vector;

    public Fixed64 Radius => Event.ScalarA;

    public Fixed64 Distance => Event.ScalarB;

    public int LayerMaskBits => Event.DataA;

    public int HitCount => Event.DataB;

    public bool Hit => Event.Hit;
}

/// <summary>
/// Typed read-only view over query reducer quality counters.
/// </summary>
public readonly struct GravitasQuerySummaryDiagnosticView
{
    internal GravitasQuerySummaryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public GravitasColliderDimension SourceDimension => Event.ColliderADimension;

    public GravitasColliderDimension TargetDimension => Event.ColliderBDimension;

    public Vector3d Start => Event.Start;

    public Vector3d End => Event.End;

    public int ExactReducerAttempts => Event.DataA;

    public int AcceptedHits => Event.DataB;

    public int FallbackHits => (int)Event.ScalarA;

    public int RejectedConservativeCandidates => (int)Event.ScalarB;

    public bool HasConservativeFallback => FallbackHits > 0 || RejectedConservativeCandidates > 0;
}

/// <summary>
/// Typed read-only view over a 3D contact diagnostic event.
/// </summary>
public readonly struct GravitasContactDiagnosticView
{
    internal GravitasContactDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int ColliderAId => Event.ColliderAId;

    public int ColliderBId => Event.ColliderBId;

    public ColliderType ColliderAType => Event.ColliderAType;

    public ColliderType ColliderBType => Event.ColliderBType;

    public Vector3d PointA => Event.PointA;

    public Vector3d PointB => Event.PointB;

    public Vector3d Normal => Event.Vector;

    public Fixed64 Depth => Event.ScalarA;

    public int ContactCount => Event.DataA;

    public bool HasContact => Event.Hit;
}

/// <summary>
/// Typed read-only view over a 3D response-impulse diagnostic event.
/// </summary>
public readonly struct GravitasResponseImpulseDiagnosticView
{
    internal GravitasResponseImpulseDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int ColliderAId => Event.ColliderAId;

    public int ColliderBId => Event.ColliderBId;

    public ColliderType ColliderAType => Event.ColliderAType;

    public ColliderType ColliderBType => Event.ColliderBType;

    public Vector3d PointA => Event.PointA;

    public Vector3d PointB => Event.PointB;

    public Vector3d Impulse => Event.Vector;

    public Fixed64 ImpulseMagnitude => Event.ScalarA;

    public Fixed64 NormalVelocity => Event.ScalarB;
}

/// <summary>
/// Typed read-only view over a mixed 3D/2D query diagnostic event.
/// </summary>
public readonly struct GravitasMixedQueryDiagnosticView
{
    internal GravitasMixedQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int Collider3DId => Event.ColliderAId;

    public int Collider2DId => Event.ColliderBId;

    public GravitasColliderDimension Collider3DDimension => Event.ColliderADimension;

    public GravitasColliderDimension Collider2DDimension => Event.ColliderBDimension;

    public ColliderType Collider3DType => Event.ColliderAType;

    public ColliderType2D Collider2DType => Event.ColliderB2DType;

    public Vector3d Start => Event.Start;

    public Vector3d End => Event.End;

    public Vector3d Point3D => Event.PointA;

    public Vector3d Point2D => Event.PointB;

    public Vector3d Normal3DTo2D => Event.Vector;

    public Fixed64 Radius => Event.ScalarA;

    public Fixed64 Distance => Event.ScalarB;

    public int LayerMaskBits => Event.DataA;

    public int HitCount => Event.DataB;

    public bool Hit => Event.Hit;
}

/// <summary>
/// Typed read-only view over a mixed 3D/2D contact diagnostic event.
/// </summary>
public readonly struct GravitasMixedContactDiagnosticView
{
    internal GravitasMixedContactDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int Collider3DId => Event.ColliderAId;

    public int Collider2DId => Event.ColliderBId;

    public GravitasColliderDimension Collider3DDimension => Event.ColliderADimension;

    public GravitasColliderDimension Collider2DDimension => Event.ColliderBDimension;

    public ColliderType Collider3DType => Event.ColliderAType;

    public ColliderType2D Collider2DType => Event.ColliderB2DType;

    public Vector3d Point3D => Event.PointA;

    public Vector3d Point2D => Event.PointB;

    public Vector3d Normal3DTo2D => Event.Vector;

    public Fixed64 Depth => Event.ScalarA;

    public bool HasContact => Event.Hit;
}

/// <summary>
/// Typed read-only view over a mixed 3D/2D response-impulse diagnostic event.
/// </summary>
public readonly struct GravitasMixedResponseImpulseDiagnosticView
{
    internal GravitasMixedResponseImpulseDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int Collider3DId => Event.ColliderAId;

    public int Collider2DId => Event.ColliderBId;

    public GravitasColliderDimension Collider3DDimension => Event.ColliderADimension;

    public GravitasColliderDimension Collider2DDimension => Event.ColliderBDimension;

    public ColliderType Collider3DType => Event.ColliderAType;

    public ColliderType2D Collider2DType => Event.ColliderB2DType;

    public Vector3d Point3D => Event.PointA;

    public Vector3d Point2D => Event.PointB;

    public Vector3d Impulse => Event.Vector;

    public Fixed64 ImpulseMagnitude => Event.ScalarA;

    public Fixed64 NormalVelocity => Event.ScalarB;

    public int Iteration => Event.DataA;

    public int IterationLimit => Event.DataB;
}

/// <summary>
/// Typed read-only view over a mixed 3D/2D response-island diagnostic event.
/// </summary>
public readonly struct GravitasMixedResponseIslandDiagnosticView
{
    internal GravitasMixedResponseIslandDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    public GravitasDiagnosticEvent Event { get; }

    public int Frame => Event.Frame;

    public int Sequence => Event.Sequence;

    public int RootKey => Event.BodyId;

    public int ConstraintCount => Event.DataA;

    public int IterationCount => Event.DataB;

    public bool ReachedIterationLimit => Event.Hit;
}
