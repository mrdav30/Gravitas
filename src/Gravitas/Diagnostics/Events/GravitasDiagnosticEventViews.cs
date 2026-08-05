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
/// Typed read-only view over a force-delta diagnostic event.
/// </summary>
public readonly struct GravitasForceDeltaDiagnosticView
{
    internal GravitasForceDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the force was captured.</summary>
    public int Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the context-local body identifier.</summary>
    public int BodyId => Event.BodyId;

    /// <summary>Gets the body's context-local collider identifier.</summary>
    public int ColliderId => Event.ColliderAId;

    /// <summary>Gets the body's collider type.</summary>
    public ColliderType ColliderType => Event.ColliderAType;

    /// <summary>Gets the applied force.</summary>
    public Vector3d Force => Event.Vector;

    /// <summary>Gets the resulting acceleration delta.</summary>
    public Vector3d AccelerationDelta => Event.PointA;

    /// <summary>Gets the magnitude of the applied force.</summary>
    public Fixed64 ForceMagnitude => Event.ScalarA;
}

/// <summary>
/// Typed read-only view over a torque-delta diagnostic event.
/// </summary>
public readonly struct GravitasTorqueDeltaDiagnosticView
{
    internal GravitasTorqueDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the torque was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over linear or angular velocity diagnostic events.
/// </summary>
public readonly struct GravitasVelocityDeltaDiagnosticView
{
    internal GravitasVelocityDeltaDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the velocity change was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a ground-probe diagnostic event.
/// </summary>
public readonly struct GravitasGroundProbeDiagnosticView
{
    internal GravitasGroundProbeDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the probe was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a 3D ray or swept-sphere query diagnostic event.
/// </summary>
public readonly struct GravitasRayQueryDiagnosticView
{
    internal GravitasRayQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the query was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a 3D X/Z circle-query diagnostic event.
/// </summary>
public readonly struct GravitasCircleQueryDiagnosticView
{
    internal GravitasCircleQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the query was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over query reducer quality counters.
/// </summary>
public readonly struct GravitasQuerySummaryDiagnosticView
{
    internal GravitasQuerySummaryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the summary was captured.</summary>
    public int Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the query source dimension.</summary>
    public GravitasColliderDimension SourceDimension => Event.ColliderADimension;

    /// <summary>Gets the query target dimension.</summary>
    public GravitasColliderDimension TargetDimension => Event.ColliderBDimension;

    /// <summary>Gets the query start point.</summary>
    public Vector3d Start => Event.Start;

    /// <summary>Gets the query endpoint.</summary>
    public Vector3d End => Event.End;

    /// <summary>Gets the number of exact reducer attempts.</summary>
    public int ExactReducerAttempts => Event.DataA;

    /// <summary>Gets the number of accepted hits.</summary>
    public int AcceptedHits => Event.DataB;

    /// <summary>Gets the number of conservative fallback hits.</summary>
    public int FallbackHits => (int)Event.ScalarA;

    /// <summary>Gets the number of rejected conservative candidates.</summary>
    public int RejectedConservativeCandidates => (int)Event.ScalarB;

    /// <summary>Gets whether conservative fallback affected the query.</summary>
    public bool HasConservativeFallback => FallbackHits > 0 || RejectedConservativeCandidates > 0;
}

/// <summary>
/// Typed read-only view over a 3D contact diagnostic event.
/// </summary>
public readonly struct GravitasContactDiagnosticView
{
    internal GravitasContactDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the contact was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a 3D response-impulse diagnostic event.
/// </summary>
public readonly struct GravitasResponseImpulseDiagnosticView
{
    internal GravitasResponseImpulseDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the impulse was captured.</summary>
    public int Frame => Event.Frame;

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

    /// <summary>Gets the materialized impulse point on the first collider.</summary>
    public Vector3d PointA => Event.PointA;

    /// <summary>Gets the materialized impulse point on the second collider.</summary>
    public Vector3d PointB => Event.PointB;

    /// <summary>Gets whether <see cref="PointA"/> contains a materialized point.</summary>
    public bool HasPointA => Event.HasPointA;

    /// <summary>Gets whether <see cref="PointB"/> contains a materialized point.</summary>
    public bool HasPointB => Event.HasPointB;

    /// <summary>Gets the applied impulse.</summary>
    public Vector3d Impulse => Event.Vector;

    /// <summary>Gets the magnitude of the applied impulse.</summary>
    public Fixed64 ImpulseMagnitude => Event.ScalarA;

    /// <summary>Gets the relative normal velocity used by the response.</summary>
    public Fixed64 NormalVelocity => Event.ScalarB;
}

/// <summary>
/// Typed read-only view over a mixed 3D/2D query diagnostic event.
/// </summary>
public readonly struct GravitasMixedQueryDiagnosticView
{
    internal GravitasMixedQueryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the query was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a mixed 3D/2D contact diagnostic event.
/// </summary>
public readonly struct GravitasMixedContactDiagnosticView
{
    internal GravitasMixedContactDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the contact was captured.</summary>
    public int Frame => Event.Frame;

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

    /// <summary>Gets the materialized contact point on the 3D collider.</summary>
    public Vector3d Point3D => Event.PointA;

    /// <summary>Gets the materialized contact point on the 2D collider.</summary>
    public Vector3d Point2D => Event.PointB;

    /// <summary>Gets whether <see cref="Point3D"/> contains a materialized point.</summary>
    public bool HasPoint3D => Event.HasPointA;

    /// <summary>Gets whether <see cref="Point2D"/> contains a materialized point.</summary>
    public bool HasPoint2D => Event.HasPointB;

    /// <summary>Gets the contact normal directed from the 3D collider toward the 2D collider.</summary>
    public Vector3d Normal3DTo2D => Event.Vector;

    /// <summary>Gets the contact penetration depth.</summary>
    public Fixed64 Depth => Event.ScalarA;

    /// <summary>Gets whether the reported penetration depth was clamped.</summary>
    public bool DepthIsClamped => Event.DataA != 0;

    /// <summary>Gets whether the mixed collision produced a contact.</summary>
    public bool HasContact => Event.Hit;
}

/// <summary>
/// Typed read-only view over a mixed 3D/2D response-impulse diagnostic event.
/// </summary>
public readonly struct GravitasMixedResponseImpulseDiagnosticView
{
    internal GravitasMixedResponseImpulseDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the impulse was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a mixed 3D/2D response-island diagnostic event.
/// </summary>
public readonly struct GravitasMixedResponseIslandDiagnosticView
{
    internal GravitasMixedResponseIslandDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the island result was captured.</summary>
    public int Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the deterministic root key of the response island.</summary>
    public int RootKey => Event.BodyId;

    /// <summary>Gets the number of constraints in the island.</summary>
    public int ConstraintCount => Event.DataA;

    /// <summary>Gets the number of response iterations performed.</summary>
    public int IterationCount => Event.DataB;

    /// <summary>Gets whether the response reached its iteration limit.</summary>
    public bool ReachedIterationLimit => Event.Hit;
}

/// <summary>
/// Typed read-only view over a joint diagnostic event.
/// </summary>
public readonly struct GravitasJointDiagnosticView
{
    internal GravitasJointDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the joint event was captured.</summary>
    public int Frame => Event.Frame;

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

/// <summary>
/// Typed read-only view over a ragdoll active-state diagnostic event.
/// </summary>
public readonly struct GravitasRagdollDiagnosticView
{
    internal GravitasRagdollDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the state change was captured.</summary>
    public int Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the context-local ragdoll identifier.</summary>
    public int RagdollId => Event.BodyId;

    /// <summary>Gets the number of ragdoll links.</summary>
    public int LinkCount => Event.DataA;

    /// <summary>Gets the number of ragdoll joints.</summary>
    public int JointCount => Event.DataB;

    /// <summary>Gets whether the ragdoll is active.</summary>
    public bool IsActive => Event.Hit;
}
