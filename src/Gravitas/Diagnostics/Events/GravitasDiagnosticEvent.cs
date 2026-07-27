//=======================================================================
// GravitasDiagnosticEvent.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System;

namespace Gravitas.Diagnostics;

/// <summary>
/// Deterministic physics diagnostic event data captured from one world context.
/// </summary>
public readonly struct GravitasDiagnosticEvent
{
    internal GravitasDiagnosticEvent(
        int frame,
        int sequence,
        GravitasDiagnosticEventKind kind,
        int bodyId,
        int jointId,
        int colliderAId,
        int colliderBId,
        GravitasColliderDimension colliderADimension,
        GravitasColliderDimension colliderBDimension,
        ColliderType colliderAType,
        ColliderType colliderBType,
        ColliderType2D colliderA2DType,
        ColliderType2D colliderB2DType,
        Vector3d start,
        Vector3d end,
        Vector3d pointA,
        Vector3d pointB,
        bool hasPointA,
        bool hasPointB,
        Vector3d vector,
        Fixed64 scalarA,
        Fixed64 scalarB,
        int dataA,
        int dataB,
        bool hit)
    {
        Frame = frame;
        Sequence = sequence;
        Kind = kind;
        BodyId = bodyId;
        JointId = jointId;
        ColliderAId = colliderAId;
        ColliderBId = colliderBId;
        ColliderADimension = colliderADimension;
        ColliderBDimension = colliderBDimension;
        ColliderAType = colliderAType;
        ColliderBType = colliderBType;
        ColliderA2DType = colliderA2DType;
        ColliderB2DType = colliderB2DType;
        Start = start;
        End = end;
        PointA = pointA;
        PointB = pointB;
        HasPointA = hasPointA;
        HasPointB = hasPointB;
        Vector = vector;
        ScalarA = scalarA;
        ScalarB = scalarB;
        DataA = dataA;
        DataB = dataB;
        Hit = hit;
    }

    public int Frame { get; }

    public int Sequence { get; }

    public GravitasDiagnosticEventKind Kind { get; }

    public int BodyId { get; }

    public int JointId { get; }

    public int ColliderAId { get; }

    public int ColliderBId { get; }

    public GravitasColliderDimension ColliderADimension { get; }

    public GravitasColliderDimension ColliderBDimension { get; }

    public ColliderType ColliderAType { get; }

    public ColliderType ColliderBType { get; }

    public ColliderType2D ColliderA2DType { get; }

    public ColliderType2D ColliderB2DType { get; }

    public Vector3d Start { get; }

    public Vector3d End { get; }

    public Vector3d PointA { get; }

    public Vector3d PointB { get; }

    /// <summary>
    /// Gets whether <see cref="PointA"/> contains a materialized point.
    /// </summary>
    public bool HasPointA { get; }

    /// <summary>
    /// Gets whether <see cref="PointB"/> contains a materialized point.
    /// </summary>
    public bool HasPointB { get; }

    public Vector3d Vector { get; }

    public Fixed64 ScalarA { get; }

    public Fixed64 ScalarB { get; }

    public int DataA { get; }

    public int DataB { get; }

    public bool Hit { get; }

    /// <summary>
    /// Dispatches this event to a typed diagnostic visitor based on <see cref="Kind"/>.
    /// </summary>
    public void DispatchTo(GravitasDiagnosticEventVisitor visitor)
    {
        if (visitor == null)
            throw new ArgumentNullException(nameof(visitor));

        switch (Kind)
        {
            case GravitasDiagnosticEventKind.ForceDelta:
                visitor.VisitForceDelta(new GravitasForceDeltaDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.TorqueDelta:
                visitor.VisitTorqueDelta(new GravitasTorqueDeltaDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.LinearVelocityDelta:
                visitor.VisitLinearVelocityDelta(new GravitasVelocityDeltaDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.AngularVelocityDelta:
                visitor.VisitAngularVelocityDelta(new GravitasVelocityDeltaDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.GroundProbe:
                visitor.VisitGroundProbe(new GravitasGroundProbeDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.RayQuery:
                visitor.VisitRayQuery(new GravitasRayQueryDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.CircleQuery:
                visitor.VisitCircleQuery(new GravitasCircleQueryDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.QuerySummary:
                visitor.VisitQuerySummary(new GravitasQuerySummaryDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.Contact:
                visitor.VisitContact(new GravitasContactDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.ResponseImpulse:
                visitor.VisitResponseImpulse(new GravitasResponseImpulseDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.MixedQuery:
                visitor.VisitMixedQuery(new GravitasMixedQueryDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.MixedContact:
                visitor.VisitMixedContact(new GravitasMixedContactDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.MixedResponseImpulse:
                visitor.VisitMixedResponseImpulse(new GravitasMixedResponseImpulseDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.MixedResponseIsland:
                visitor.VisitMixedResponseIsland(new GravitasMixedResponseIslandDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.JointRegistered:
            case GravitasDiagnosticEventKind.JointRemoved:
            case GravitasDiagnosticEventKind.JointImpulse:
            case GravitasDiagnosticEventKind.JointLimitReached:
                visitor.VisitJoint(new GravitasJointDiagnosticView(this));
                break;
            case GravitasDiagnosticEventKind.RagdollActivated:
                visitor.VisitRagdoll(new GravitasRagdollDiagnosticView(this));
                break;
            default:
                visitor.VisitUnknown(this);
                break;
        }
    }

    /// <summary>
    /// Tries to decode this event as a force-delta diagnostic view.
    /// </summary>
    public bool TryAsForceDelta(out GravitasForceDeltaDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.ForceDelta)
        {
            view = new GravitasForceDeltaDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a torque-delta diagnostic view.
    /// </summary>
    public bool TryAsTorqueDelta(out GravitasTorqueDeltaDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.TorqueDelta)
        {
            view = new GravitasTorqueDeltaDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a linear-velocity-delta diagnostic view.
    /// </summary>
    public bool TryAsLinearVelocityDelta(out GravitasVelocityDeltaDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.LinearVelocityDelta)
        {
            view = new GravitasVelocityDeltaDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as an angular-velocity-delta diagnostic view.
    /// </summary>
    public bool TryAsAngularVelocityDelta(out GravitasVelocityDeltaDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.AngularVelocityDelta)
        {
            view = new GravitasVelocityDeltaDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a ground-probe diagnostic view.
    /// </summary>
    public bool TryAsGroundProbe(out GravitasGroundProbeDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.GroundProbe)
        {
            view = new GravitasGroundProbeDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a ray or swept-sphere query diagnostic view.
    /// </summary>
    public bool TryAsRayQuery(out GravitasRayQueryDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.RayQuery)
        {
            view = new GravitasRayQueryDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as an X/Z circle-query diagnostic view.
    /// </summary>
    public bool TryAsCircleQuery(out GravitasCircleQueryDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.CircleQuery)
        {
            view = new GravitasCircleQueryDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a query quality summary diagnostic view.
    /// </summary>
    public bool TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.QuerySummary)
        {
            view = new GravitasQuerySummaryDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a 3D contact diagnostic view.
    /// </summary>
    public bool TryAsContact(out GravitasContactDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.Contact)
        {
            view = new GravitasContactDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a 3D response-impulse diagnostic view.
    /// </summary>
    public bool TryAsResponseImpulse(out GravitasResponseImpulseDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.ResponseImpulse)
        {
            view = new GravitasResponseImpulseDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a mixed 3D/2D query diagnostic view.
    /// </summary>
    public bool TryAsMixedQuery(out GravitasMixedQueryDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.MixedQuery)
        {
            view = new GravitasMixedQueryDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a mixed 3D/2D contact diagnostic view.
    /// </summary>
    public bool TryAsMixedContact(out GravitasMixedContactDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.MixedContact)
        {
            view = new GravitasMixedContactDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a mixed 3D/2D response-impulse diagnostic view.
    /// </summary>
    public bool TryAsMixedResponseImpulse(out GravitasMixedResponseImpulseDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.MixedResponseImpulse)
        {
            view = new GravitasMixedResponseImpulseDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a mixed 3D/2D response-island diagnostic view.
    /// </summary>
    public bool TryAsMixedResponseIsland(out GravitasMixedResponseIslandDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.MixedResponseIsland)
        {
            view = new GravitasMixedResponseIslandDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a 3D joint diagnostic view.
    /// </summary>
    public bool TryAsJoint(out GravitasJointDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.JointRegistered
            || Kind == GravitasDiagnosticEventKind.JointRemoved
            || Kind == GravitasDiagnosticEventKind.JointImpulse
            || Kind == GravitasDiagnosticEventKind.JointLimitReached)
        {
            view = new GravitasJointDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }

    /// <summary>
    /// Tries to decode this event as a 3D ragdoll diagnostic view.
    /// </summary>
    public bool TryAsRagdoll(out GravitasRagdollDiagnosticView view)
    {
        if (Kind == GravitasDiagnosticEventKind.RagdollActivated)
        {
            view = new GravitasRagdollDiagnosticView(this);
            return true;
        }

        view = default;
        return false;
    }
}
