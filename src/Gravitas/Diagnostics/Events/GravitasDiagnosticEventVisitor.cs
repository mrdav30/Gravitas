//=======================================================================
// GravitasDiagnosticEventVisitor.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Adapter-friendly visitor for typed diagnostic event payloads.
/// </summary>
public abstract class GravitasDiagnosticEventVisitor
{
    /// <summary>Visits a force-delta event.</summary>
    public virtual void VisitForceDelta(in GravitasForceDeltaDiagnosticView view) { }

    /// <summary>Visits a torque-delta event.</summary>
    public virtual void VisitTorqueDelta(in GravitasTorqueDeltaDiagnosticView view) { }

    /// <summary>Visits a linear-velocity-delta event.</summary>
    public virtual void VisitLinearVelocityDelta(in GravitasVelocityDeltaDiagnosticView view) { }

    /// <summary>Visits an angular-velocity-delta event.</summary>
    public virtual void VisitAngularVelocityDelta(in GravitasVelocityDeltaDiagnosticView view) { }

    /// <summary>Visits a ground-probe event.</summary>
    public virtual void VisitGroundProbe(in GravitasGroundProbeDiagnosticView view) { }

    /// <summary>Visits a 3D ray or swept-sphere query event.</summary>
    public virtual void VisitRayQuery(in GravitasRayQueryDiagnosticView view) { }

    /// <summary>Visits a 3D X/Z circle-query event.</summary>
    public virtual void VisitCircleQuery(in GravitasCircleQueryDiagnosticView view) { }

    /// <summary>Visits a query reducer summary event.</summary>
    public virtual void VisitQuerySummary(in GravitasQuerySummaryDiagnosticView view) { }

    /// <summary>Visits a 3D contact event.</summary>
    public virtual void VisitContact(in GravitasContactDiagnosticView view) { }

    /// <summary>Visits a 3D response-impulse event.</summary>
    public virtual void VisitResponseImpulse(in GravitasResponseImpulseDiagnosticView view) { }

    /// <summary>Visits a mixed 3D/2D query event.</summary>
    public virtual void VisitMixedQuery(in GravitasMixedQueryDiagnosticView view) { }

    /// <summary>Visits a mixed 3D/2D contact event.</summary>
    public virtual void VisitMixedContact(in GravitasMixedContactDiagnosticView view) { }

    /// <summary>Visits a mixed 3D/2D response-impulse event.</summary>
    public virtual void VisitMixedResponseImpulse(in GravitasMixedResponseImpulseDiagnosticView view) { }

    /// <summary>Visits a mixed response-island event.</summary>
    public virtual void VisitMixedResponseIsland(in GravitasMixedResponseIslandDiagnosticView view) { }

    /// <summary>Visits a joint lifecycle or solver event.</summary>
    public virtual void VisitJoint(in GravitasJointDiagnosticView view) { }

    /// <summary>Visits a ragdoll event.</summary>
    public virtual void VisitRagdoll(in GravitasRagdollDiagnosticView view) { }

    /// <summary>Visits an event whose kind is not recognized.</summary>
    public virtual void VisitUnknown(in GravitasDiagnosticEvent diagnosticEvent) { }
}
