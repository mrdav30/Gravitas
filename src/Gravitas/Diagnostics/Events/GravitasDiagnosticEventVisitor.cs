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
    public virtual void VisitForceDelta(in GravitasForceDeltaDiagnosticView view) { }

    public virtual void VisitTorqueDelta(in GravitasTorqueDeltaDiagnosticView view) { }

    public virtual void VisitLinearVelocityDelta(in GravitasVelocityDeltaDiagnosticView view) { }

    public virtual void VisitAngularVelocityDelta(in GravitasVelocityDeltaDiagnosticView view) { }

    public virtual void VisitGroundProbe(in GravitasGroundProbeDiagnosticView view) { }

    public virtual void VisitRayQuery(in GravitasRayQueryDiagnosticView view) { }

    public virtual void VisitCircleQuery(in GravitasCircleQueryDiagnosticView view) { }

    public virtual void VisitQuerySummary(in GravitasQuerySummaryDiagnosticView view) { }

    public virtual void VisitContact(in GravitasContactDiagnosticView view) { }

    public virtual void VisitResponseImpulse(in GravitasResponseImpulseDiagnosticView view) { }

    public virtual void VisitMixedQuery(in GravitasMixedQueryDiagnosticView view) { }

    public virtual void VisitMixedContact(in GravitasMixedContactDiagnosticView view) { }

    public virtual void VisitMixedResponseImpulse(in GravitasMixedResponseImpulseDiagnosticView view) { }

    public virtual void VisitMixedResponseIsland(in GravitasMixedResponseIslandDiagnosticView view) { }

    public virtual void VisitUnknown(in GravitasDiagnosticEvent diagnosticEvent) { }
}
