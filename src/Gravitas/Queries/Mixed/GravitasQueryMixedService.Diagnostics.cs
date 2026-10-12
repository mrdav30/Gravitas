//=======================================================================
// GravitasQueryMixedService.Diagnostics.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed query diagnostics, candidate accounting, and reducer counters.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private void EmitMixedSweepDiagnostics(
        GravitasColliderDimension sourceDimension,
        GravitasColliderDimension targetDimension,
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        int layerMaskBits,
        bool found,
        int hitCount,
        PhysicsMixedHit hit,
        bool captureReducerDiagnostics,
        QueryReducerCounters reducerCounters)
    {
        _context.Diagnostics.EmitMixedQuery(
            start,
            end,
            radius,
            layerMaskBits,
            found,
            hitCount,
            hit);

        if (!captureReducerDiagnostics)
            return;

        _context.Diagnostics.EmitQuerySummary(
            sourceDimension,
            targetDimension,
            start,
            end,
            reducerCounters.ExactReducerAttempts,
            reducerCounters.AcceptedHits,
            fallbackHits: 0,
            rejectedConservativeCandidates:
                reducerCounters.RejectedConservativeCandidates);
    }

    private bool TrySweepSphereAgainst2DCandidate(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool captureReducerDiagnostics,
        ref QueryReducerCounters reducerCounters,
        out PhysicsMixedHit candidate,
        bool requireClosingContact = false)
    {
        if (!IsEligible2DTarget(collider, excludedCollider, includeTriggers, staticTargetsOnly))
        {
            candidate = default;
            return false;
        }

        PhysicsQueryReducerKind reducerKind = default;
        if (captureReducerDiagnostics)
        {
            reducerKind = MixedQueryReducerClassifier.ClassifySweepSphereAgainst2D(collider);
            reducerCounters.RecordAttempt(reducerKind);
        }

        candidate = default;
        bool initialClosing = requireClosingContact && excludedCollider != null
            && TryGetInitialMixedCcdHit(excludedCollider, collider, direction, source3D: true, out candidate);
        if (!initialClosing && !TrySweepSphereAgainst2D(start, end, direction, length, radius, collider,
                out candidate, requireClosingContact))
        {
            if (captureReducerDiagnostics)
                reducerCounters.RecordRejected(reducerKind);
            return false;
        }

        if (captureReducerDiagnostics)
            reducerCounters.RecordAccepted();
        return true;
    }

    private bool TrySweepCircleAgainst3DCandidate(
        LSCollider collider,
        Vector2d start,
        Vector2d end,
        Vector2d direction2D,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool captureReducerDiagnostics,
        ref QueryReducerCounters reducerCounters,
        out PhysicsMixedHit candidate,
        bool requireClosingContact = false)
    {
        if (!IsEligible3DTarget(collider, excludedCollider, includeTriggers, staticTargetsOnly))
        {
            candidate = default;
            return false;
        }

        PhysicsQueryReducerKind reducerKind = default;
        if (captureReducerDiagnostics)
        {
            reducerKind = MixedQueryReducerClassifier.ClassifySweepCircleAgainst3D(collider);
            reducerCounters.RecordAttempt(reducerKind);
        }

        candidate = default;
        bool initialClosing = requireClosingContact && excludedCollider != null
            && TryGetInitialMixedCcdHit(collider, excludedCollider, direction3D, source3D: false, out candidate);
        if (!initialClosing && !TrySweepCircleAgainst3DCollider(
            collider,
            start,
            end,
            direction2D,
            length,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            excludedCollider,
            out candidate,
            requireClosingContact))
        {
            if (captureReducerDiagnostics)
                reducerCounters.RecordRejected(reducerKind);
            return false;
        }

        if (captureReducerDiagnostics)
            reducerCounters.RecordAccepted();
        return true;
    }

    private bool TryGetInitialMixedCcdHit(LSCollider collider3D, LSCollider2D collider2D,
        Vector3d direction, bool source3D, out PhysicsMixedHit hit)
    {
        hit = default;
        // A proxy's representative may hide an independent closing leaf. Keep
        // primitive analytic paths cheap; complex owners must inspect real rows.
        if (collider3D is not (LSCompoundCollider or LSMeshCollider)
            && collider2D is not LSCompoundCollider2D)
            return false;
        ContactManifold manifold = _context.CollisionScratch.MixedManifold;
        if (!CollisionDetectionMixed.TryBuildManifold(collider3D, collider2D, manifold))
            return false;
        ContactAnchor center = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        if (!MixedContact.TrySelectClosingContact(manifold, center,
                source3D ? direction : Vector3d.Zero, Vector3d.Zero,
                center, source3D ? Vector2d.Zero : direction.ToVector2d(), Fixed64.Zero,
                out MixedContact contact))
            return false;
        hit = new PhysicsMixedHit(collider3D, collider2D, contact.Anchor3D, contact.Anchor2D,
            contact.Normal3DTo2D, PhysicsQueryReducerKind.Exact, Fixed64.Zero, direction);
        return true;
    }


    private struct QueryReducerCounters
    {
        public int ExactReducerAttempts;
        public int AcceptedHits;
        public int RejectedConservativeCandidates;

        public void RecordAttempt(PhysicsQueryReducerKind reducerKind)
        {
            if (reducerKind == PhysicsQueryReducerKind.Exact)
                ExactReducerAttempts++;
        }

        public void RecordAccepted() => AcceptedHits++;

        public void RecordRejected(PhysicsQueryReducerKind reducerKind)
        {
            if (reducerKind == PhysicsQueryReducerKind.ConservativeFallback)
                RejectedConservativeCandidates++;
        }
    }
}
