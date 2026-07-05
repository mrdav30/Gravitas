//=======================================================================
// GravitasQueryMixedService.Diagnostics.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
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
            reducerCounters.FallbackHits,
            reducerCounters.RejectedConservativeCandidates);
    }

    private static bool TrySweepSphereAgainst2DCandidate(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool captureReducerDiagnostics,
        ref QueryReducerCounters reducerCounters,
        out PhysicsMixedHit candidate)
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

        if (!TrySweepSphereAgainst2D(start, direction, length, radius, collider, out candidate))
        {
            if (captureReducerDiagnostics)
                reducerCounters.RecordRejected(reducerKind);
            return false;
        }

        if (captureReducerDiagnostics)
            reducerCounters.RecordAccepted(candidate.ReducerKind);
        return true;
    }

    private bool TrySweepCircleAgainst3DCandidate(
        LSCollider collider,
        Vector2d start,
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
        out PhysicsMixedHit candidate)
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

        if (!TrySweepCircleAgainst3DCollider(
            collider,
            start,
            direction2D,
            length,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            excludedCollider,
            out candidate))
        {
            if (captureReducerDiagnostics)
                reducerCounters.RecordRejected(reducerKind);
            return false;
        }

        if (captureReducerDiagnostics)
            reducerCounters.RecordAccepted(candidate.ReducerKind);
        return true;
    }


    private struct QueryReducerCounters
    {
        public int ExactReducerAttempts;
        public int AcceptedHits;
        public int FallbackHits;
        public int RejectedConservativeCandidates;

        public void RecordAttempt(PhysicsQueryReducerKind reducerKind)
        {
            if (reducerKind == PhysicsQueryReducerKind.Exact)
                ExactReducerAttempts++;
        }

        public void RecordAccepted(PhysicsQueryReducerKind reducerKind)
        {
            AcceptedHits++;
            if (reducerKind == PhysicsQueryReducerKind.ConservativeFallback)
                FallbackHits++;
        }

        public void RecordRejected(PhysicsQueryReducerKind reducerKind)
        {
            if (reducerKind == PhysicsQueryReducerKind.ConservativeFallback)
                RejectedConservativeCandidates++;
        }
    }
}
