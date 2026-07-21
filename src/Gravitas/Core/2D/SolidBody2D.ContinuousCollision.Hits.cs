//=======================================================================
// SolidBody2D.ContinuousCollision.Hits.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    private bool TryGetFirstContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        out Vector2d normal,
        out Fixed64 distance,
        out ContinuousCollisionTargetKind targetKind,
        out LSCollider2D? target2D,
        out LSCollider? target3D)
    {
        Vector2d originalPosition = _position;
        try
        {
            _position = startPosition;
            Collider.RebuildRuntimeShapeOnly();

            int hitCount = Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);
            int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
                ? Context.QueryMixed.SweepCircleAgainstStatic3DAll(
                    startPosition,
                    proposedPosition,
                    proxyRadius,
                    Collider.MixedSlabCenterY,
                    Collider.MixedHalfThickness,
                    PhysicsLayerMask.All,
                    _continuousMixedCollisionHits,
                    Collider,
                    includeTriggers: false,
                    cacheTargetPartitions: true)
                : 0;

            bool found2D = TryGetFirstValidContinuousCollisionHit(startPosition, proposedPosition, hitCount, out Physics2DHit hit2D);
            ContinuousCollisionTargetKind hit2DKind = found2D
                ? ContinuousCollisionTargetKind.Static2D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamic2D = TryGetFirstDynamicContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                out Physics2DHit dynamicHit2D,
                out Fixed64 dynamicClosingSpeed2D);
            if (ContinuousCollisionCandidateOrdering.ShouldReplaceHit(dynamicHit2D, dynamicClosingSpeed2D, foundDynamic2D, found2D, hit2D, Fixed64.Zero))
            {
                hit2D = dynamicHit2D;
                found2D = true;
                hit2DKind = ContinuousCollisionTargetKind.Dynamic2D;
            }

            bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(startPosition, proposedPosition, mixedHitCount, out PhysicsMixedHit hitMixed);
            ContinuousCollisionTargetKind hitMixedKind = foundMixed
                ? ContinuousCollisionTargetKind.Static3D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamicMixed = TryGetFirstDynamicMixedContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                out PhysicsMixedHit dynamicHitMixed,
                out Fixed64 dynamicClosingSpeedMixed);
            if (ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(dynamicHitMixed, dynamicClosingSpeedMixed, foundDynamicMixed, foundMixed, hitMixed, Fixed64.Zero))
            {
                hitMixed = dynamicHitMixed;
                foundMixed = true;
                hitMixedKind = ContinuousCollisionTargetKind.Dynamic3D;
            }

            if (found2D
                && (!foundMixed
                    || ContinuousCollisionCandidateOrdering.Is2DHitFirst(
                        hit2D.Distance,
                        hitMixed.Distance)))
            {
                normal = hit2D.Normal;
                distance = hit2D.Distance;
                targetKind = hit2DKind;
                target2D = hit2D.Collider;
                target3D = null;
                return true;
            }

            if (foundMixed)
            {
                normal = hitMixed.NormalFor2DSource;
                distance = hitMixed.Distance;
                targetKind = hitMixedKind;
                target2D = null;
                target3D = hitMixed.Collider3D;
                return true;
            }
        }
        finally
        {
            _position = originalPosition;
            Collider.RebuildRuntimeShapeOnly();
        }

        normal = Vector2d.Zero;
        distance = Fixed64.Zero;
        targetKind = ContinuousCollisionTargetKind.None;
        target2D = null;
        target3D = null;
        return false;
    }

    private bool TryGetFirstValidContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        int hitCount,
        out Physics2DHit hit)
    {
        Vector2d displacement = proposedPosition - startPosition;
        bool found = false;
        Physics2DHit best = default;
        for (int i = 0; i < hitCount; i++)
        {
            Physics2DHit candidate = _continuousCollisionHits[i];
            if (!IsValidContinuousCollisionHit(candidate))
                continue;

            if (!QueryDetection2D.TrySweepMoverShape(Collider, displacement, candidate.Collider, out Physics2DHit refined)
                || !IsClosingContinuousCollisionHit(displacement, refined.Normal))
                continue;

            if (found && !Physics2DHitSorter.ComesBefore(refined, best))
                continue;

            best = refined;
            found = true;
        }

        hit = best;
        return found;
    }

    private bool TryGetFirstValidMixedContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        int hitCount,
        out PhysicsMixedHit hit)
    {
        Vector2d displacement = proposedPosition - startPosition;
        for (int i = 0; i < hitCount; i++)
        {
            PhysicsMixedHit candidate = _continuousMixedCollisionHits[i];
            if (!IsValidMixedContinuousCollisionHit(candidate.Collider3D!)
                || !IsClosingContinuousCollisionHit(displacement, candidate.NormalFor2DSource))
                continue;

            hit = candidate;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryGetFirstDynamicContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector2d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        bool found = false;
        Physics2DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            SolidBody2D target = Context.Physics2D.GetContinuousCollisionCandidate(dynamicId);
            if (!IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            if (!TryGetDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    sourceLength,
                    elapsedFrameFraction,
                    out Physics2DHit candidate,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            if (!ContinuousCollisionCandidateOrdering.ShouldReplaceHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    internal bool TryGetDynamicRelativeContinuousCollisionHit(
        SolidBody2D target,
        Vector2d sourceStart,
        Vector2d sourceDisplacement,
        Fixed64 sourceRadius,
        Fixed64 sourceLength,
        Fixed64 queryStartFraction,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;
        Vector2d sourceEnd = sourceStart + sourceDisplacement;
        Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
        bool found = false;
        Physics2DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;

        Vector2d originalSourcePosition = _position;
        Fixed64 originalSourceRotation = _rotation;
        Vector2d originalTargetPosition = target._position;
        Fixed64 originalTargetRotation = target._rotation;
        try
        {
            target.GetContinuousCollisionTrajectorySegmentRange(
                queryStartFraction,
                out int segmentStartIndex,
                out int segmentEndExclusive);
            for (int segmentIndex = segmentStartIndex;
                segmentIndex < segmentEndExclusive;
                segmentIndex++)
            {
                ContinuousCollisionMotionSegment2D segment =
                    target.GetContinuousCollisionTrajectorySegment(segmentIndex);
                ContinuousCollisionMath.ClipTranslationalTrajectoryInterval(
                    queryStartFraction,
                    segment.StartFraction,
                    segment.EndFraction,
                    out Fixed64 overlapStart,
                    out Fixed64 overlapEnd,
                    out Fixed64 sourceStartTime,
                    out Fixed64 sourceEndTime);

                Vector2d sourceSegmentStart = Vector2d.Lerp(
                    sourceStart,
                    sourceEnd,
                    sourceStartTime);
                Vector2d sourceSegmentEnd = Vector2d.Lerp(
                    sourceStart,
                    sourceEnd,
                    sourceEndTime);
                Vector2d sourceSegmentDisplacement =
                    ContinuousCollisionSweepRange.ValidateEndpoint(
                        sourceSegmentStart,
                        sourceSegmentEnd,
                        out _);
                Vector2d targetStart = segment.SamplePosition(overlapStart);
                Vector2d targetEnd = segment.SamplePosition(overlapEnd);
                Vector2d targetDisplacement =
                    ContinuousCollisionSweepRange.ValidateEndpoint(
                        targetStart,
                        targetEnd,
                        out _);
                if (!ContinuousCollisionMath.TrySweepRelativeCircles(
                        sourceSegmentStart,
                        sourceSegmentDisplacement,
                        sourceRadius,
                        targetStart,
                        targetDisplacement,
                        targetRadius,
                        out _,
                        out _,
                        out _))
                {
                    continue;
                }

                Vector2d relativeDisplacement =
                    ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
                        sourceSegmentDisplacement,
                        targetDisplacement,
                        out Fixed64 relativeLength);
                _position = sourceSegmentStart;
                target._position = targetStart;
                target._rotation = target.SampleContinuousCollisionRotation(
                    overlapStart);
                Collider.RebuildRuntimeShapeOnly();
                target.Collider.RebuildRuntimeShapeOnly();

                if (!QueryDetection2D.TrySweepMoverShape(
                        Collider,
                        relativeDisplacement,
                        target.Collider,
                        out Physics2DHit relativeHit))
                {
                    continue;
                }

                Fixed64 segmentTime = FixedMath.Clamp01(
                    relativeHit.Distance / relativeLength);
                Fixed64 successorStart = segmentIndex + 1
                    < target.ContinuousCollisionTrajectoryCount
                    ? target.GetContinuousCollisionTrajectorySegment(segmentIndex + 1)
                        .StartFraction
                    : Fixed64.Zero;
                if (ContinuousCollisionMath.IsSupersededTranslationalBoundaryHit(
                        segmentTime,
                        overlapEnd,
                        segmentIndex,
                        target.ContinuousCollisionTrajectoryCount,
                        successorStart))
                {
                    continue;
                }

                Fixed64 localClosingSpeed =
                    -Vector2d.Dot(relativeDisplacement, relativeHit.Normal);
                if (!ContinuousCollisionMath.TryNormalizeTranslationalClosingSpeed(
                        localClosingSpeed,
                        sourceEndTime - sourceStartTime,
                        out Fixed64 candidateClosingSpeed))
                    continue;

                Fixed64 sourceTime = FixedMath.Lerp(
                    sourceStartTime,
                    sourceEndTime,
                    segmentTime);
                var candidate = new Physics2DHit(
                    target.Collider,
                    relativeHit.Point + targetDisplacement * segmentTime,
                    relativeHit.Normal,
                    sourceLength * sourceTime);
                best = candidate;
                bestClosingSpeed = candidateClosingSpeed;
                found = true;
                break;
            }
        }
        finally
        {
            _position = originalSourcePosition;
            _rotation = originalSourceRotation;
            target._position = originalTargetPosition;
            target._rotation = originalTargetRotation;
            target.Collider.RebuildRuntimeShapeOnly();
            Collider.RebuildRuntimeShapeOnly();
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        out PhysicsMixedHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement2D.Magnitude;
        Vector3d sourceStart = new(startPosition.X, Collider.MixedSlabCenterY, startPosition.Y);
        Vector3d sourceDisplacement = new(sourceDisplacement2D.X, Fixed64.Zero, sourceDisplacement2D.Y);
        Fixed64 sourceRadius = FixedMath.Max(proxyRadius, Collider.MixedHalfThickness);
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(sourceStart, sourceDisplacement, sourceRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            SolidBody target = Context.Physics.GetContinuousCollisionCandidate(dynamicId);
            if (!IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            if (!TryGetDynamicMixed3DContinuousCollisionHit(
                    target,
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
                    sourceLength,
                    elapsedFrameFraction,
                    out PhysicsMixedHit candidate,
                    out Fixed64 candidateClosingSpeed))
                continue;

            if (!ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool TryGetDynamicMixed3DContinuousCollisionHit(
        SolidBody target,
        Vector3d sourceStart,
        Vector3d sourceDisplacement,
        Fixed64 sourceRadius,
        Fixed64 sourceLength,
        Fixed64 queryStartFraction,
        out PhysicsMixedHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;
        Vector3d sourceEnd = sourceStart + sourceDisplacement;
        Vector3d sourceDirection = sourceDisplacement.Normalized;
        Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        target.GetContinuousCollisionTrajectorySegmentRange(
            queryStartFraction,
            out int segmentStartIndex,
            out int segmentEndExclusive);
        for (int segmentIndex = segmentStartIndex;
            segmentIndex < segmentEndExclusive;
            segmentIndex++)
        {
            ContinuousCollisionMotionSegment3D segment =
                target.GetContinuousCollisionTrajectorySegment(segmentIndex);
            ContinuousCollisionMath.ClipTranslationalTrajectoryInterval(
                queryStartFraction,
                segment.StartFraction,
                segment.EndFraction,
                out Fixed64 overlapStart,
                out Fixed64 overlapEnd,
                out Fixed64 sourceStartTime,
                out Fixed64 sourceEndTime);

            Vector3d sourceSegmentStart = Vector3d.Lerp(
                sourceStart,
                sourceEnd,
                sourceStartTime);
            Vector3d sourceSegmentEnd = Vector3d.Lerp(
                sourceStart,
                sourceEnd,
                sourceEndTime);
            Vector3d sourceSegmentDisplacement =
                ContinuousCollisionSweepRange.ValidateEndpoint(
                    sourceSegmentStart,
                    sourceSegmentEnd,
                    out _);
            Vector3d targetStart = segment.SamplePosition(overlapStart);
            Vector3d targetEnd = segment.SamplePosition(overlapEnd);
            Vector3d targetDisplacement =
                ContinuousCollisionSweepRange.ValidateEndpoint(
                    targetStart,
                    targetEnd,
                    out _);
            if (!ContinuousCollisionMath.TrySweepRelativeSpheres(
                    sourceSegmentStart,
                    sourceSegmentDisplacement,
                    sourceRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normalForSource,
                    out Fixed64 localClosingSpeed))
            {
                continue;
            }

            Fixed64 successorStart = segmentIndex + 1
                < target.ContinuousCollisionTrajectoryCount
                ? target.GetContinuousCollisionTrajectorySegment(segmentIndex + 1)
                    .StartFraction
                : Fixed64.Zero;
            if (ContinuousCollisionMath.IsSupersededTranslationalBoundaryHit(
                    normalizedTime,
                    overlapEnd,
                    segmentIndex,
                    target.ContinuousCollisionTrajectoryCount,
                    successorStart))
            {
                continue;
            }

            if (!ContinuousCollisionMath.TryNormalizeTranslationalClosingSpeed(
                    localClosingSpeed,
                    sourceEndTime - sourceStartTime,
                    out Fixed64 candidateClosingSpeed))
                continue;

            Fixed64 sourceTime = FixedMath.Lerp(
                sourceStartTime,
                sourceEndTime,
                normalizedTime);
            Vector3d sourceCenter = Vector3d.Lerp(
                sourceSegmentStart,
                sourceSegmentEnd,
                normalizedTime);
            Vector3d targetCenter = Vector3d.Lerp(
                targetStart,
                targetEnd,
                normalizedTime);
            Vector3d point2D = sourceCenter - normalForSource * sourceRadius;
            Vector3d point3D = ContinuousCollisionMath.ResolveContactPointOnTarget(
                sourceCenter,
                targetCenter,
                normalForSource,
                targetRadius);
            var candidate = new PhysicsMixedHit(
                target.Collider,
                null,
                point3D,
                point2D,
                normalForSource,
                PhysicsQueryReducerKind.ConservativeFallback,
                sourceLength * sourceTime,
                sourceDirection);
            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
            break;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool IsEligibleDynamicContinuousCollisionTarget(SolidBody2D target)
    {
        return (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
                HasContinuousCollisionRotationalMotion))
            && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
                target.Collider,
                _continuousCollisionHandoffIgnoredCollider2D)
            && ContinuousCollisionTargetPolicy.AllowsIndexed2DTarget(
                ReferenceEquals(target, this),
                target.Active,
                target.IsDynamic,
                target.IsKinematic,
                target.IsKinematic && target.HasContinuousCollisionMotion,
                target.Collider.IsTrigger,
                Context.Physics2D.RequireCollisionPair(Collider, target.Collider));
    }

    private bool IsEligibleDynamicMixed3DTarget(SolidBody target)
    {
        return (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
                HasContinuousCollisionRotationalMotion))
            && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
                target.Collider,
                _continuousCollisionHandoffIgnoredCollider3D)
            && ContinuousCollisionTargetPolicy.AllowsMixedIndexedTarget(
                target.Active,
                target.IsDynamic,
                target.IsKinematic,
                target.IsKinematic && target.HasContinuousCollisionMotion,
                target.Collider.IsTrigger,
                Context.MixedCollisions.RequireCollisionPair(target.Collider, Collider));
    }

}
