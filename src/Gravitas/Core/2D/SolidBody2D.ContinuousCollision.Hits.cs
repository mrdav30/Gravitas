//=======================================================================
// SolidBody2D.ContinuousCollision.Hits.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
            Fixed64 hitMixedDistance = foundMixed
                ? hitMixed.Distance
                : Fixed64.Zero;
            bool foundDynamicMixed = TryGetFirstDynamicMixedContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                out DynamicMixedIntervalHit dynamicMixed);
            if (DynamicMixedIntervalHit.ShouldReplaceStatic(
                    dynamicMixed,
                    foundDynamicMixed,
                    hitMixed,
                    foundMixed))
            {
                hitMixed = dynamicMixed.ExactHit;
                hitMixedDistance = dynamicMixed.SafeDistance;
                foundMixed = true;
                hitMixedKind = dynamicMixed.Status
                    == ContinuousCollisionMath.IntervalSearchStatus.ExactHit
                    ? ContinuousCollisionTargetKind.Dynamic3D
                    : ContinuousCollisionTargetKind.UnresolvedMixed;
            }

            if (DynamicMixedIntervalHit.ShouldSelect2D(
                    found2D,
                    hit2D.Distance,
                    foundMixed,
                    hitMixedDistance))
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
                normal = hitMixedKind == ContinuousCollisionTargetKind.UnresolvedMixed
                    ? Vector2d.Zero
                    : hitMixed.NormalFor2DSource;
                distance = hitMixedDistance;
                targetKind = hitMixedKind;
                target2D = null;
                target3D = hitMixedKind == ContinuousCollisionTargetKind.UnresolvedMixed
                    ? null
                    : hitMixed.Collider3D;
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
                        out Fixed64 sourceSegmentLength);
                Vector2d targetStart = segment.SamplePosition(overlapStart);
                Vector2d targetEnd = segment.SamplePosition(overlapEnd);
                Vector2d targetDisplacement =
                    ContinuousCollisionSweepRange.ValidateEndpoint(
                        targetStart,
                        targetEnd,
                        out _);
                LSCircleCollider2D? sourceCircle = Collider as LSCircleCollider2D;
                LSCircleCollider2D? targetCircle = target.Collider as LSCircleCollider2D;
                bool radialPair = sourceCircle is not null && targetCircle is not null;
                Vector2d relativeDisplacement = default;
                Fixed64 relativeLength = default;
                if (!radialPair
                    && !ContinuousCollisionMath.TryGetRelativeCircleOverlapDistanceInterval(
                            sourceSegmentStart,
                            sourceSegmentDisplacement,
                            sourceRadius,
                            targetStart,
                            targetDisplacement,
                            targetRadius,
                            out _,
                            out _,
                            out relativeDisplacement,
                            out relativeLength,
                            out _,
                            out _))
                {
                    continue;
                }
                _position = sourceSegmentStart;
                target._position = targetStart;
                target._rotation = target.SampleContinuousCollisionRotation(
                    overlapStart);
                Collider.RebuildRuntimeShapeOnly();
                target.Collider.RebuildRuntimeShapeOnly();

                Physics2DHit relativeHit;
                if (radialPair)
                {
                    if (!ContinuousCollisionMath.TryGetRelativeCircleOverlapDistanceInterval(
                            sourceCircle!.Center,
                            sourceSegmentDisplacement,
                            sourceCircle.ScaledRadius,
                            targetCircle!.Center,
                            targetDisplacement,
                            targetCircle.ScaledRadius,
                            out Fixed64 circleDistance,
                            out _,
                            out relativeDisplacement,
                            out relativeLength,
                            out Vector2d circleNormal,
                            out _))
                    {
                        continue;
                    }

                    relativeHit = new Physics2DHit(
                        targetCircle,
                        new ContactAnchor2D(
                            targetCircle.Center,
                            circleNormal * targetCircle.ScaledRadius),
                        circleNormal,
                        circleDistance);
                }
                else if (!QueryDetection2D.TrySweepMoverShape(
                            Collider,
                            relativeDisplacement,
                            target.Collider,
                            out relativeHit))
                {
                    continue;
                }

                Fixed64 successorStart = segmentIndex + 1
                    < target.ContinuousCollisionTrajectoryCount
                    ? target.GetContinuousCollisionTrajectorySegment(segmentIndex + 1)
                        .StartFraction
                    : Fixed64.Zero;
                if (ContinuousCollisionMath.IsSupersededTranslationalBoundaryHit(
                        relativeHit.Distance >= relativeLength,
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

                ContactAnchor2D candidateAnchor = TranslateContactAnchor(
                    relativeHit.Anchor,
                    new FixedSegment2d(targetStart, targetEnd),
                    relativeHit.Distance,
                    relativeLength);
                if (!Fixed64.TryMultiplyDivide(
                        sourceSegmentLength,
                        relativeHit.Distance,
                        relativeLength,
                        out Fixed64 localSourceDistance)
                    || !Fixed64.TryMultiplyAdd(
                        sourceLength,
                        sourceStartTime,
                        localSourceDistance,
                        out Fixed64 sourceDistance))
                {
                    continue;
                }

                var candidate = new Physics2DHit(
                    target.Collider,
                    candidateAnchor,
                    relativeHit.Normal,
                    sourceDistance);
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

    private static ContactAnchor2D TranslateContactAnchor(
        ContactAnchor2D anchor,
        FixedSegment2d targetTrajectory,
        Fixed64 distance,
        Fixed64 totalDistance)
    {
        Vector2d targetAtImpact = targetTrajectory.GetPointAtDistance(
            distance,
            totalDistance);
        _ = Vector2d.TrySubtract(
            targetAtImpact,
            targetTrajectory.Start,
            out Vector2d translation);
        _ = Vector2d.TryAdd(anchor.Origin, translation, out Vector2d origin);
        return new ContactAnchor2D(
            origin,
            anchor.Rotation,
            anchor.LocalPoint,
            anchor.LocalDisplacement);
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        out DynamicMixedIntervalHit hit)
    {
        hit = default;

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement2D.Magnitude;
        Vector3d sourceStart = new(startPosition.X, Collider.MixedSlabCenterY, startPosition.Y);
        Vector3d sourceDisplacement = new(sourceDisplacement2D.X, Fixed64.Zero, sourceDisplacement2D.Y);
        Fixed64 sourceRadius = FixedMath.Max(proxyRadius, Collider.MixedHalfThickness);
        bool found = false;
        DynamicMixedIntervalHit best = default;
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
            ContinuousCollisionMath.IntervalSearchStatus status =
                TryGetDynamicMixed3DContinuousCollisionHit(
                    target,
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
                    sourceLength,
                    elapsedFrameFraction,
                    out DynamicMixedIntervalHit candidate);
            if (status
                == ContinuousCollisionMath.IntervalSearchStatus.CertifiedNoHit)
            {
                continue;
            }

            best = DynamicMixedIntervalHit.Select(
                candidate,
                best,
                ref found);
        }

        hit = best;
        return found;
    }

    private ContinuousCollisionMath.IntervalSearchStatus
        TryGetDynamicMixed3DContinuousCollisionHit(
        SolidBody target,
        Vector3d sourceStart,
        Vector3d sourceDisplacement,
        Fixed64 sourceRadius,
        Fixed64 sourceLength,
        Fixed64 queryStartFraction,
        out DynamicMixedIntervalHit hit)
    {
        hit = default;
        Vector3d sourceEnd = sourceStart + sourceDisplacement;
        Vector3d sourceDirection = sourceDisplacement.Normalized;
        Fixed64 targetRadius =
            target.ResolveContinuousCollisionProxyRadius();
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
            if (!ContinuousCollisionMath.TryGetRelativeSphereOverlapDistanceInterval(
                    sourceSegmentStart,
                    sourceSegmentDisplacement,
                    sourceRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 entryDistance,
                    out Fixed64 exitDistance,
                    out _,
                    out Fixed64 relativeLength,
                    out _,
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
                    entryDistance >= relativeLength,
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

            Fixed64 sourceStartRotation =
                SampleContinuousCollisionRotation(overlapStart);
            Fixed64 sourceEndRotation =
                SampleContinuousCollisionRotation(overlapEnd);
            Fixed64 angularDelta = CanonicalizeRotation(
                sourceEndRotation - sourceStartRotation);
            Fixed64 intervalElapsedTime =
                overlapStart * Context.DeltaTime;
            Fixed64 intervalDuration =
                (overlapEnd - overlapStart) * Context.DeltaTime;
            if (TryGetExactSphereCircleTranslationalContact(
                    target,
                    segment,
                    targetDisplacement,
                    sourceSegmentStart.ToVector2d(),
                    sourceSegmentDisplacement.ToVector2d(),
                    sourceStartRotation,
                    intervalElapsedTime,
                    intervalDuration,
                    out _,
                    out Fixed64 sourceContactDistance,
                    out MixedContact exactContact))
            {
                if (!Fixed64.TryMultiplyAdd(
                        sourceLength,
                        sourceStartTime,
                        sourceContactDistance,
                        out Fixed64 exactSourceDistance))
                {
                    continue;
                }

                var analyticHit = new PhysicsMixedHit(
                    target.Collider,
                    null,
                    exactContact.Anchor3D,
                    exactContact.Anchor2D,
                    exactContact.Normal3DTo2D,
                    PhysicsQueryReducerKind.Exact,
                    exactSourceDistance,
                    sourceDirection);
                hit = new DynamicMixedIntervalHit(
                    ContinuousCollisionMath.IntervalSearchStatus.ExactHit,
                    analyticHit,
                    analyticHit.Distance,
                    candidateClosingSpeed,
                    target.Collider.Id);
                return ContinuousCollisionMath.IntervalSearchStatus.ExactHit;
            }

            Fixed64 entryTime = entryDistance / relativeLength;
            Fixed64 exitTime = exitDistance / relativeLength;

            bool searchFound =
                TryFindEarliestMixedRotationalContinuousCollisionAgainstTarget(
                    target.Collider,
                    sourceSegmentStart.ToVector2d(),
                    sourceSegmentDisplacement.ToVector2d(),
                    sourceStartRotation,
                    angularDelta,
                    angularDelta.Abs(),
                    sourceRadius,
                    intervalElapsedTime,
                    intervalDuration,
                    entryTime,
                    exitTime,
                    out Fixed64 safeTime,
                    out MixedContact contact,
                    out bool hasContact,
                    out Fixed64 contactTime);
            if (!searchFound)
            {
                continue;
            }

            Fixed64 safeSourceTime = FixedMath.Lerp(
                sourceStartTime,
                sourceEndTime,
                safeTime);
            bool exactHit = hasContact & contactTime == safeTime;
            PhysicsMixedHit exact = exactHit
                ? new PhysicsMixedHit(
                    target.Collider,
                    null,
                    contact.Anchor3D,
                    contact.Anchor2D,
                    contact.Normal3DTo2D,
                    PhysicsQueryReducerKind.ConservativeFallback,
                    sourceLength * safeSourceTime,
                    sourceDirection)
                : default;
            ContinuousCollisionMath.IntervalSearchStatus status = exactHit
                ? ContinuousCollisionMath.IntervalSearchStatus.ExactHit
                : ContinuousCollisionMath.IntervalSearchStatus.Unresolved;
            hit = new DynamicMixedIntervalHit(
                status,
                exact,
                sourceLength * safeSourceTime,
                candidateClosingSpeed,
                target.Collider.Id);
            return status;
        }

        return ContinuousCollisionMath.IntervalSearchStatus.CertifiedNoHit;
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
