//=======================================================================
// SolidBody2D.ContinuousCollision.Rotational.cs
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
using System;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    private bool HasNearbyRotationalContinuousCollisionTarget(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 pivotRadius)
    {
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                startPosition,
                displacement,
                pivotRadius));
        for (int i = 0; i < candidateIds.Count; i++)
        {
            SolidBody2D target = Context.Physics2D.GetContinuousCollisionCandidate(candidateIds[i]);
            if (IsRotatingContinuousCollisionTarget(target))
            {
                return true;
            }
        }

        return false;
    }

    private bool IsRotatingContinuousCollisionTarget(SolidBody2D target)
    {
        if (ReferenceEquals(target, this)
            || !IsMovingRotationalContinuousCollisionTarget(target))
        {
            return false;
        }

        target.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        return target.HasContinuousCollisionRotationalMotion;
    }

    private bool ShouldUseRotationalContinuousCollisionArbiter(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 startRotation,
        Fixed64 proposedRotation,
        bool forceContinuous)
    {
        ContinuousCollisionMode mode = ContinuousCollisionMode.Continuous;
        if (!forceContinuous && !ShouldUseContinuousCollision(out mode))
            return false;

        Fixed64 pivotRadius = ResolveContinuousCollisionProxyRadius();
        if (pivotRadius <= Fixed64.Epsilon)
            return false;

        Vector2d displacement = proposedPosition - startPosition;
        Fixed64 angularDistance = (proposedRotation - startRotation).Abs();
        bool targetRequiresRotationalSampling = angularDistance <= Fixed64.Epsilon
            && HasNearbyRotationalContinuousCollisionTarget(
                startPosition,
                displacement,
                pivotRadius);
        if (angularDistance <= Fixed64.Epsilon && !targetRequiresRotationalSampling)
            return false;

        Fixed64 angularArcLength = angularDistance * pivotRadius;
        if (!targetRequiresRotationalSampling
            && angularArcLength <= Fixed64.Epsilon)
            return false;
        if (targetRequiresRotationalSampling)
            return true;

        if (mode != ContinuousCollisionMode.Auto)
            return true;

        return angularArcLength > pivotRadius
            || !ContinuousCollisionMath.IsWithinProxyRadius(
                displacement,
                displacement.MagnitudeSquared,
                pivotRadius);
    }

    private bool TryResolveRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation) =>
        TryResolveRotationalContinuousCollision(
            startPosition,
            ref proposedPosition,
            startRotation,
            ref proposedRotation,
            Context.DeltaTime,
            Fixed64.Zero,
            forceContinuous: false);

    private bool TryResolveRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation,
        Fixed64 remainingTime,
        Fixed64 elapsedTime,
        bool forceContinuous)
    {
        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
        LSCollider2D? originalIgnoredCollider = _continuousCollisionHandoffIgnoredCollider2D;
        LSCollider? originalIgnoredMixedCollider = _continuousCollisionHandoffIgnoredCollider3D;
        bool resolved = false;
        Vector2d currentPosition = startPosition;
        Fixed64 currentRotation = startRotation;
        Fixed64 pivotRadius = ResolveContinuousCollisionProxyRadius();
        Vector2d kinematicLinearVelocity = (proposedPosition - startPosition) / remainingTime;
        Fixed64 kinematicAngularVelocity =
            CanonicalizeRotation(proposedRotation - startRotation) / remainingTime;
        Vector2d motionSegmentStartPosition = startPosition;
        Fixed64 motionSegmentStartRotation = startRotation;
        Fixed64 motionSegmentStartElapsedTime = elapsedTime;
        Vector2d motionSegmentLinearVelocity = IsKinematic
            ? kinematicLinearVelocity
            : ProjectLinearMotion(_linearVelocity);
        Fixed64 motionSegmentAngularVelocity = IsKinematic
            ? kinematicAngularVelocity
            : _angularVelocity;
        int maxToiIterations = Context.Settings.ContinuousCollisionMaxToiIterations;
        int conservativeRefinementCount = 0;
        try
        {
            while (remainingTime > Fixed64.Epsilon)
            {
                Fixed64 segmentEndElapsedTime = elapsedTime + remainingTime;
                Fixed64 segmentElapsedTime = segmentEndElapsedTime
                    - motionSegmentStartElapsedTime;
                Vector2d segmentEnd = motionSegmentStartPosition
                    + motionSegmentLinearVelocity * segmentElapsedTime;
                Fixed64 segmentEndRotation = CanonicalizeRotation(
                    motionSegmentStartRotation
                    + motionSegmentAngularVelocity * segmentElapsedTime);
                Vector2d displacement = segmentEnd - currentPosition;
                Fixed64 angularDelta = CanonicalizeRotation(
                    segmentEndRotation - currentRotation);
                Fixed64 angularDistance = angularDelta.Abs();
                if (angularDistance <= Fixed64.Epsilon)
                {
                    angularDelta = Fixed64.Zero;
                    angularDistance = Fixed64.Zero;
                }
                int hitCount = GatherRotationalContinuousCollisionCandidates(
                    currentPosition,
                    segmentEnd,
                    displacement,
                    pivotRadius);
                Fixed64 safeTime = Fixed64.Zero;
                Contact2D contact = default;
                bool hasContact = false;
                Fixed64 contactTime = Fixed64.Zero;
                LSCollider2D? target = null;
                bool foundSameDimension = hitCount != 0
                    && TryFindEarliestRotationalContinuousCollision(
                        currentPosition,
                        displacement,
                        currentRotation,
                        angularDelta,
                        angularDistance,
                        pivotRadius,
                        elapsedTime,
                        remainingTime,
                        out safeTime,
                        out contact,
                        out hasContact,
                        out contactTime,
                        out target);
                bool foundMixed = TryFindEarliestMixedRotationalContinuousCollision(
                    currentPosition,
                    displacement,
                    currentRotation,
                    angularDelta,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    out RotationalMixed3DHit mixedHit);
                // 2D targets precede 3D targets at equal normalized time.
                bool useMixed = foundMixed
                    && (!foundSameDimension || mixedHit.SafeTime < safeTime);
                if (!foundSameDimension && !foundMixed)
                {
                    currentPosition = segmentEnd;
                    currentRotation = segmentEndRotation;
                    break;
                }

                Fixed64 conservativeTime = useMixed ? mixedHit.SafeTime : safeTime;
                Fixed64 witnessedTime = useMixed ? mixedHit.ContactTime : contactTime;
                bool responseWitnessIsEarliest = witnessedTime == conservativeTime;
                bool canResolveMovingPair = responseWitnessIsEarliest && (useMixed
                    ? mixedHit.HasContact
                        && TryApplyMixedRotationalContinuousCollisionResponse(
                            mixedHit.Target,
                            mixedHit.Contact,
                            mixedHit.ContactTime,
                            currentPosition,
                            displacement,
                            currentRotation,
                            angularDelta,
                            elapsedTime,
                            remainingTime)
                    : hasContact
                        && target!.Body is SolidBody2D targetBody
                        && TryApplyRotationalContinuousCollisionResponse(
                            targetBody,
                            contact,
                            contactTime,
                            currentPosition,
                            currentRotation,
                            displacement,
                            angularDelta,
                            elapsedTime,
                            remainingTime));
                bool deferUnresolvedWitness = !responseWitnessIsEarliest
                    && witnessedTime > conservativeTime
                    && conservativeTime > Fixed64.Epsilon
                    && conservativeRefinementCount
                        < ContinuousCollisionMath.RotationalIntervalMaxDepth;
                Fixed64 eventTime = canResolveMovingPair
                    ? witnessedTime
                    : conservativeTime;
                Fixed64 consumedTime = remainingTime * eventTime;
                Fixed64 impactElapsedTime = elapsedTime + consumedTime;
                if (!canResolveMovingPair
                    && !deferUnresolvedWitness
                    && (!CanAppendContinuousCollisionFrameSegment(impactElapsedTime)
                        || !Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this)))
                {
                    LastContinuousCollisionToiIterationLimitReached = true;
                    break;
                }

                Fixed64 impactSegmentElapsedTime = impactElapsedTime
                    - motionSegmentStartElapsedTime;
                currentPosition = motionSegmentStartPosition
                    + motionSegmentLinearVelocity * impactSegmentElapsedTime;
                currentRotation = CanonicalizeRotation(
                    motionSegmentStartRotation
                    + motionSegmentAngularVelocity * impactSegmentElapsedTime);
                resolved = true;

                Fixed64 remainingAfterImpact = remainingTime - consumedTime;
                if (deferUnresolvedWitness)
                {
                    conservativeRefinementCount++;
                    remainingTime = remainingAfterImpact;
                    elapsedTime = impactElapsedTime;
                    continue;
                }

                LastContinuousCollisionToiIterationCount++;
                if (!canResolveMovingPair)
                {
                    if (!IsKinematic)
                    {
                        StopRotationalContinuousCollision(
                            useMixed
                                ? mixedHit.Contact.Normal3DTo2D.ToVector2d()
                                : contact.Normal);
                    }
                    AppendContinuousCollisionFrameSegment(
                        currentPosition,
                        currentRotation,
                        Vector2d.Zero,
                        Fixed64.Zero,
                        impactElapsedTime);
                    Context.Physics2D.RefreshContinuousCollisionCandidate(this);
                    break;
                }

                if (!IsKinematic)
                {
                    AppendContinuousCollisionFrameSegment(
                        currentPosition,
                        currentRotation,
                        _linearVelocity,
                        _angularVelocity,
                        impactElapsedTime);
                    Context.Physics2D.RefreshContinuousCollisionCandidate(this);
                    motionSegmentStartPosition = currentPosition;
                    motionSegmentStartRotation = currentRotation;
                    motionSegmentStartElapsedTime = impactElapsedTime;
                    motionSegmentLinearVelocity = ProjectLinearMotion(_linearVelocity);
                    motionSegmentAngularVelocity = _angularVelocity;
                }

                if (useMixed)
                {
                    _continuousCollisionHandoffIgnoredCollider3D = mixedHit.Target;
                    _continuousCollisionHandoffIgnoredCollider2D = null;
                }
                else
                {
                    _continuousCollisionHandoffIgnoredCollider2D = target;
                    _continuousCollisionHandoffIgnoredCollider3D = null;
                }
                remainingTime = remainingAfterImpact;
                elapsedTime = impactElapsedTime;
                if (LastContinuousCollisionToiIterationCount >= maxToiIterations)
                {
                    LastContinuousCollisionToiIterationLimitReached = remainingTime > Fixed64.Epsilon;
                    if (IsKinematic)
                    {
                        currentPosition = proposedPosition;
                        currentRotation = proposedRotation;
                    }
                    break;
                }
            }

            proposedPosition = currentPosition;
            proposedRotation = currentRotation;
            return resolved;
        }
        finally
        {
            _continuousCollisionHandoffIgnoredCollider2D = originalIgnoredCollider;
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
            _continuousCollisionHandoffIgnoredCollider3D = originalIgnoredMixedCollider;
        }
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation)
        => TryResolveRotationalContinuousCollision(
            startPosition,
            ref proposedPosition,
            startRotation,
            ref proposedRotation,
            Context.DeltaTime,
            Fixed64.Zero,
            forceContinuous: true);

    internal int GatherRotationalContinuousCollisionCandidates(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Vector2d displacement,
        Fixed64 pivotRadius)
    {
        _rotationalContinuousCollisionCandidateIds.FastClear();
        if (pivotRadius == Fixed64.MaxValue)
            return GatherAllRegisteredRotationalContinuousCollisionCandidates();

        int staticHitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query2D.OverlapCircleAgainstStaticAll(
                startPosition,
                pivotRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                pivotRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                startPosition,
                displacement,
                pivotRadius));
        _rotationalContinuousCollisionCandidateIds.EnsureCapacity(candidateIds.Count);
        for (int i = 0; i < candidateIds.Count; i++)
            _rotationalContinuousCollisionCandidateIds.Add(candidateIds[i]);

        return staticHitCount + _rotationalContinuousCollisionCandidateIds.Count;
    }

    private int GatherAllRegisteredRotationalContinuousCollisionCandidates()
    {
        _continuousCollisionHits.FastClear();
        _rotationalContinuousCollisionCandidateIds.FastClear();
        int colliderCount = Context.Physics2D.ColliderCount;
        _continuousCollisionHits.EnsureCapacity(colliderCount);
        for (int serviceIndex = 0; serviceIndex < colliderCount; serviceIndex++)
        {
            LSCollider2D target = Context.Physics2D.GetColliderByServiceIndex(serviceIndex);
            if (target.Body is SolidBody2D targetBody
                && IsMovingRotationalContinuousCollisionTarget(targetBody))
            {
                _rotationalContinuousCollisionCandidateIds.Add(targetBody.DynamicId);
                continue;
            }

            if (!IsValidContinuousCollisionTarget(target))
                continue;

            _continuousCollisionHits.Add(new Physics2DHit(
                target,
                target.Center,
                Vector2d.Zero,
                Fixed64.Zero));
        }

        return _continuousCollisionHits.Count
            + _rotationalContinuousCollisionCandidateIds.Count;
    }

    private bool TryFindEarliestRotationalContinuousCollision(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        out Fixed64 safeTime,
        out Contact2D contact,
        out bool hasContact,
        out Fixed64 contactTime,
        out LSCollider2D? hitTarget)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        hitTarget = null;

        bool foundCollision = false;
        Fixed64 earliestTime = Fixed64.One;
        int earliestTargetId = int.MaxValue;
        Contact2D earliestContact = default;
        bool earliestHasContact = false;
        Fixed64 earliestContactTime = Fixed64.Zero;
        LSCollider2D? earliestTarget = null;

        int staticHitCount = _continuousCollisionHits.Count;
        for (int hitIndex = 0; hitIndex < staticHitCount; hitIndex++)
        {
            LSCollider2D target = _continuousCollisionHits[hitIndex].Collider;
            if (!IsValidContinuousCollisionTarget(target)
                || ColliderSettings2D.GetCollisionType(Collider.Shape, target.Shape) == CollisionType2D.None
                || !TryFindEarliestRotationalContinuousCollisionAgainstTarget(
                    target,
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    out Fixed64 candidateTime,
                    out Contact2D candidateContact,
                    out bool candidateHasContact,
                    out Fixed64 candidateContactTime))
            {
                continue;
            }

            if (!ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                    candidateTime,
                    target.Id,
                    foundCollision,
                    earliestTime,
                    earliestTargetId))
            {
                continue;
            }

            foundCollision = true;
            earliestTime = candidateTime;
            earliestTargetId = target.Id;
            earliestContact = candidateContact;
            earliestHasContact = candidateHasContact;
            earliestContactTime = candidateContactTime;
            earliestTarget = target;
        }

        for (int candidateIndex = 0;
            candidateIndex < _rotationalContinuousCollisionCandidateIds.Count;
            candidateIndex++)
        {
            int dynamicId = _rotationalContinuousCollisionCandidateIds[candidateIndex];
            SolidBody2D targetBody = Context.Physics2D.GetContinuousCollisionCandidate(dynamicId);
            if (!IsMovingRotationalContinuousCollisionTarget(targetBody)
                || ColliderSettings2D.GetCollisionType(
                    Collider.Shape,
                    targetBody.Collider.Shape) == CollisionType2D.None
                || !TryFindEarliestRotationalContinuousCollisionAgainstTarget(
                    targetBody.Collider,
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    out Fixed64 candidateTime,
                    out Contact2D candidateContact,
                    out bool candidateHasContact,
                    out Fixed64 candidateContactTime))
            {
                continue;
            }

            if (!ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                    candidateTime,
                    targetBody.Collider.Id,
                    foundCollision,
                    earliestTime,
                    earliestTargetId))
            {
                continue;
            }

            foundCollision = true;
            earliestTime = candidateTime;
            earliestTargetId = targetBody.Collider.Id;
            earliestContact = candidateContact;
            earliestHasContact = candidateHasContact;
            earliestContactTime = candidateContactTime;
            earliestTarget = targetBody.Collider;
        }

        if (!foundCollision)
            return false;

        safeTime = earliestTime;
        contact = earliestContact;
        hasContact = earliestHasContact;
        contactTime = earliestContactTime;
        hitTarget = earliestTarget;
        return true;
    }

    private bool TryFindEarliestRotationalContinuousCollisionAgainstTarget(
        LSCollider2D target,
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        out Fixed64 safeTime,
        out Contact2D contact,
        out bool hasContact,
        out Fixed64 contactTime)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        SolidBody2D? targetBody = target.Body;
        bool samplesTargetMotion = targetBody != null
            && IsMovingRotationalContinuousCollisionTarget(targetBody);
        Vector2d originalTargetPosition = targetBody?._position ?? Vector2d.Zero;
        Fixed64 originalTargetRotation = targetBody?._rotation ?? Fixed64.Zero;
        if (samplesTargetMotion)
            targetBody!.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

        try
        {
            Span<ContinuousCollisionMath.RotationalInterval> intervals =
                stackalloc ContinuousCollisionMath.RotationalInterval[
                    ContinuousCollisionMath.RotationalIntervalMaxDepth + 2];
            int intervalCount = 1;
            int processedNodeCount = 0;
            bool hasKnownContact = false;
            Fixed64 knownContactTime = Fixed64.One;
            Contact2D knownContact = default;
            intervals[0] = new ContinuousCollisionMath.RotationalInterval(
                Fixed64.Zero,
                Fixed64.One,
                depth: 0);

            while (intervalCount > 0)
            {
                ContinuousCollisionMath.RotationalInterval interval = intervals[--intervalCount];
                Fixed64 midpoint = (interval.LowerTime + interval.UpperTime) * Fixed64.Half;
                Fixed64 intervalSpan = interval.UpperTime - interval.LowerTime;
                Fixed64 midpointFrameFraction = ResolveRotationalFrameFraction(
                    elapsedTime,
                    remainingTime,
                    midpoint);
                SampleRotationalContinuousPairPose(
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    midpoint,
                    targetBody,
                    midpointFrameFraction,
                    samplesTargetMotion);
                processedNodeCount++;

                bool hasMotionBound = ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                    displacement,
                    angularDistance,
                    pivotRadius,
                    intervalSpan,
                    out Fixed64 motionBound);
                if (hasMotionBound && samplesTargetMotion)
                {
                    Fixed64 lowerFrameFraction = ResolveRotationalFrameFraction(
                        elapsedTime,
                        remainingTime,
                        interval.LowerTime);
                    Fixed64 upperFrameFraction = ResolveRotationalFrameFraction(
                        elapsedTime,
                        remainingTime,
                        interval.UpperTime);
                    Fixed64 targetRadius = targetBody!.ResolveContinuousCollisionProxyRadius();
                    bool targetBoundResolved = targetBody.TryResolveContinuousCollisionMotionBound(
                        lowerFrameFraction,
                        upperFrameFraction,
                        targetRadius,
                        out Fixed64 targetMotionBound);
                    bool combinedBoundResolved = Fixed64.TryAdd(
                        motionBound,
                        targetMotionBound,
                        out motionBound);
                    hasMotionBound = targetBoundResolved & combinedBoundResolved;
                }
                bool sampleHasContact = CollisionDetection2D.TryCollide(
                    Collider,
                    target,
                    out Contact2D sampleContact);
                if (sampleHasContact
                    && (!hasKnownContact || midpoint < knownContactTime))
                {
                    hasKnownContact = true;
                    knownContactTime = midpoint;
                    knownContact = sampleContact;
                }

                if (!sampleHasContact
                    && hasMotionBound
                    && IsRotationalIntervalSeparated(target, motionBound))
                {
                    continue;
                }

                if (ContinuousCollisionMath.TryResolveRotationalSearchLimit(
                        interval,
                        processedNodeCount,
                        hasKnownContact,
                        knownContactTime,
                        out safeTime,
                        out contactTime,
                        out hasContact))
                {
                    contact = knownContact;
                    return true;
                }

                int childDepth = interval.Depth + 1;
                if (sampleHasContact)
                {
                    // A real midpoint contact is already an upper bound. Only the
                    // earlier half can contain an earlier first contact.
                    intervals[intervalCount++] = new ContinuousCollisionMath.RotationalInterval(
                        interval.LowerTime,
                        midpoint,
                        childDepth);
                    continue;
                }

                intervals[intervalCount++] = new ContinuousCollisionMath.RotationalInterval(
                    midpoint,
                    interval.UpperTime,
                    childDepth);
                intervals[intervalCount++] = new ContinuousCollisionMath.RotationalInterval(
                    interval.LowerTime,
                    midpoint,
                    childDepth);
            }

            safeTime = knownContactTime;
            contact = knownContact;
            hasContact = hasKnownContact;
            contactTime = knownContactTime;
            return hasKnownContact;
        }
        finally
        {
            if (targetBody != null)
            {
                targetBody._position = originalTargetPosition;
                targetBody._rotation = originalTargetRotation;
                targetBody.Collider.RebuildRuntimeShapeOnly();
            }
        }
    }

    private bool IsRotationalIntervalSeparated(LSCollider2D target, Fixed64 motionBound)
    {
        if (TryGetCirclePairSeparationGap(target, out Fixed64 closestPointGap))
            return closestPointGap > motionBound;

        return ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(
            Collider.Bounds,
            target.Bounds,
            motionBound);
    }

    private bool TryGetCirclePairSeparationGap(LSCollider2D target, out Fixed64 separationGap)
    {
        if (Collider is LSCircleCollider2D sourceCircle)
            return TryGetCircleSeparationGap(sourceCircle, target, out separationGap);

        if (target is LSCircleCollider2D targetCircle)
            return TryGetCircleSeparationGap(targetCircle, Collider, out separationGap);

        separationGap = default;
        return false;
    }

    private static bool TryGetCircleSeparationGap(
        LSCircleCollider2D circle,
        LSCollider2D other,
        out Fixed64 separationGap)
    {
        LSCircleCollider2D? otherCircle = other as LSCircleCollider2D;
        Vector2d closestPoint = otherCircle?.Center ?? other.GetClosestPoint(circle.Center);
        Fixed64 otherRadius = otherCircle?.ScaledRadius ?? Fixed64.Zero;

        // Unrepresentable distance and saturating sums remain conservative:
        // they cannot enlarge the certified gap relative to its uncertainty.
        _ = Vector2d.TryGetDistance(circle.Center, closestPoint, out Fixed64 distance);
        Fixed64 combinedRadius = circle.ScaledRadius + otherRadius;
        Fixed64 rawGap = distance - combinedRadius;
        Fixed64 characteristicScale = distance
            + combinedRadius
            + Vector2d.GetMagnitude(other.Bounds.Size);
        _ = ContinuousCollisionMath.TrySubtractClosestFeatureUncertainty(
            rawGap,
            characteristicScale,
            out separationGap);

        return separationGap > Fixed64.Zero;
    }

    private void SampleRotationalContinuousPairPose(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 sampleTime,
        SolidBody2D? targetBody,
        Fixed64 targetFrameFraction,
        bool samplesTargetMotion)
    {
        _position = startPosition + displacement * sampleTime;
        _rotation = startRotation + angularDelta * sampleTime;
        Collider.RebuildRuntimeShapeOnly();

        if (!samplesTargetMotion)
            return;

        targetBody!._position = targetBody.SampleContinuousCollisionPosition(targetFrameFraction);
        targetBody._rotation = targetBody.SampleContinuousCollisionRotation(targetFrameFraction);
        targetBody.Collider.RebuildRuntimeShapeOnly();
    }

    private bool IsMovingRotationalContinuousCollisionTarget(SolidBody2D target)
    {
        return !ReferenceEquals(target, this)
            && (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
                HasContinuousCollisionRotationalMotion))
            && target.Active
            && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
                target.Collider,
                _continuousCollisionHandoffIgnoredCollider2D)
            && !target.Collider.IsTrigger
            && Context.Physics2D.RequireCollisionPair(Collider, target.Collider)
            && (target.IsKinematic || !target.IsPositionFullyFrozen);
    }

}
