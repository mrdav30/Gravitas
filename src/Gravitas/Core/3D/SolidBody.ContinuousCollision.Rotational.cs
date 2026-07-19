//=======================================================================
// SolidBody.ContinuousCollision.Rotational.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas;

public partial class SolidBody
{
    private bool TryResolveRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation) =>
        TryResolveRotationalContinuousCollision(
            startPosition,
            ref proposedPosition,
            startRotation,
            ref proposedRotation,
            Context.DeltaTime,
            Fixed64.Zero,
            forceContinuous: false);

    private bool TryResolveRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation,
        Fixed64 initialRemainingTime,
        Fixed64 initialElapsedTime,
        bool forceContinuous)
    {
        ContinuousCollisionMode mode = ContinuousCollisionMode.Continuous;
        if (!forceContinuous && !ShouldUseContinuousCollision(out mode))
            return false;

        Fixed64 pivotRadius = ResolveContinuousCollisionProxyRadius();
        Vector3d initialDisplacement = proposedPosition - startPosition;
        Fixed64 angularDistance = _angularVelocity.Magnitude * initialRemainingTime;
        bool targetRequiresRotationalSampling = angularDistance <= Fixed64.Epsilon
            && HasNearbyRotationalContinuousCollisionTarget(
                startPosition,
                initialDisplacement,
                pivotRadius);
        if (angularDistance <= Fixed64.Epsilon && !targetRequiresRotationalSampling)
            return false;

        Fixed64 angularArcLength = angularDistance * pivotRadius;
        if (pivotRadius <= Fixed64.Epsilon
            || (!targetRequiresRotationalSampling
                && angularArcLength <= Fixed64.Epsilon)
            || (!targetRequiresRotationalSampling
                && mode == ContinuousCollisionMode.Auto
                && angularArcLength <= pivotRadius
                && ContinuousCollisionMath.IsWithinProxyRadius(
                    initialDisplacement,
                    initialDisplacement.MagnitudeSquared,
                    pivotRadius)))
        {
            return false;
        }

        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        LSCollider2D? originalIgnoredMixedTarget = _continuousCollisionHandoffIgnoredCollider2D;
        try
        {
            Vector3d currentPosition = startPosition;
            FixedQuaternion currentRotation = startRotation;
            Fixed64 remainingTime = initialRemainingTime;
            Fixed64 elapsedTime = initialElapsedTime;
            Vector3d motionSegmentStartPosition = startPosition;
            FixedQuaternion motionSegmentStartRotation = startRotation;
            Fixed64 motionSegmentStartElapsedTime = initialElapsedTime;
            Vector3d motionSegmentLinearVelocity = ProjectLinearMotion(_linearVelocity);
            Vector3d motionSegmentAngularVelocity = _angularVelocity;
            LSCollider? ignoredTarget = _continuousCollisionHandoffIgnoredCollider3D;
            int maxIterations = Context.Settings.ContinuousCollisionMaxToiIterations;
            int conservativeRefinementCount = 0;
            int arbiterIterationLimit = maxIterations
                + ContinuousCollisionMath.RotationalIntervalMaxDepth;
            for (int iteration = 0; iteration < arbiterIterationLimit; iteration++)
            {
                Fixed64 segmentEndElapsedTime = elapsedTime + remainingTime;
                Fixed64 segmentElapsedTime = segmentEndElapsedTime - motionSegmentStartElapsedTime;
                Vector3d segmentEnd = motionSegmentStartPosition
                    + motionSegmentLinearVelocity * segmentElapsedTime;
                Vector3d displacement = segmentEnd - currentPosition;
                FixedQuaternion targetRotation = IntegrateAngularRotation(
                    motionSegmentStartRotation,
                    motionSegmentAngularVelocity,
                    segmentElapsedTime);
                angularDistance = motionSegmentAngularVelocity.Magnitude * remainingTime;
                GatherRotationalContinuousCollisionCandidates(
                    currentPosition,
                    segmentEnd,
                    displacement,
                    pivotRadius);
                bool foundSameDimension = TryFindEarliestRotationalContinuousCollision(
                        currentPosition,
                        displacement,
                        currentRotation,
                        targetRotation,
                        angularDistance,
                        pivotRadius,
                        elapsedTime,
                        remainingTime,
                        isKinematic: false,
                        ignoredTarget,
                        out Fixed64 safeTime,
                        out ManifoldContact contact,
                        out bool hasContact,
                        out Fixed64 contactTime,
                        out LSCollider? target);
                bool foundMixed = TryFindEarliestMixedRotationalContinuousCollision(
                    currentPosition,
                    displacement,
                    currentRotation,
                    targetRotation,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    sourceIsKinematic: false,
                    out RotationalMixed2DHit mixedHit);
                // Both searches initialize their hit structures, so evaluating
                // the tie policy eagerly avoids encoding an artificial
                // short-circuit state in this deterministic arbitration rule.
                bool useMixed = foundMixed
                    & (!foundSameDimension | mixedHit.SafeTime <= safeTime);
                if (!foundSameDimension && !foundMixed)
                {
                    currentPosition = segmentEnd;
                    currentRotation = targetRotation;
                    break;
                }

                Fixed64 conservativeTime = useMixed ? mixedHit.SafeTime : safeTime;
                Fixed64 witnessedTime = useMixed ? mixedHit.ContactTime : contactTime;
                bool responseWitnessIsEarliest = witnessedTime == conservativeTime;
                Fixed64 witnessedConsumedTime = remainingTime * witnessedTime;
                bool appliedResponse = responseWitnessIsEarliest && (useMixed
                    ? mixedHit.HasContact
                        && TryApplyMixedRotationalContinuousCollisionResponse(
                            mixedHit.Target,
                            mixedHit.Contact,
                            mixedHit.ContactTime,
                            currentPosition,
                            displacement,
                            currentRotation,
                            targetRotation,
                            elapsedTime,
                            remainingTime,
                            sourceIsKinematic: false)
                    : hasContact
                        && target!.Body is SolidBody targetBody
                        && IsMovingRotationalContinuousCollisionTarget(targetBody)
                        && TryApplyRotationalContinuousCollisionResponse(
                            targetBody,
                            contact,
                            contactTime,
                            currentPosition,
                            displacement,
                            currentRotation,
                            elapsedTime,
                            remainingTime,
                            sourceIsKinematic: false));
                bool deferUnresolvedWitness = !responseWitnessIsEarliest
                    && witnessedTime > conservativeTime
                    && conservativeTime > Fixed64.Epsilon
                    && conservativeRefinementCount
                        < ContinuousCollisionMath.RotationalIntervalMaxDepth;
                Fixed64 consumedTime = appliedResponse
                    ? witnessedConsumedTime
                    : remainingTime * conservativeTime;
                Fixed64 impactElapsedTime = elapsedTime + consumedTime;
                Fixed64 impactSegmentElapsedTime = impactElapsedTime
                    - motionSegmentStartElapsedTime;
                Vector3d impactPosition = motionSegmentStartPosition
                    + motionSegmentLinearVelocity * impactSegmentElapsedTime;
                FixedQuaternion impactRotation = IntegrateAngularRotation(
                    motionSegmentStartRotation,
                    motionSegmentAngularVelocity,
                    impactSegmentElapsedTime);
                if (appliedResponse)
                {
                    elapsedTime = impactElapsedTime;
                    remainingTime -= consumedTime;
                    currentPosition = impactPosition;
                    currentRotation = impactRotation;
                    motionSegmentStartPosition = currentPosition;
                    motionSegmentStartRotation = currentRotation;
                    motionSegmentStartElapsedTime = elapsedTime;
                    motionSegmentLinearVelocity = ProjectLinearMotion(_linearVelocity);
                    motionSegmentAngularVelocity = _angularVelocity;
                    LastContinuousCollisionToiIterationCount++;
                    if (useMixed)
                    {
                        _continuousCollisionHandoffIgnoredCollider2D = mixedHit.Target;
                        ignoredTarget = null;
                    }
                    else
                    {
                        ignoredTarget = target;
                        _continuousCollisionHandoffIgnoredCollider2D = null;
                    }
                    Fixed64 remainingMotionSquared = FixedMath.Max(
                        _linearVelocity.MagnitudeSquared,
                        _angularVelocity.MagnitudeSquared);
                    if (remainingTime <= Fixed64.Epsilon
                        || remainingMotionSquared <= Fixed64.Epsilon)
                    {
                        break;
                    }

                    if (LastContinuousCollisionToiIterationCount >= maxIterations)
                    {
                        LastContinuousCollisionToiIterationLimitReached = true;
                        break;
                    }

                    continue;
                }

                currentPosition = impactPosition;
                currentRotation = impactRotation;
                if (deferUnresolvedWitness)
                {
                    conservativeRefinementCount++;
                    elapsedTime = impactElapsedTime;
                    remainingTime -= consumedTime;
                    continue;
                }

                LastContinuousCollisionToiIterationCount++;
                StopRotationalContinuousCollision(
                    useMixed
                        ? -mixedHit.Contact.Normal3DTo2D
                        : ResolveSourceContactNormal(target!, contact));
                if (CanAppendContinuousCollisionSegment(impactElapsedTime))
                {
                    _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this);
                    AppendContinuousCollisionSegment(
                        currentPosition,
                        currentRotation,
                        Vector3d.Zero,
                        Vector3d.Zero,
                        impactElapsedTime);
                    Context.Physics.RefreshContinuousCollisionCandidate(this);
                }
                else
                {
                    LastContinuousCollisionToiIterationLimitReached = true;
                }
                break;
            }

            proposedPosition = currentPosition;
            proposedRotation = currentRotation;
            return true;
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
            _continuousCollisionHandoffIgnoredCollider2D = originalIgnoredMixedTarget;
        }
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 pivotRadius = ResolveContinuousCollisionProxyRadius();
        Vector3d displacement = proposedPosition - startPosition;
        Fixed64 angularDistance = ResolveKinematicAngularDistanceRadians(startRotation, proposedRotation);
        bool targetRequiresRotationalSampling = angularDistance <= Fixed64.Epsilon
            && HasNearbyRotationalContinuousCollisionTarget(
                startPosition,
                displacement,
                pivotRadius);
        if (angularDistance <= Fixed64.Epsilon && !targetRequiresRotationalSampling)
            return false;

        Fixed64 angularArcLength = angularDistance * pivotRadius;
        if (pivotRadius <= Fixed64.Epsilon
            || (!targetRequiresRotationalSampling
                && angularArcLength <= Fixed64.Epsilon)
            || (!targetRequiresRotationalSampling
                && mode == ContinuousCollisionMode.Auto
                && angularArcLength <= pivotRadius
                && ContinuousCollisionMath.IsWithinProxyRadius(
                    displacement,
                    displacement.MagnitudeSquared,
                    pivotRadius)))
        {
            return false;
        }

        FixedQuaternion targetRotation = proposedRotation;
        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        LSCollider2D? originalIgnoredMixedTarget = _continuousCollisionHandoffIgnoredCollider2D;
        try
        {
            Vector3d currentPosition = startPosition;
            FixedQuaternion currentRotation = startRotation;
            Fixed64 elapsedTime = Fixed64.Zero;
            Fixed64 remainingTime = Context.DeltaTime;
            LSCollider? ignoredTarget = null;
            int maxIterations = Context.Settings.ContinuousCollisionMaxToiIterations;
            int conservativeRefinementCount = 0;
            int arbiterIterationLimit = maxIterations
                + ContinuousCollisionMath.RotationalIntervalMaxDepth;
            for (int iteration = 0; iteration < arbiterIterationLimit; iteration++)
            {
                Vector3d segmentDisplacement = (proposedPosition - currentPosition);
                FixedQuaternion segmentTargetRotation = targetRotation;
                angularDistance = ResolveKinematicAngularDistanceRadians(
                    currentRotation,
                    segmentTargetRotation);
                GatherRotationalContinuousCollisionCandidates(
                    currentPosition,
                    proposedPosition,
                    segmentDisplacement,
                    pivotRadius);
                bool foundSameDimension = TryFindEarliestRotationalContinuousCollision(
                        currentPosition,
                        segmentDisplacement,
                        currentRotation,
                        segmentTargetRotation,
                        angularDistance,
                        pivotRadius,
                        elapsedTime,
                        remainingTime,
                        isKinematic: true,
                        ignoredTarget,
                        out Fixed64 safeTime,
                        out ManifoldContact contact,
                        out bool hasContact,
                        out Fixed64 contactTime,
                        out LSCollider? target);
                bool foundMixed = TryFindEarliestMixedRotationalContinuousCollision(
                    currentPosition,
                    segmentDisplacement,
                    currentRotation,
                    segmentTargetRotation,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    sourceIsKinematic: true,
                    out RotationalMixed2DHit mixedHit);
                bool useMixed = foundMixed
                    && (!foundSameDimension || mixedHit.SafeTime <= safeTime);
                if (!foundSameDimension && !foundMixed)
                {
                    currentPosition = proposedPosition;
                    currentRotation = targetRotation;
                    break;
                }

                Fixed64 conservativeTime = useMixed ? mixedHit.SafeTime : safeTime;
                Fixed64 witnessedTime = useMixed ? mixedHit.ContactTime : contactTime;
                bool responseWitnessIsEarliest = witnessedTime == conservativeTime;
                bool appliedResponse = responseWitnessIsEarliest && (useMixed
                    ? mixedHit.HasContact
                        && TryApplyMixedRotationalContinuousCollisionResponse(
                            mixedHit.Target,
                            mixedHit.Contact,
                            mixedHit.ContactTime,
                            currentPosition,
                            segmentDisplacement,
                            currentRotation,
                            segmentTargetRotation,
                            elapsedTime,
                            remainingTime,
                            sourceIsKinematic: true)
                    : hasContact
                        && target!.Body is SolidBody targetBody
                        && !targetBody.IsKinematic
                        && IsMovingRotationalContinuousCollisionTarget(targetBody)
                        && TryApplyRotationalContinuousCollisionResponse(
                            targetBody,
                            contact,
                            contactTime,
                            currentPosition,
                            segmentDisplacement,
                            currentRotation,
                            elapsedTime,
                            remainingTime,
                            sourceIsKinematic: true));
                bool deferUnresolvedWitness = !responseWitnessIsEarliest
                    && witnessedTime > conservativeTime
                    && conservativeTime > Fixed64.Epsilon
                    && conservativeRefinementCount
                        < ContinuousCollisionMath.RotationalIntervalMaxDepth;
                Fixed64 eventTime = appliedResponse ? witnessedTime : conservativeTime;
                if (appliedResponse)
                {
                    Fixed64 consumedTime = remainingTime * eventTime;
                    elapsedTime += consumedTime;
                    remainingTime -= consumedTime;
                    currentPosition += segmentDisplacement * eventTime;
                    currentRotation = FixedQuaternion.Slerp(
                        currentRotation,
                        segmentTargetRotation,
                        eventTime).Normalized;
                    LastContinuousCollisionToiIterationCount++;
                    if (useMixed)
                    {
                        _continuousCollisionHandoffIgnoredCollider2D = mixedHit.Target;
                        ignoredTarget = null;
                    }
                    else
                    {
                        ignoredTarget = target;
                        _continuousCollisionHandoffIgnoredCollider2D = null;
                    }
                    if (remainingTime <= Fixed64.Epsilon)
                        break;
                    if (LastContinuousCollisionToiIterationCount >= maxIterations)
                    {
                        LastContinuousCollisionToiIterationLimitReached = true;
                        currentPosition = proposedPosition;
                        currentRotation = targetRotation;
                        break;
                    }

                    continue;
                }

                currentPosition += segmentDisplacement * eventTime;
                currentRotation = FixedQuaternion.Slerp(
                    currentRotation,
                    segmentTargetRotation,
                    eventTime).Normalized;
                Fixed64 impactElapsedTime = elapsedTime + remainingTime * eventTime;
                if (deferUnresolvedWitness)
                {
                    conservativeRefinementCount++;
                    Fixed64 consumedTime = remainingTime * eventTime;
                    elapsedTime = impactElapsedTime;
                    remainingTime -= consumedTime;
                    continue;
                }

                LastContinuousCollisionToiIterationCount++;
                // Kinematic sources never receive handoff trajectory segments.
                // Their prepared authored segment is therefore the only retained
                // segment before this terminal clamp, so the validated positive
                // TOI budget always admits this single replacement tail.
                _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this);
                AppendContinuousCollisionSegment(
                    currentPosition,
                    currentRotation,
                    Vector3d.Zero,
                    Vector3d.Zero,
                    impactElapsedTime);
                Context.Physics.RefreshContinuousCollisionCandidate(this);
                proposedPosition = currentPosition;
                proposedRotation = currentRotation;
                return true;
            }

            proposedPosition = currentPosition;
            proposedRotation = currentRotation;
            return true;
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
            _continuousCollisionHandoffIgnoredCollider2D = originalIgnoredMixedTarget;
        }
    }

    private bool TryFindEarliestRotationalContinuousCollisionAgainstTarget(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool isKinematic,
        out Fixed64 safeTime,
        out ManifoldContact contact,
        out bool hasContact,
        out Fixed64 contactTime)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        SolidBody? targetBody = target.Body;
        bool samplesTargetMotion = targetBody != null
            && IsMovingRotationalContinuousCollisionTarget(targetBody);
        Vector3d originalTargetPosition = targetBody?.Position3d ?? Vector3d.Zero;
        FixedQuaternion originalTargetRotation = targetBody?.Rotation ?? FixedQuaternion.Identity;
        bool originalTargetPositionMutated = targetBody?._positionMutated ?? false;
        bool originalTargetRotationMutated = targetBody?._rotationMutated ?? false;
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
        ManifoldContact knownContact = default;
        intervals[0] = new ContinuousCollisionMath.RotationalInterval(
            Fixed64.Zero,
            Fixed64.One,
            depth: 0);

        while (intervalCount > 0)
        {
            ContinuousCollisionMath.RotationalInterval interval = intervals[--intervalCount];
            if (hasKnownContact && interval.LowerTime >= knownContactTime)
                continue;

            Fixed64 midpoint = (interval.LowerTime + interval.UpperTime) * Fixed64.Half;
            Fixed64 intervalSpan = interval.UpperTime - interval.LowerTime;
            if (isKinematic)
            {
                SampleKinematicRotationalContinuousPose(
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    midpoint);
            }
            else
            {
                SampleDynamicRotationalContinuousPose(
                    startPosition,
                    displacement,
                    startRotation,
                    remainingTime,
                    midpoint);
            }

            Fixed64 midpointFrameFraction = ResolveRotationalFrameFraction(
                elapsedTime,
                remainingTime,
                midpoint);
            if (samplesTargetMotion)
            {
                targetBody!.Position3d = targetBody.SampleContinuousCollisionPosition(midpointFrameFraction);
                targetBody.Rotation = targetBody.SampleContinuousCollisionRotation(midpointFrameFraction);
                targetBody.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            }

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
            bool sampleHasContact = TrySampleRotationalContinuousCollision(
                target,
                out ManifoldContact sampleContact);
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

        return false;
        }
        finally
        {
            if (samplesTargetMotion)
            {
                targetBody!.Position3d = originalTargetPosition;
                targetBody.Rotation = originalTargetRotation;
                targetBody.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
                targetBody._positionMutated = originalTargetPositionMutated;
                targetBody._rotationMutated = originalTargetRotationMutated;
            }
        }
    }

    private bool IsRotationalIntervalSeparated(LSCollider target, Fixed64 motionBound)
    {
        if (TryGetSpherePairSeparationGap(target, out Fixed64 closestPointGap))
            return closestPointGap > motionBound;

        return ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(
            Collider.Bounds,
            target.Bounds,
            motionBound);
    }

    private bool TryGetSpherePairSeparationGap(LSCollider target, out Fixed64 separationGap)
    {
        if (Collider is LSSphereCollider sourceSphere)
            return TryGetSphereSeparationGap(sourceSphere, target, out separationGap);

        if (target is LSSphereCollider targetSphere)
            return TryGetSphereSeparationGap(targetSphere, Collider, out separationGap);

        separationGap = default;
        return false;
    }

    internal static bool TryGetSphereSeparationGap(
        LSSphereCollider sphere,
        LSCollider other,
        out Fixed64 separationGap)
    {
        Vector3d closestPoint;
        Fixed64 otherRadius;
        if (other is LSSphereCollider otherSphere)
        {
            closestPoint = otherSphere.Center;
            otherRadius = otherSphere.ScaledRadius;
        }
        else if (other is LSCuboidCollider cuboid)
        {
            closestPoint = cuboid.ClosestPointOnSurface(sphere.Center);
            otherRadius = Fixed64.Zero;
        }
        else
        {
            separationGap = default;
            return false;
        }

        if (!Vector3d.TrySubtract(sphere.Center, closestPoint, out Vector3d separation)
            || !Vector3d.TryGetMagnitude(separation, out Fixed64 distance)
            || !Fixed64.TryAdd(sphere.ScaledRadius, otherRadius, out Fixed64 combinedRadius))
        {
            separationGap = default;
            return false;
        }

        Fixed64 rawGap = distance - combinedRadius;
        if (!Fixed64.TryAdd(distance, combinedRadius, out Fixed64 characteristicScale)
            || !Fixed64.TryAdd(characteristicScale, other.ScaledRadius, out characteristicScale)
            || !ContinuousCollisionMath.TrySubtractClosestFeatureUncertainty(
                rawGap,
                characteristicScale,
                out separationGap))
        {
            separationGap = default;
            return false;
        }

        return separationGap > Fixed64.Zero;
    }

    private void SampleDynamicRotationalContinuousPose(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        Fixed64 remainingTime,
        Fixed64 sampleTime)
    {
        Position3d = startPosition + displacement * sampleTime;
        Rotation = IntegrateAngularRotation(startRotation, remainingTime * sampleTime);
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
    }

    private void SampleKinematicRotationalContinuousPose(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 sampleTime)
    {
        Position3d = startPosition + displacement * sampleTime;
        Rotation = FixedQuaternion.Slerp(startRotation, targetRotation, sampleTime).Normalized;
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
    }

    private static Fixed64 ResolveKinematicAngularDistanceRadians(
        FixedQuaternion startRotation,
        FixedQuaternion proposedRotation)
    {
        Fixed64 angleDegrees = FixedQuaternion.Angle(startRotation, proposedRotation);
        return FixedMath.DegToRad(angleDegrees.Abs());
    }

    private bool TrySampleRotationalContinuousCollision(
        LSCollider target,
        out ManifoldContact contact)
    {
        contact = default;
        OrderRotationalContinuousCollisionPair(
            target,
            out LSCollider colliderA,
            out LSCollider colliderB,
            out _);
        CollisionType collisionType = ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape);
        _rotationalContinuousCollisionManifold.BeginUpdate(Context.FrameCount);
        var workItem = new CollisionWorkItem(
            Context,
            colliderA,
            colliderB,
            collisionType,
            _rotationalContinuousCollisionManifold);
        if (!CollisionDetection.DoCollisionCheck(workItem)
            || !_rotationalContinuousCollisionManifold.HasContact)
        {
            return false;
        }

        contact = _rotationalContinuousCollisionManifold.PrimaryContact;
        return true;
    }

    private Vector3d ResolveSourceContactNormal(
        LSCollider target,
        ManifoldContact contact)
    {
        OrderRotationalContinuousCollisionPair(
            target,
            out _,
            out _,
            out bool sourceIsA);
        return sourceIsA ? contact.Normal : -contact.Normal;
    }

    private bool IsMovingRotationalContinuousCollisionTarget(SolidBody target) =>
        target != this
        && (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
            HasContinuousCollisionRotationalMotion))
        && target.Active
        && !target.Collider.IsTrigger
        && (target.IsKinematic || !target.IsPositionFullyFrozen)
        && target.Collider != _continuousCollisionHandoffIgnoredCollider3D;

    private Fixed64 ResolveRotationalFrameFraction(
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        Fixed64 localFraction) =>
        FixedMath.Clamp01(
            (elapsedTime + remainingTime * localFraction) / Context.DeltaTime);

    private void OrderRotationalContinuousCollisionPair(
        LSCollider target,
        out LSCollider colliderA,
        out LSCollider colliderB,
        out bool sourceIsA)
    {
        if (Collider.Priority >= target.Priority)
        {
            colliderA = Collider;
            colliderB = target;
            sourceIsA = true;
            return;
        }

        colliderA = target;
        colliderB = Collider;
        sourceIsA = false;
    }

    private FixedQuaternion IntegrateAngularRotation(FixedQuaternion startRotation, Fixed64 deltaTime)
        => IntegrateAngularRotation(startRotation, _angularVelocity, deltaTime);

    private static FixedQuaternion IntegrateAngularRotation(
        FixedQuaternion startRotation,
        Vector3d angularVelocity,
        Fixed64 deltaTime)
    {
        FixedQuaternion angularVelocityQuaternion = new(
            angularVelocity.X,
            angularVelocity.Y,
            angularVelocity.Z,
            Fixed64.Zero);
        FixedQuaternion spin = angularVelocityQuaternion * startRotation * Fixed64.Half * deltaTime;
        return (startRotation + spin).Normalized;
    }

}
