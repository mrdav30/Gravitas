//=======================================================================
// SolidBody2D.ContinuousCollision.Dynamic.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    internal bool ApplyContinuousCollisionHandoff(
        Vector2d positionAtImpact,
        Vector2d velocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        Fixed64 elapsedTime = Context.DeltaTime - remainingTime;
        Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(elapsedTime);
        Fixed64 rotationAtImpact = _continuousCollisionTrajectory.Count == 0
            ? _rotation
            : SampleContinuousCollisionRotation(frameFraction);
        return ApplyContinuousCollisionHandoff(
            positionAtImpact,
            rotationAtImpact,
            velocityDelta,
            Fixed64.Zero,
            remainingTime,
            ignoredCollider3D,
            ignoredCollider2D);
    }

    internal bool ApplyContinuousCollisionHandoff(
        Vector2d positionAtImpact,
        Fixed64 rotationAtImpact,
        Vector2d linearVelocityDelta,
        Fixed64 angularVelocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        Vector2d projectedLinearDelta = ProjectLinearMotion(linearVelocityDelta);
        bool linearVelocityResolved = Vector2d.TryAdd(
            _linearVelocity,
            projectedLinearDelta,
            out Vector2d postLinearVelocity);
        bool angularVelocityResolved = Fixed64.TryAdd(
            _angularVelocity,
            angularVelocityDelta,
            out Fixed64 postAngularVelocity);
        if (!(linearVelocityResolved & angularVelocityResolved))
        {
            return false;
        }

        return ApplyContinuousCollisionHandoffState(
            positionAtImpact,
            rotationAtImpact,
            postLinearVelocity,
            postAngularVelocity,
            remainingTime,
            ignoredCollider3D,
            ignoredCollider2D);
    }

    internal bool ApplyContinuousCollisionHandoffState(
        Vector2d positionAtImpact,
        Fixed64 rotationAtImpact,
        Vector2d postLinearVelocity,
        Fixed64 postAngularVelocity,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!CanApplyContinuousCollisionHandoffState(
                positionAtImpact,
                rotationAtImpact,
                remainingTime,
                out Vector2d resolvedPosition))
        {
            return false;
        }

        _ = Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this);

        ApplyContinuousCollisionHandoffStateReserved(
            resolvedPosition,
            rotationAtImpact,
            postLinearVelocity,
            postAngularVelocity,
            remainingTime,
            ignoredCollider3D,
            ignoredCollider2D);
        return true;
    }

    internal void ApplyContinuousCollisionHandoffStateReserved(
        Vector2d resolvedPosition,
        Fixed64 rotationAtImpact,
        Vector2d postLinearVelocity,
        Fixed64 postAngularVelocity,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        _position = resolvedPosition;
        _rotation = CanonicalizeRotation(rotationAtImpact);
        Collider.PublishPreparedBodyPose();
        postLinearVelocity = ProjectLinearMotion(postLinearVelocity);
        postAngularVelocity = CanRotate ? postAngularVelocity : Fixed64.Zero;
        if (_linearVelocity != postLinearVelocity || _angularVelocity != postAngularVelocity)
            WakeFromCollision();
        _linearVelocity = postLinearVelocity;
        _angularVelocity = postAngularVelocity;
        RefreshLinearSpeed();
        RefreshAngularSpeed();
        bool hasRemainingMotion = remainingTime > Fixed64.Epsilon
            && (_linearVelocity.MagnitudeSquared > Fixed64.Epsilon
                || _angularVelocity.Abs() > Fixed64.Epsilon);
        AppendContinuousCollisionFrameSegment(
            resolvedPosition,
            _rotation,
            hasRemainingMotion ? _linearVelocity : Vector2d.Zero,
            hasRemainingMotion ? _angularVelocity : Fixed64.Zero,
            Context.DeltaTime - remainingTime);
        Context.Physics2D.RefreshContinuousCollisionCandidate(this);
        if (!hasRemainingMotion)
        {
            DiscardContinuousCollisionHandoff();
            return;
        }

        _continuousCollisionHandoffToken = Context.LateSimulateToken;
        _continuousCollisionHandoffRemainingTime = remainingTime;
        _continuousCollisionHandoffIgnoredCollider3D = ignoredCollider3D;
        _continuousCollisionHandoffIgnoredCollider2D = ignoredCollider2D;
        _continuousCollisionHandoffPending = true;
        Context.Physics2D.QueueContinuousCollisionHandoff(this);
    }

    internal bool CanApplyContinuousCollisionHandoffState(
        Vector2d positionAtImpact,
        Fixed64 rotationAtImpact,
        Fixed64 remainingTime,
        out Vector2d resolvedPosition)
    {
        Fixed64 elapsedTime = Context.DeltaTime - remainingTime;
        bool hasMobility = CanTranslate | CanRotate;
        bool hasTrajectoryCapacity = CanAppendContinuousCollisionFrameSegment(elapsedTime);
        bool positionDeltaResolved = Vector2d.TrySubtract(
            positionAtImpact,
            _position,
            out Vector2d positionDelta);
        bool positionResolved = Vector2d.TryAdd(
            _position,
            ProjectLinearMotion(positionDelta),
            out resolvedPosition);
        if (!(hasMobility
                & hasTrajectoryCapacity
                & positionDeltaResolved
                & positionResolved))
        {
            return false;
        }

        return Collider.TryPrepareBodyPose(
            resolvedPosition,
            CanonicalizeRotation(rotationAtImpact));
    }

    internal bool TryConsumeContinuousCollisionHandoff(bool updateSleepState, bool updateColliderState)
    {
        if (!_continuousCollisionHandoffPending || _continuousCollisionHandoffToken != Context.LateSimulateToken)
            return false;

        Fixed64 remainingTime = _continuousCollisionHandoffRemainingTime;
        _continuousCollisionHandoffPending = false;
        _continuousCollisionHandoffRemainingTime = Fixed64.Zero;
        bool hasLinearMotion = CanTranslate
            && _linearVelocity.MagnitudeSquared > Fixed64.Epsilon;
        bool hasAngularMotion = CanRotate
            && _angularVelocity.Abs() > Fixed64.Epsilon;
        if (!hasLinearMotion && !hasAngularMotion)
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
            if (updateColliderState)
                Collider.Rebuild();
            return true;
        }

        Vector2d startPosition = _position;
        Vector2d proposedPosition = hasLinearMotion
            ? startPosition + _linearVelocity * remainingTime
            : startPosition;
        Fixed64 startRotation = _rotation;
        Fixed64 proposedRotation = hasAngularMotion
            ? startRotation + _angularVelocity * remainingTime
            : startRotation;
        Fixed64 elapsedTime = FixedMath.Max(Fixed64.Zero, Context.DeltaTime - remainingTime);
        try
        {
            if (ShouldUseRotationalContinuousCollisionArbiter(
                    startPosition,
                    proposedPosition,
                    startRotation,
                    proposedRotation,
                    forceContinuous: true))
            {
                TryResolveRotationalContinuousCollision(
                    startPosition,
                    ref proposedPosition,
                    startRotation,
                    ref proposedRotation,
                    remainingTime,
                    elapsedTime,
                    forceContinuous: true);
            }
            else if (hasLinearMotion)
            {
                TryResolveContinuousCollision(
                    startPosition,
                    ref proposedPosition,
                    remainingTime,
                    elapsedTime,
                    forceContinuous: true);
            }
        }
        finally
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
        }
        _position = proposedPosition;
        _rotation = CanonicalizeRotation(proposedRotation);
        if (updateColliderState)
            Collider.Rebuild();
        else
            Collider.RebuildRuntimeShapeOnly();

        if (updateSleepState)
            UpdateSleepState();

        return true;
    }

    internal void DiscardContinuousCollisionHandoff()
    {
        _continuousCollisionHandoffPending = false;
        _continuousCollisionHandoffRemainingTime = Fixed64.Zero;
        _continuousCollisionHandoffIgnoredCollider3D = null;
        _continuousCollisionHandoffIgnoredCollider2D = null;
    }

    private bool TryResolveContinuousCollision(Vector2d startPosition, ref Vector2d proposedPosition) =>
        TryResolveContinuousCollision(
            startPosition,
            ref proposedPosition,
            Context.DeltaTime,
            Fixed64.Zero,
            forceContinuous: false);

    private bool TryResolveContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 initialRemainingTime,
        Fixed64 initialElapsedTime,
        bool forceContinuous)
    {
        LastContinuousCollisionToiIterationCount = 0;
        LastContinuousCollisionToiIterationLimitReached = false;

        ContinuousCollisionMode mode = ContinuousCollisionMode.Continuous;
        if (!forceContinuous && !ShouldUseContinuousCollision(out mode))
            return false;

        Vector2d requestedDisplacement = _linearVelocity * initialRemainingTime;
        Vector2d displacement = ContinuousCollisionSweepRange.ValidateEndpoint(
            startPosition,
            proposedPosition,
            requestedDisplacement,
            out _);

        Fixed64 displacementMagnitudeSquared = displacement.MagnitudeSquared;
        if (displacementMagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto
                && ContinuousCollisionMath.IsWithinProxyRadius(displacement, displacementMagnitudeSquared, proxyRadius)))
        {
            return false;
        }

        bool resolved = false;
        int conservativeMixedRefinementCount = 0;
        Vector2d currentPosition = startPosition;
        Fixed64 remainingTime = initialRemainingTime;
        Fixed64 elapsedTime = initialElapsedTime;
        int maxToiIterations = Context.Settings.ContinuousCollisionMaxToiIterations;
        LSCollider2D? originalIgnoredCollider2D = _continuousCollisionHandoffIgnoredCollider2D;
        LSCollider? originalIgnoredCollider3D = _continuousCollisionHandoffIgnoredCollider3D;
        try
        {
            while (LastContinuousCollisionToiIterationCount < maxToiIterations)
            {
                Vector2d requestedSegmentDisplacement = _linearVelocity * remainingTime;
                Vector2d requestedSegmentEnd = currentPosition + requestedSegmentDisplacement;
                Vector2d segmentDisplacement = ContinuousCollisionSweepRange.ValidateEndpoint(
                    currentPosition,
                    requestedSegmentEnd,
                    requestedSegmentDisplacement,
                    out Fixed64 segmentLength);
                Vector2d segmentEnd = requestedSegmentEnd;

                Fixed64 elapsedFraction = elapsedTime / Context.DeltaTime;
                if (!TryGetFirstContinuousCollisionHit(
                        currentPosition,
                        segmentEnd,
                        proxyRadius,
                        elapsedFraction,
                        out Vector2d hitNormal,
                        out Fixed64 hitDistance,
                        out ContinuousCollisionTargetKind targetKind,
                        out LSCollider2D? target2D,
                        out LSCollider? target3D))
                {
                    currentPosition = segmentEnd;
                    break;
                }

                Fixed64 hitTime = FixedMath.Clamp01(hitDistance / segmentLength);
                currentPosition += segmentDisplacement.Normalized * hitDistance;
                Vector2d previousVelocity = _linearVelocity;
                Fixed64 consumedTime = remainingTime * hitTime;
                Fixed64 remainingAfterHit = remainingTime - consumedTime;
                Fixed64 hitElapsedTime = elapsedTime + consumedTime;
                if (targetKind
                    == ContinuousCollisionTargetKind.UnresolvedMixed)
                {
                    resolved = true;
                    if (hitTime > Fixed64.Epsilon
                        && conservativeMixedRefinementCount
                            < ContinuousCollisionMath.RotationalIntervalMaxDepth)
                    {
                        conservativeMixedRefinementCount++;
                        remainingTime = remainingAfterHit;
                        elapsedTime = hitElapsedTime;
                        if (remainingTime > Fixed64.Epsilon)
                            continue;
                    }

                    LastContinuousCollisionToiIterationCount++;
                    LastContinuousCollisionToiIterationLimitReached =
                        remainingAfterHit > Fixed64.Epsilon;
                    _ = Context.Physics2D
                        .TryReserveContinuousCollisionCandidateRefresh(this);
                    UpdateContinuousCollisionFrameTrajectory(
                        currentPosition,
                        Vector2d.Zero,
                        hitElapsedTime);
                    Context.Physics2D
                        .RefreshContinuousCollisionCandidate(this);

                    if (LastContinuousCollisionToiIterationLimitReached)
                        Context.Physics2D.ReportContinuousCollisionIterationLimit();

                    break;
                }

                if (!CanAppendContinuousCollisionFrameSegment(hitElapsedTime))
                {
                    ContinuousCollisionMotionSegment2D activeSegment =
                        ResolveContinuousCollisionSegment(
                            ResolveContinuousCollisionFrameFraction(hitElapsedTime));
                    currentPosition = activeSegment.StartPosition;
                    elapsedTime = activeSegment.StartFraction * Context.DeltaTime;
                    RemoveClosingContinuousCollisionVelocity(hitNormal);
                    UpdateContinuousCollisionFrameTrajectory(
                        currentPosition,
                        _linearVelocity,
                        elapsedTime);
                    Context.Physics2D.RefreshContinuousCollisionCandidate(this);
                    LastContinuousCollisionToiIterationCount++;
                    LastContinuousCollisionToiIterationLimitReached = true;
                    Context.Physics2D.ReportContinuousCollisionIterationLimit();
                    resolved = true;
                    break;
                }

                bool appliedResponse = TryApplyContinuousCollisionDynamicResponse(
                        hitNormal,
                        targetKind,
                        target2D,
                        target3D,
                        currentPosition,
                        hitElapsedTime,
                        remainingAfterHit);
                if (!appliedResponse)
                {
                    RemoveClosingContinuousCollisionVelocity(hitNormal);
                    UpdateContinuousCollisionFrameTrajectory(currentPosition, _linearVelocity, hitElapsedTime);
                }
                else if (targetKind == ContinuousCollisionTargetKind.Dynamic2D)
                {
                    _continuousCollisionHandoffIgnoredCollider2D = target2D;
                    _continuousCollisionHandoffIgnoredCollider3D = null;
                }
                else
                {
                    _continuousCollisionHandoffIgnoredCollider2D = null;
                    _continuousCollisionHandoffIgnoredCollider3D = target3D;
                }

                LastContinuousCollisionToiIterationCount++;
                resolved = true;

                remainingTime = remainingAfterHit;
                elapsedTime = hitElapsedTime;
                if (remainingTime <= Fixed64.Epsilon || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
                    break;

                if (LastContinuousCollisionToiIterationCount >= maxToiIterations)
                {
                    LastContinuousCollisionToiIterationLimitReached = true;
                    break;
                }

                if (hitTime <= Fixed64.Epsilon && previousVelocity == _linearVelocity)
                    break;
            }

            proposedPosition = currentPosition;
            return resolved;
        }
        finally
        {
            _continuousCollisionHandoffIgnoredCollider2D = originalIgnoredCollider2D;
            _continuousCollisionHandoffIgnoredCollider3D = originalIgnoredCollider3D;
        }
    }

    private bool TryApplyContinuousCollisionDynamicResponse(
        Vector2d normalForSource,
        ContinuousCollisionTargetKind targetKind,
        LSCollider2D? target2D,
        LSCollider? target3D,
        Vector2d sourcePositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        if (targetKind == ContinuousCollisionTargetKind.Dynamic2D)
        {
            SolidBody2D targetBody = target2D!.Body!;
            Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
            Vector2d targetPositionAtImpact = targetBody.SampleContinuousCollisionPosition(
                frameFraction);
            return TryApplyContinuousCollisionDynamicResponse(
                targetBody,
                normalForSource,
                sourcePositionAtImpact,
                targetPositionAtImpact,
                hitElapsedTime,
                remainingTime);
        }

        if (targetKind == ContinuousCollisionTargetKind.Dynamic3D)
        {
            SolidBody targetBody = target3D!.Body!;
            Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
            Vector3d targetPositionAtImpact = targetBody.SampleContinuousCollisionPosition(
                frameFraction);
            return TryApplyContinuousCollisionMixed3DResponse(
                targetBody,
                normalForSource,
                sourcePositionAtImpact,
                targetPositionAtImpact,
                hitElapsedTime,
                remainingTime);
        }

        return false;
    }

    internal bool TryApplyContinuousCollisionDynamicResponse(
        SolidBody2D target,
        Vector2d normalForSource,
        Vector2d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        if (!ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(normalForSource, out Vector2d normal))
            return false;

        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(normal);
        Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
        Vector2d targetLinearVelocity = target.SampleContinuousCollisionLinearVelocity(frameFraction);
        bool inverseMassResolved = Fixed64.TryAdd(
            constrainedInverseMassA,
            constrainedInverseMassB,
            out Fixed64 inverseMass);
        bool relativeVelocityResolved = Vector2d.TrySubtract(
            _linearVelocity,
            targetLinearVelocity,
            out Vector2d relativeVelocity);
        if (!(inverseMassResolved & relativeVelocityResolved))
        {
            return false;
        }

        Fixed64 normalVelocity = Vector2d.Dot(relativeVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        bool responseFactorResolved = Fixed64.TryAdd(
            Fixed64.One,
            restitution,
            out Fixed64 responseFactor);
        bool closingSpeedResolved = Fixed64.TrySubtract(
            Fixed64.Zero,
            normalVelocity,
            out Fixed64 closingSpeed);
        bool responseSpeedResolved = Fixed64.TryMultiplyDivide(
            closingSpeed,
            responseFactor,
            Fixed64.One,
            out Fixed64 responseSpeed);
        if (!(responseFactorResolved & closingSpeedResolved & responseSpeedResolved))
        {
            return false;
        }

        Vector2d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector2d targetResponseNormal = target.ProjectLinearMotion(-normal);
        bool sourceDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            sourceResponseNormal,
            responseSpeed,
            EffectiveInverseMass,
            inverseMass,
            out Vector2d sourceVelocityDelta);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            target.EffectiveInverseMass,
            inverseMass,
            out Vector2d targetVelocityDelta);
        if (!(sourceDeltaResolved & targetDeltaResolved))
        {
            return false;
        }

        bool sourceVelocityResolved = Vector2d.TryAdd(
            _linearVelocity,
            sourceVelocityDelta,
            out Vector2d sourcePostLinearVelocity);
        bool sourceTrajectoryAvailable = CanAppendContinuousCollisionFrameSegment(hitElapsedTime);
        if (!(sourceVelocityResolved & sourceTrajectoryAvailable))
        {
            return false;
        }

        Vector2d targetResolvedPosition = default;
        Vector2d targetPostLinearVelocity = targetLinearVelocity;
        Fixed64 targetRotationAtImpact = target.SampleContinuousCollisionRotation(frameFraction);
        Fixed64 targetAngularVelocity = target.SampleContinuousCollisionAngularVelocity(frameFraction);
        bool targetStateAvailable = true;
        if (target.CanTranslate)
        {
            bool targetVelocityResolved = Vector2d.TryAdd(
                targetLinearVelocity,
                targetVelocityDelta,
                out targetPostLinearVelocity);
            bool targetTrajectoryAvailable = target.CanApplyContinuousCollisionHandoffState(
                targetPositionAtImpact,
                targetRotationAtImpact,
                remainingTime,
                out targetResolvedPosition);
            targetStateAvailable = targetVelocityResolved & targetTrajectoryAvailable;
        }

        if (!targetStateAvailable)
        {
            return false;
        }

        _ = target.CanTranslate
            ? Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this, target)
            : Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this);

        ApplyContinuousCollisionSourceResponse(
            sourcePositionAtImpact,
            sourcePostLinearVelocity,
            hitElapsedTime);
        if (target.CanTranslate)
        {
            target.ApplyContinuousCollisionHandoffStateReserved(
                targetResolvedPosition,
                targetRotationAtImpact,
                targetPostLinearVelocity,
                targetAngularVelocity,
                remainingTime,
                ignoredCollider2D: Collider);
        }

        return true;
    }

    internal bool TryApplyContinuousCollisionMixed3DResponse(
        SolidBody target,
        Vector2d normalForSource,
        Vector2d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        bool normalResolved = ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            normalForSource,
            out Vector2d normal);
        Vector3d normal3D = normal.ToVector3d(Fixed64.Zero);
        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(normal3D);
        Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
        Vector3d targetVelocity = target.SampleContinuousCollisionLinearVelocity(
            frameFraction);
        bool inverseMassResolved = Fixed64.TryAdd(
            constrainedInverseMassA,
            constrainedInverseMassB,
            out Fixed64 inverseMass);
        bool relativeVelocityResolved = Vector3d.TrySubtract(
            _linearVelocity.ToVector3d(Fixed64.Zero),
            targetVelocity,
            out Vector3d relativeVelocity);
        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal3D);
        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        bool responseFactorResolved = Fixed64.TryAdd(
            Fixed64.One,
            restitution,
            out Fixed64 responseFactor);
        bool closingSpeedResolved = Fixed64.TrySubtract(
            Fixed64.Zero,
            normalVelocity,
            out Fixed64 closingSpeed);
        bool responseSpeedResolved = Fixed64.TryMultiplyDivide(
            closingSpeed,
            responseFactor,
            Fixed64.One,
            out Fixed64 responseSpeed);
        Vector2d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector3d targetResponseNormal = target.ProjectLinearMotion(-normal3D);
        bool sourceDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            sourceResponseNormal,
            responseSpeed,
            EffectiveInverseMass,
            inverseMass,
            out Vector2d sourceVelocityDelta);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            target.EffectiveInverseMass,
            inverseMass,
            out Vector3d targetVelocityDelta);
        bool sourceVelocityResolved = Vector2d.TryAdd(
            _linearVelocity,
            sourceVelocityDelta,
            out Vector2d sourcePostLinearVelocity);
        bool sourceTrajectoryAvailable = CanAppendContinuousCollisionFrameSegment(hitElapsedTime);
        Vector3d targetResolvedPosition = default;
        Vector3d targetPostLinearVelocity = targetVelocity;
        FixedQuaternion targetRotationAtImpact = target.SampleContinuousCollisionRotation(frameFraction);
        Vector3d targetAngularVelocity = target.SampleContinuousCollisionAngularVelocity(frameFraction);
        bool targetStateAvailable = true;
        if (target.CanTranslate)
        {
            bool targetVelocityResolved = Vector3d.TryAdd(
                targetVelocity,
                targetVelocityDelta,
                out targetPostLinearVelocity);
            bool targetTrajectoryAvailable = target.CanApplyContinuousCollisionHandoff(
                targetPositionAtImpact,
                targetRotationAtImpact,
                remainingTime,
                out targetResolvedPosition);
            targetStateAvailable = targetVelocityResolved & targetTrajectoryAvailable;
        }

        bool responseAdmissible = normalResolved
            & inverseMassResolved
            & relativeVelocityResolved
            & normalVelocity < -Fixed64.Epsilon
            & responseFactorResolved
            & closingSpeedResolved
            & responseSpeedResolved
            & sourceDeltaResolved
            & targetDeltaResolved
            & sourceVelocityResolved
            & sourceTrajectoryAvailable
            & targetStateAvailable;
        if (!responseAdmissible)
        {
            return false;
        }

        _ = Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this);
        if (target.CanTranslate)
            _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(target);

        ApplyContinuousCollisionSourceResponse(
            sourcePositionAtImpact,
            sourcePostLinearVelocity,
            hitElapsedTime);
        if (target.CanTranslate)
        {
            target.ApplyContinuousCollisionHandoffReserved(
                targetResolvedPosition,
                targetRotationAtImpact,
                targetPostLinearVelocity,
                targetAngularVelocity,
                remainingTime,
                ignoredCollider2D: Collider);
        }

        return true;
    }

    private void ApplyContinuousCollisionSourceResponse(
        Vector2d positionAtImpact,
        Vector2d postLinearVelocity,
        Fixed64 elapsedTime)
    {
        postLinearVelocity = ProjectLinearMotion(postLinearVelocity);
        WakeFromCollision();
        _linearVelocity = postLinearVelocity;
        RefreshLinearSpeed();
        UpdateContinuousCollisionFrameTrajectory(positionAtImpact, _linearVelocity, elapsedTime);
        Context.Physics2D.RefreshContinuousCollisionCandidate(this);
    }

    private void UpdateContinuousCollisionFrameTrajectory(
        Vector2d positionAtElapsedTime,
        Vector2d velocity,
        Fixed64 elapsedTime)
    {
        AppendContinuousCollisionFrameSegment(
            positionAtElapsedTime,
            velocity,
            elapsedTime);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveContinuousCollisionFrameFraction(Fixed64 hitElapsedTime) =>
        FixedMath.Clamp01(hitElapsedTime / Context.DeltaTime);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveContinuousCollisionRestitution(SolidBody2D target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= Context.Settings.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return PhysicsMaterial.CombineRestitution(Collider.Material, target.Collider.Material);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveContinuousCollisionRestitution(SolidBody target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= Context.Settings.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return PhysicsMaterial.CombineRestitution(Collider.Material, target.Collider.Material);
    }

    private void RemoveClosingContinuousCollisionVelocity(Vector2d normal)
    {
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return;

        Fixed64 closingSpeed = Vector2d.Dot(_linearVelocity, normal);
        if (closingSpeed >= Fixed64.Zero)
            return;

        _linearVelocity -= normal * closingSpeed;
        RefreshLinearSpeed();
    }

}
