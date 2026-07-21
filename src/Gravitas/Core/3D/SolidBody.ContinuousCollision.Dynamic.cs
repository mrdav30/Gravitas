//=======================================================================
// SolidBody.ContinuousCollision.Dynamic.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;

namespace Gravitas;

public partial class SolidBody
{
    internal bool ApplyContinuousCollisionHandoff(
        Vector3d positionAtImpact,
        Vector3d velocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!Vector3d.TryAdd(_linearVelocity, ProjectLinearMotion(velocityDelta), out Vector3d postLinearVelocity))
            return false;

        return ApplyContinuousCollisionHandoff(
            positionAtImpact,
            Rotation,
            postLinearVelocity,
            _angularVelocity,
            remainingTime,
            ignoredCollider3D,
            ignoredCollider2D);
    }

    internal bool ApplyContinuousCollisionHandoff(
        Vector3d positionAtImpact,
        FixedQuaternion rotationAtImpact,
        Vector3d postLinearVelocity,
        Vector3d postAngularVelocity,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!CanApplyContinuousCollisionHandoff(
                positionAtImpact,
                remainingTime,
                out Vector3d resolvedPosition))
            return false;

        _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this);

        ApplyContinuousCollisionHandoffReserved(
            resolvedPosition,
            rotationAtImpact,
            postLinearVelocity,
            postAngularVelocity,
            remainingTime,
            ignoredCollider3D,
            ignoredCollider2D);
        return true;
    }

    internal void ApplyContinuousCollisionHandoffReserved(
        Vector3d resolvedPosition,
        FixedQuaternion rotationAtImpact,
        Vector3d postLinearVelocity,
        Vector3d postAngularVelocity,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        Position3d = resolvedPosition;
        Rotation = rotationAtImpact.Normalized;
        UpdateInertiaTensorOrientation();
        ApplyCollisionVelocityState(postLinearVelocity, postAngularVelocity);
        bool hasRemainingMotion = remainingTime > Fixed64.Epsilon
            && (_linearVelocity.MagnitudeSquared > Fixed64.Epsilon
                || _angularVelocity.MagnitudeSquared > Fixed64.Epsilon);
        AppendContinuousCollisionSegment(
            resolvedPosition,
            Rotation,
            hasRemainingMotion ? _linearVelocity : Vector3d.Zero,
            hasRemainingMotion ? _angularVelocity : Vector3d.Zero,
            Context.DeltaTime - remainingTime);
        Context.Physics.RefreshContinuousCollisionCandidate(this);
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
        Context.Physics.QueueContinuousCollisionHandoff(this);
    }

    internal bool CanApplyContinuousCollisionHandoff(
        Vector3d positionAtImpact,
        Fixed64 remainingTime,
        out Vector3d resolvedPosition)
    {
        bool hasMobility = CanTranslate | CanRotate;
        bool hasTrajectoryCapacity = CanAppendContinuousCollisionSegment(
            Context.DeltaTime - remainingTime);
        bool positionDeltaResolved = Vector3d.TrySubtract(
            positionAtImpact,
            Position3d,
            out Vector3d positionDelta);
        bool positionResolved = Vector3d.TryAdd(
            Position3d,
            ProjectLinearMotion(positionDelta),
            out resolvedPosition);
        return hasMobility
            & hasTrajectoryCapacity
            & positionDeltaResolved
            & positionResolved;
    }

    internal bool TryConsumeContinuousCollisionHandoff(bool updateSleepState, bool updateColliderState) =>
        TryConsumeContinuousCollisionHandoff(
            updateSleepState,
            updateColliderState,
            invokeMovementCallback: true,
            out _);

    internal bool TryConsumeQueuedContinuousCollisionHandoff(
        bool updateSleepState,
        bool updateColliderState,
        out bool shouldNotifyMovement) =>
        TryConsumeContinuousCollisionHandoff(
            updateSleepState,
            updateColliderState,
            invokeMovementCallback: false,
            out shouldNotifyMovement);

    private bool TryConsumeContinuousCollisionHandoff(
        bool updateSleepState,
        bool updateColliderState,
        bool invokeMovementCallback,
        out bool shouldNotifyMovement)
    {
        shouldNotifyMovement = false;
        if (!_continuousCollisionHandoffPending || _continuousCollisionHandoffToken != Context.LateSimulateToken)
            return false;

        Fixed64 remainingTime = _continuousCollisionHandoffRemainingTime;
        _continuousCollisionHandoffPending = false;
        _continuousCollisionHandoffRemainingTime = Fixed64.Zero;
        if ((!CanTranslate || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
            && (!CanRotate || _angularVelocity.MagnitudeSquared <= Fixed64.Epsilon))
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
            if (updateColliderState)
                Collider.Simulate();
            return true;
        }

        Vector3d startPosition = Position3d;
        Vector3d proposedPosition = startPosition + _linearVelocity * remainingTime;
        FixedQuaternion startRotation = Rotation;
        FixedQuaternion proposedRotation = IntegrateAngularRotation(
            startRotation,
            _angularVelocity,
            remainingTime);
        Fixed64 elapsedTime = FixedMath.Max(Fixed64.Zero, Context.DeltaTime - remainingTime);
        try
        {
            if (!TryResolveRotationalContinuousCollision(
                    startPosition,
                    ref proposedPosition,
                    startRotation,
                    ref proposedRotation,
                    remainingTime,
                    elapsedTime,
                    forceContinuous: true))
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

        SetPosition2d(proposedPosition.ToVector2d());
        HeightPos = proposedPosition.Y;
        Rotation = proposedRotation;
        UpdateInertiaTensorOrientation();

        CheckGroundForSimulation();
        if (_isGrounded)
            HeightPos = HitPoint.Y;
        else
            ResetGroundCalculations();

        CheckChangedValues();

        if (updateColliderState)
            Collider.Simulate();
        else
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

        if (updateSleepState)
            UpdateSleepState();

        shouldNotifyMovement = PositionChangePending || RotationChangePending;
        if (invokeMovementCallback && shouldNotifyMovement)
            NotifyAuthoritativeMovement();

        return true;
    }

    internal void NotifyAuthoritativeMovement() => OnMoved?.Invoke();

    internal void DiscardContinuousCollisionHandoff()
    {
        _continuousCollisionHandoffPending = false;
        _continuousCollisionHandoffRemainingTime = Fixed64.Zero;
        _continuousCollisionHandoffIgnoredCollider3D = null;
        _continuousCollisionHandoffIgnoredCollider2D = null;
    }

    private bool TryResolveContinuousCollision(Vector3d startPosition, ref Vector3d proposedPosition) =>
        TryResolveContinuousCollision(
            startPosition,
            ref proposedPosition,
            Context.DeltaTime,
            Fixed64.Zero,
            forceContinuous: false);

    private bool TryResolveContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        Fixed64 initialRemainingTime,
        Fixed64 initialElapsedTime,
        bool forceContinuous)
    {
        LastContinuousCollisionToiIterationCount = 0;
        LastContinuousCollisionToiIterationLimitReached = false;

        ContinuousCollisionMode mode = ContinuousCollisionMode.Continuous;
        if (!forceContinuous && !ShouldUseContinuousCollision(out mode))
            return false;

        Vector3d requestedDisplacement = _linearVelocity * initialRemainingTime;
        Vector3d displacement = ContinuousCollisionSweepRange.ValidateEndpoint(
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
        Vector3d currentPosition = startPosition;
        Fixed64 remainingTime = initialRemainingTime;
        Fixed64 elapsedTime = initialElapsedTime;
        int maxToiIterations = Context.Settings.ContinuousCollisionMaxToiIterations;
        LSCollider? originalIgnoredCollider3D = _continuousCollisionHandoffIgnoredCollider3D;
        LSCollider2D? originalIgnoredCollider2D = _continuousCollisionHandoffIgnoredCollider2D;
        try
        {
            for (int toiIteration = 0; toiIteration < maxToiIterations; toiIteration++)
            {
                Vector3d requestedSegmentDisplacement = _linearVelocity * remainingTime;
                Vector3d requestedSegmentEnd = currentPosition + requestedSegmentDisplacement;
                Vector3d segmentDisplacement = ContinuousCollisionSweepRange.ValidateEndpoint(
                    currentPosition,
                    requestedSegmentEnd,
                    requestedSegmentDisplacement,
                    out Fixed64 segmentLength);
                Vector3d segmentEnd = requestedSegmentEnd;

                Fixed64 elapsedFraction = elapsedTime / Context.DeltaTime;
                if (!TryGetFirstContinuousCollisionHit(
                        currentPosition,
                        segmentEnd,
                        proxyRadius,
                        elapsedFraction,
                        out Vector3d hitNormal,
                        out Fixed64 hitDistance,
                        out ContinuousCollisionTargetKind targetKind,
                        out LSCollider? target3D,
                        out LSCollider2D? target2D))
                {
                    currentPosition = segmentEnd;
                    break;
                }

                Fixed64 hitTime = FixedMath.Clamp01(hitDistance / segmentLength);
                currentPosition += segmentDisplacement.Normalized * hitDistance;
                Vector3d previousVelocity = _linearVelocity;
                Fixed64 consumedTime = remainingTime * hitTime;
                Fixed64 remainingAfterHit = remainingTime - consumedTime;
                Fixed64 hitElapsedTime = elapsedTime + consumedTime;
                if (!CanAppendContinuousCollisionSegment(hitElapsedTime))
                {
                    ContinuousCollisionMotionSegment3D activeSegment =
                        ResolveContinuousCollisionSegment(
                            ResolveContinuousCollisionFrameFraction(hitElapsedTime));
                    currentPosition = activeSegment.StartPosition;
                    elapsedTime = activeSegment.StartFraction * Context.DeltaTime;
                    RemoveClosingContinuousCollisionVelocity(hitNormal);
                    UpdateContinuousCollisionFrameTrajectory(
                        currentPosition,
                        _linearVelocity,
                        elapsedTime);
                    Context.Physics.RefreshContinuousCollisionCandidate(this);
                    LastContinuousCollisionToiIterationCount++;
                    LastContinuousCollisionToiIterationLimitReached = true;
                    Context.Physics.ReportContinuousCollisionIterationLimit();
                    resolved = true;
                    break;
                }

                bool appliedResponse = TryApplyContinuousCollisionDynamicResponse(
                        hitNormal,
                        targetKind,
                        target3D,
                        target2D,
                        currentPosition,
                        hitElapsedTime,
                        remainingAfterHit);
                if (!appliedResponse)
                {
                    RemoveClosingContinuousCollisionVelocity(hitNormal);
                    UpdateContinuousCollisionFrameTrajectory(currentPosition, _linearVelocity, hitElapsedTime);
                }
                else if (targetKind == ContinuousCollisionTargetKind.Dynamic3D)
                {
                    _continuousCollisionHandoffIgnoredCollider3D = target3D;
                    _continuousCollisionHandoffIgnoredCollider2D = null;
                }
                else
                {
                    _continuousCollisionHandoffIgnoredCollider3D = null;
                    _continuousCollisionHandoffIgnoredCollider2D = target2D;
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
            _continuousCollisionHandoffIgnoredCollider3D = originalIgnoredCollider3D;
            _continuousCollisionHandoffIgnoredCollider2D = originalIgnoredCollider2D;
        }
    }

    private bool TryApplyContinuousCollisionDynamicResponse(
        Vector3d normalForSource,
        ContinuousCollisionTargetKind targetKind,
        LSCollider? target3D,
        LSCollider2D? target2D,
        Vector3d sourcePositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        if (targetKind == ContinuousCollisionTargetKind.Dynamic3D)
        {
            SolidBody targetBody = target3D!.Body!;
            Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
            Vector3d targetPositionAtImpact = targetBody.SampleContinuousCollisionPosition(
                frameFraction);
            return TryApplyContinuousCollisionDynamicResponse(
                targetBody,
                normalForSource,
                sourcePositionAtImpact,
                targetPositionAtImpact,
                hitElapsedTime,
                remainingTime);
        }

        if (targetKind == ContinuousCollisionTargetKind.Dynamic2D)
        {
            SolidBody2D targetBody = target2D!.Body!;
            Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
            Vector2d targetPositionAtImpact = targetBody.SampleContinuousCollisionPosition(
                frameFraction);
            return TryApplyContinuousCollisionMixed2DResponse(
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
        SolidBody target,
        Vector3d normalForSource,
        Vector3d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        if (!ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(normalForSource, out Vector3d normal))
            return false;

        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(normal);
        Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
        Vector3d targetLinearVelocity = target.SampleContinuousCollisionLinearVelocity(frameFraction);
        bool inverseMassResolved = Fixed64.TryAdd(
            constrainedInverseMassA,
            constrainedInverseMassB,
            out Fixed64 inverseMass);
        bool relativeVelocityResolved = Vector3d.TrySubtract(
            _linearVelocity,
            targetLinearVelocity,
            out Vector3d relativeVelocity);
        if (!(inverseMassResolved & relativeVelocityResolved))
        {
            return false;
        }

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
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

        Vector3d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector3d targetResponseNormal = target.ProjectLinearMotion(-normal);
        bool sourceDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            sourceResponseNormal,
            responseSpeed,
            EffectiveInverseMass,
            inverseMass,
            out Vector3d sourceVelocityDelta);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            target.IsKinematic ? Fixed64.Zero : target.EffectiveInverseMass,
            inverseMass,
            out Vector3d targetVelocityDelta);
        if (!(sourceDeltaResolved & targetDeltaResolved))
        {
            return false;
        }

        bool sourceVelocityResolved = Vector3d.TryAdd(
            _linearVelocity,
            sourceVelocityDelta,
            out Vector3d sourcePostLinearVelocity);
        bool sourceTrajectoryAvailable = CanAppendContinuousCollisionSegment(hitElapsedTime);
        if (!(sourceVelocityResolved & sourceTrajectoryAvailable))
        {
            return false;
        }

        Vector3d targetResolvedPosition = default;
        Vector3d targetPostLinearVelocity = targetLinearVelocity;
        FixedQuaternion targetRotationAtImpact = target.SampleContinuousCollisionRotation(frameFraction);
        Vector3d targetAngularVelocity = target.SampleContinuousCollisionAngularVelocity(frameFraction);
        bool targetStateAvailable = true;
        if (!target.IsKinematic)
        {
            bool targetVelocityResolved = Vector3d.TryAdd(
                targetLinearVelocity,
                targetVelocityDelta,
                out targetPostLinearVelocity);
            bool targetTrajectoryAvailable = target.CanApplyContinuousCollisionHandoff(
                targetPositionAtImpact,
                remainingTime,
                out targetResolvedPosition);
            targetStateAvailable = targetVelocityResolved & targetTrajectoryAvailable;
        }

        if (!targetStateAvailable)
        {
            return false;
        }

        _ = target.IsKinematic
            ? Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this)
            : Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this, target);

        ApplyCollisionVelocityState(sourcePostLinearVelocity, _angularVelocity);
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        Context.Physics.RefreshContinuousCollisionCandidate(this);
        if (!target.IsKinematic)
        {
            target.ApplyContinuousCollisionHandoffReserved(
                targetResolvedPosition,
                targetRotationAtImpact,
                targetPostLinearVelocity,
                targetAngularVelocity,
                remainingTime,
                ignoredCollider3D: Collider);
        }

        return true;
    }

    internal bool TryApplyContinuousCollisionMixed2DResponse(
        SolidBody2D target,
        Vector3d normalForSource,
        Vector3d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        bool normalResolved = ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            normalForSource,
            out Vector3d normal);
        Vector2d planarNormal = normal.ToVector2d();
        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(planarNormal) * planarNormal.MagnitudeSquared;
        Fixed64 frameFraction = ResolveContinuousCollisionFrameFraction(hitElapsedTime);
        Vector2d targetLinearVelocity = target.SampleContinuousCollisionLinearVelocity(frameFraction);
        Vector3d targetVelocity = targetLinearVelocity
            .ToVector3d(Fixed64.Zero);
        bool inverseMassResolved = Fixed64.TryAdd(
            constrainedInverseMassA,
            constrainedInverseMassB,
            out Fixed64 inverseMass);
        bool relativeVelocityResolved = Vector3d.TrySubtract(
            _linearVelocity,
            targetVelocity,
            out Vector3d relativeVelocity);
        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
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
        Vector3d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector2d targetResponseNormal = target.ProjectLinearMotion(-planarNormal);
        bool sourceDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            sourceResponseNormal,
            responseSpeed,
            EffectiveInverseMass,
            inverseMass,
            out Vector3d sourceVelocityDelta);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            target.IsKinematic ? Fixed64.Zero : target.EffectiveInverseMass,
            inverseMass,
            out Vector2d targetVelocityDelta);
        bool sourceVelocityResolved = Vector3d.TryAdd(
            _linearVelocity,
            sourceVelocityDelta,
            out Vector3d sourcePostLinearVelocity);
        bool sourceTrajectoryAvailable = CanAppendContinuousCollisionSegment(hitElapsedTime);
        Vector2d targetResolvedPosition = default;
        Vector2d targetPostLinearVelocity = targetLinearVelocity;
        Fixed64 targetRotationAtImpact = target.SampleContinuousCollisionRotation(frameFraction);
        Fixed64 targetAngularVelocity = target.SampleContinuousCollisionAngularVelocity(frameFraction);
        bool targetStateAvailable = true;
        if (!target.IsKinematic)
        {
            bool targetVelocityResolved = Vector2d.TryAdd(
                targetLinearVelocity,
                targetVelocityDelta,
                out targetPostLinearVelocity);
            bool targetTrajectoryAvailable = target.CanApplyContinuousCollisionHandoffState(
                targetPositionAtImpact,
                remainingTime,
                out targetResolvedPosition);
            targetStateAvailable = targetVelocityResolved & targetTrajectoryAvailable;
        }

        bool responseAdmissible = normalResolved
            & planarNormal != Vector2d.Zero
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

        _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this);
        if (!target.IsKinematic)
            _ = Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(target);

        ApplyCollisionVelocityState(sourcePostLinearVelocity, _angularVelocity);
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        Context.Physics.RefreshContinuousCollisionCandidate(this);
        if (!target.IsKinematic)
        {
            target.ApplyContinuousCollisionHandoffStateReserved(
                targetResolvedPosition,
                targetRotationAtImpact,
                targetPostLinearVelocity,
                targetAngularVelocity,
                remainingTime,
                ignoredCollider3D: Collider);
        }

        return true;
    }

    private void UpdateContinuousCollisionFrameTrajectory(
        Vector3d positionAtElapsedTime,
        Vector3d velocity,
        Fixed64 elapsedTime)
    {
        AppendContinuousCollisionSegment(
            positionAtElapsedTime,
            velocity,
            elapsedTime);
    }

    private Fixed64 ResolveContinuousCollisionFrameFraction(Fixed64 hitElapsedTime)
    {
        return FixedMath.Clamp01(hitElapsedTime / Context.DeltaTime);
    }

    private Fixed64 ResolveContinuousCollisionRestitution(SolidBody target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= Context.Settings.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return PhysicsMaterial.CombineRestitution(Collider.Material, target.Collider.Material);
    }

    private Fixed64 ResolveContinuousCollisionRestitution(SolidBody2D target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= Context.Settings.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return PhysicsMaterial.CombineRestitution(Collider.Material, target.Collider.Material);
    }

    private void RemoveClosingContinuousCollisionVelocity(Vector3d normal)
    {
        Fixed64 closingSpeed = Vector3d.Dot(_linearVelocity, normal);
        if (closingSpeed >= Fixed64.Zero)
            return;

        Vector3d lastVelocity = _linearVelocity;
        _linearVelocity -= normal * closingSpeed;
        RefreshLinearMotionState(lastVelocity);
        Context.Diagnostics.EmitLinearVelocityDelta(this, lastVelocity, _linearVelocity);
    }

}
