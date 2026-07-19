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
    internal void ApplyContinuousCollisionHandoff(
        Vector3d positionAtImpact,
        Vector3d velocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!CanTranslate)
            return;

        Position3d += ProjectLinearMotion(positionAtImpact - Position3d);
        ApplyCollisionLinearVelocityDelta(velocityDelta);
        if (remainingTime <= Fixed64.Epsilon || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
        {
            DiscardContinuousCollisionHandoff();
            return;
        }

        UpdateContinuousCollisionFrameTrajectory(positionAtImpact, _linearVelocity, Context.DeltaTime - remainingTime);
        _continuousCollisionHandoffToken = Context.LateSimulateToken;
        _continuousCollisionHandoffRemainingTime = remainingTime;
        _continuousCollisionHandoffIgnoredCollider3D = ignoredCollider3D;
        _continuousCollisionHandoffIgnoredCollider2D = ignoredCollider2D;
        _continuousCollisionHandoffPending = true;
        Context.Physics.QueueContinuousCollisionHandoff(this);
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
        if (!CanTranslate || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
            if (updateColliderState)
                Collider.Simulate();
            return true;
        }

        Vector3d startPosition = Position3d;
        Vector3d proposedPosition = startPosition + _linearVelocity * remainingTime;
        Fixed64 elapsedTime = FixedMath.Max(Fixed64.Zero, Context.DeltaTime - remainingTime);
        try
        {
            TryResolveContinuousCollision(startPosition, ref proposedPosition, remainingTime, elapsedTime, forceContinuous: true);
        }
        finally
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
        }

        SetPosition2d(proposedPosition.ToVector2d());
        HeightPos = proposedPosition.Y;

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
            Fixed64 remainingFraction = remainingTime / Context.DeltaTime;
            if (!TryGetFirstContinuousCollisionHit(
                    currentPosition,
                    segmentEnd,
                    proxyRadius,
                    elapsedFraction,
                    remainingFraction,
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
            if (!TryApplyContinuousCollisionDynamicResponse(
                    hitNormal,
                    targetKind,
                    target3D,
                    target2D,
                    currentPosition,
                    hitElapsedTime,
                    remainingAfterHit))
            {
                RemoveClosingContinuousCollisionVelocity(hitNormal);
                UpdateContinuousCollisionFrameTrajectory(currentPosition, _linearVelocity, hitElapsedTime);
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
            Vector3d targetPositionAtImpact = targetBody.ContinuousCollisionFrameStart
                + targetBody.ContinuousCollisionFrameDisplacement * frameFraction;
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
            Vector2d targetPositionAtImpact = targetBody.ContinuousCollisionFrameStart
                + targetBody.ContinuousCollisionFrameDisplacement * frameFraction;
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

    private bool TryApplyContinuousCollisionDynamicResponse(
        SolidBody target,
        Vector3d normalForSource,
        Vector3d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        _ = ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(normalForSource, out Vector3d normal);

        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(normal);
        Fixed64 inverseMass = constrainedInverseMassA + constrainedInverseMassB;

        Fixed64 normalVelocity = Vector3d.Dot(_linearVelocity - target.ResolveContinuousCollisionFrameVelocity(), normal);
        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 responseSpeed = -(Fixed64.One + restitution) * normalVelocity;
        Vector3d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector3d targetResponseNormal = target.ProjectLinearMotion(-normal);
        if (!ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                sourceResponseNormal,
                responseSpeed,
                EffectiveInverseMass,
                inverseMass,
                out Vector3d sourceVelocityDelta)
            || !ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                targetResponseNormal,
                responseSpeed,
                target.EffectiveInverseMass,
                inverseMass,
                out Vector3d targetVelocityDelta))
        {
            return false;
        }

        ApplyCollisionLinearVelocityDelta(sourceVelocityDelta);
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            targetVelocityDelta,
            remainingTime);
        return true;
    }

    private bool TryApplyContinuousCollisionMixed2DResponse(
        SolidBody2D target,
        Vector3d normalForSource,
        Vector3d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        _ = ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(normalForSource, out Vector3d normal);

        Vector2d planarNormal = normal.ToVector2d();
        if (planarNormal == Vector2d.Zero)
            return false;

        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(planarNormal) * planarNormal.MagnitudeSquared;
        Fixed64 inverseMass = constrainedInverseMassA + constrainedInverseMassB;

        Vector3d targetVelocity = target.ResolveContinuousCollisionFrameVelocity().ToVector3d(Fixed64.Zero);
        Fixed64 normalVelocity = Vector3d.Dot(_linearVelocity - targetVelocity, normal);
        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 responseSpeed = -(Fixed64.One + restitution) * normalVelocity;
        Vector3d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector2d targetResponseNormal = target.ProjectLinearMotion(-planarNormal);
        if (!ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                sourceResponseNormal,
                responseSpeed,
                EffectiveInverseMass,
                inverseMass,
                out Vector3d sourceVelocityDelta)
            || !ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                targetResponseNormal,
                responseSpeed,
                target.EffectiveInverseMass,
                inverseMass,
                out Vector2d targetVelocityDelta))
        {
            return false;
        }

        ApplyCollisionLinearVelocityDelta(sourceVelocityDelta);
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            targetVelocityDelta,
            remainingTime);
        return true;
    }

    private void UpdateContinuousCollisionFrameTrajectory(
        Vector3d positionAtElapsedTime,
        Vector3d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 elapsedFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Vector3d frameDisplacement = ProjectLinearMotion(velocity) * deltaTime;
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameDisplacement = frameDisplacement;
        _continuousCollisionFrameStart = positionAtElapsedTime - frameDisplacement * elapsedFraction;
        _continuousCollisionFrameRotation = Rotation;
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
