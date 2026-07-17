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
    internal void ApplyContinuousCollisionHandoff(
        Vector2d positionAtImpact,
        Vector2d velocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!CanTranslate)
            return;

        _position += ProjectLinearMotion(positionAtImpact - _position);
        ApplyCollisionLinearVelocityDelta(velocityDelta);
        if (remainingTime <= Fixed64.Epsilon || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
            return;
        }

        UpdateContinuousCollisionFrameTrajectory(positionAtImpact, _linearVelocity, Context.DeltaTime - remainingTime);
        _continuousCollisionHandoffToken = Context.LateSimulateToken;
        _continuousCollisionHandoffRemainingTime = remainingTime;
        _continuousCollisionHandoffIgnoredCollider3D = ignoredCollider3D;
        _continuousCollisionHandoffIgnoredCollider2D = ignoredCollider2D;
        _continuousCollisionHandoffPending = true;
        Context.Physics2D.QueueContinuousCollisionHandoff(this);
    }

    internal bool TryConsumeContinuousCollisionHandoff(bool updateSleepState, bool updateColliderState)
    {
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
                Collider.Rebuild();
            return true;
        }

        Vector2d startPosition = _position;
        Vector2d proposedPosition = startPosition + _linearVelocity * remainingTime;
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
        _position = proposedPosition;
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
        Vector2d currentPosition = startPosition;
        Fixed64 remainingTime = initialRemainingTime;
        Fixed64 elapsedTime = initialElapsedTime;
        int maxToiIterations = Context.Settings.ContinuousCollisionMaxToiIterations;
        for (int toiIteration = 0; toiIteration < maxToiIterations; toiIteration++)
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
            Fixed64 remainingFraction = remainingTime / Context.DeltaTime;
            if (!TryGetFirstContinuousCollisionHit(
                    currentPosition,
                    segmentEnd,
                    proxyRadius,
                    elapsedFraction,
                    remainingFraction,
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
            if (!TryApplyContinuousCollisionDynamicResponse(
                    hitNormal,
                    targetKind,
                    target2D,
                    target3D,
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
            Vector2d targetPositionAtImpact = targetBody.ContinuousCollisionFrameStart
                + targetBody.ContinuousCollisionFrameDisplacement * frameFraction;
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
            Vector3d targetPositionAtImpact = targetBody.ContinuousCollisionFrameStart
                + targetBody.ContinuousCollisionFrameDisplacement * frameFraction;
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

    private bool TryApplyContinuousCollisionDynamicResponse(
        SolidBody2D target,
        Vector2d normalForSource,
        Vector2d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        _ = ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(normalForSource, out Vector2d normal);

        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(normal);
        Fixed64 inverseMass = constrainedInverseMassA + constrainedInverseMassB;

        Fixed64 normalVelocity = Vector2d.Dot(_linearVelocity - target.ResolveContinuousCollisionFrameVelocity(), normal);
        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 responseSpeed = -(Fixed64.One + restitution) * normalVelocity;
        Vector2d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector2d targetResponseNormal = target.ProjectLinearMotion(-normal);
        if (!ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                sourceResponseNormal,
                responseSpeed,
                EffectiveInverseMass,
                inverseMass,
                out Vector2d sourceVelocityDelta)
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

    private bool TryApplyContinuousCollisionMixed3DResponse(
        SolidBody target,
        Vector2d normalForSource,
        Vector2d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        if (!ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(normalForSource, out Vector2d normal))
            return false;

        Vector3d normal3D = normal.ToVector3d(Fixed64.Zero);
        Fixed64 constrainedInverseMassA = GetConstrainedInverseMass(normal);
        Fixed64 constrainedInverseMassB = target.GetConstrainedInverseMass(normal3D);
        Fixed64 inverseMass = constrainedInverseMassA + constrainedInverseMassB;

        Vector3d targetVelocity = target.ResolveContinuousCollisionFrameVelocity();
        Fixed64 normalVelocity = Vector3d.Dot(_linearVelocity.ToVector3d(Fixed64.Zero) - targetVelocity, normal3D);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 responseSpeed = -(Fixed64.One + restitution) * normalVelocity;
        Vector2d sourceResponseNormal = ProjectLinearMotion(normal);
        Vector3d targetResponseNormal = target.ProjectLinearMotion(-normal3D);
        if (!ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                sourceResponseNormal,
                responseSpeed,
                EffectiveInverseMass,
                inverseMass,
                out Vector2d sourceVelocityDelta)
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

    private void UpdateContinuousCollisionFrameTrajectory(
        Vector2d positionAtElapsedTime,
        Vector2d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 elapsedFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Vector2d frameDisplacement = ProjectLinearMotion(velocity) * deltaTime;
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameDisplacement = frameDisplacement;
        _continuousCollisionFrameStart = positionAtElapsedTime - frameDisplacement * elapsedFraction;
        _continuousCollisionFrameRotation = _rotation;
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
