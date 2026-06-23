//=======================================================================
// StiffBody.ContinuousCollision.cs
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
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class StiffBody
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector3d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameStart;
    }

    internal Vector3d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameDisplacement;
    }

    internal FixedQuaternion ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameRotation;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        _continuousCollisionFrameToken = token;
        _continuousCollisionFrameStart = Position3d;
        _continuousCollisionFrameDisplacement = PredictContinuousCollisionDisplacement();
        _continuousCollisionFrameRotation = Rotation;
    }

    private Vector3d PredictContinuousCollisionDisplacement()
    {
        if (!Active || Immovable || IsKinematic || _isSleeping)
            return Vector3d.Zero;

        Fixed64 deltaTime = Context.DeltaTime;
        PhysicsEnvironment environment = Context.Environment;
        Vector3d predictedVelocity = _linearVelocity + _impulseStore + (_deltaAcceleration * deltaTime);
        if (!_isGrounded)
            predictedVelocity.Y -= environment.Gravity * deltaTime;

        predictedVelocity.Y = FixedMath.Max(predictedVelocity.Y, -environment.MaxFallSpeed);
        Fixed64 predictedSpeed = predictedVelocity.Magnitude;
        if (predictedSpeed > environment.MaxSpeed)
            predictedVelocity = predictedVelocity.Normalized * environment.MaxSpeed;
        else if (predictedSpeed <= environment.MinSpeed)
            predictedVelocity = Vector3d.Zero;

        return predictedVelocity * deltaTime;
    }

    private void CaptureKinematicContinuousCollisionFrame(
        Vector3d startPosition,
        Vector3d targetPosition,
        FixedQuaternion startRotation)
    {
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameStart = startPosition;
        _continuousCollisionFrameDisplacement = targetPosition - startPosition;
        _continuousCollisionFrameRotation = startRotation;
    }

    private bool TryResolveKinematicContinuousCollision(Vector3d startPosition, ref Vector3d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector3d displacement = proposedPosition - startPosition;
        Fixed64 displacementMagnitudeSquared = displacement.MagnitudeSquared;
        if (displacementMagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && displacementMagnitudeSquared <= proxyRadius * proxyRadius))
        {
            return false;
        }

        Fixed64 sourceLength = FixedMath.Sqrt(displacementMagnitudeSquared);
        bool foundStatic = TryGetFirstKinematicStaticContinuousCollisionHit(
            startPosition,
            proposedPosition,
            proxyRadius,
            out Fixed64 staticHitDistance);
        Fixed64 dynamicPushLimit = foundStatic ? staticHitDistance : sourceLength;
        bool pushedDynamic = TryApplyKinematicDynamicContinuousCollisionPushes(
            startPosition,
            proposedPosition,
            proxyRadius,
            dynamicPushLimit,
            sourceLength);
        if (!foundStatic && !pushedDynamic)
            return false;

        LastContinuousCollisionToiIterationCount++;
        if (foundStatic)
            proposedPosition = startPosition + displacement / sourceLength * staticHitDistance;

        return true;
    }

    private bool TryGetFirstKinematicStaticContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        out Fixed64 distance)
    {
        Vector3d originalPosition = Position3d;
        bool originalPositionMutated = _positionMutated;
        try
        {
            Position3d = startPosition;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

            int hitCount = Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);
            int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
                ? Context.QueryMixed.SweepSphereAgainstStatic2DAll(
                    startPosition,
                    proposedPosition,
                    proxyRadius,
                    PhysicsLayerMask.All,
                    _continuousMixedCollisionHits,
                    Collider,
                    includeTriggers: false,
                    cacheTargetPartitions: true)
                : 0;

            bool found3D = TryGetFirstValidContinuousCollisionHit(startPosition, proposedPosition, hitCount, out Physics3DHit hit3D);
            bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(startPosition, proposedPosition, mixedHitCount, out PhysicsMixedHit hitMixed);
            if (found3D && (!foundMixed || hit3D.Distance <= hitMixed.Distance))
            {
                distance = hit3D.Distance;
                return true;
            }

            if (foundMixed)
            {
                distance = hitMixed.Distance;
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
        }

        distance = Fixed64.Zero;
        return false;
    }

    private bool TryApplyKinematicDynamicContinuousCollisionPushes(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        bool pushed = TryApplyKinematicDynamic3DContinuousCollisionPushes(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);
        pushed |= TryApplyKinematicDynamicMixed2DContinuousCollisionPushes(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);
        return pushed;
    }

    private bool TryApplyKinematicDynamic3DContinuousCollisionPushes(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        Vector3d sourceDisplacement = proposedPosition - startPosition;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        bool pushed = false;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out StiffBody target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector3d targetStart = target.ContinuousCollisionFrameStart;
            Vector3d targetDisplacement = target.ContinuousCollisionFrameDisplacement;
            Fixed64 targetRadius = ResolveContinuousCollisionProxyRadius(target.Collider);
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normal,
                    out _))
            {
                continue;
            }

            if (TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    out Physics3DHit exactHit,
                    out _,
                    out bool exactSupported))
            {
                normal = exactHit.Normal;
                normalizedTime = sourceLength > Fixed64.Epsilon
                    ? FixedMath.Clamp01(exactHit.Distance / sourceLength)
                    : normalizedTime;
            }
            else if (exactSupported)
                continue;

            Fixed64 distance = sourceLength * normalizedTime;
            if (distance > maxDistance)
                continue;

            Fixed64 frameFraction = sourceLength > Fixed64.Epsilon
                ? FixedMath.Clamp01(distance / sourceLength)
                : Fixed64.Zero;
            Vector3d targetPositionAtImpact = targetStart + targetDisplacement * frameFraction;
            if (ApplyKinematicContinuousCollisionHandoff(
                    target,
                    sourceDisplacement,
                    normal,
                    targetPositionAtImpact,
                    distance,
                    sourceLength))
            {
                pushed = true;
            }
        }

        return pushed;
    }

    private bool TryApplyKinematicDynamicMixed2DContinuousCollisionPushes(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        bool pushed = false;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryMixedContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out StiffBody2D target)
                || !IsEligibleDynamicMixed2DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = FixedMath.Max(
                target.ResolveContinuousCollisionProxyRadiusForDynamicTarget(),
                target.Collider.MixedHalfThickness);
            if (targetRadius <= Fixed64.Epsilon)
                continue;

            Vector2d targetStart2D = target.ContinuousCollisionFrameStart;
            Vector2d targetDisplacement2D = target.ContinuousCollisionFrameDisplacement;
            Vector3d targetStart = new(targetStart2D.X, target.Collider.MixedSlabCenterY, targetStart2D.Y);
            Vector3d targetDisplacement = new(targetDisplacement2D.X, Fixed64.Zero, targetDisplacement2D.Y);
            if (!ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normalForSource,
                    out _))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            if (distance > maxDistance)
                continue;

            Fixed64 frameFraction = sourceLength > Fixed64.Epsilon
                ? FixedMath.Clamp01(distance / sourceLength)
                : Fixed64.Zero;
            Vector2d targetPositionAtImpact = targetStart2D + targetDisplacement2D * frameFraction;
            if (ApplyKinematicContinuousCollisionHandoff(
                    target,
                    sourceDisplacement,
                    normalForSource,
                    targetPositionAtImpact,
                    distance,
                    sourceLength))
            {
                pushed = true;
            }
        }

        return pushed;
    }

    private bool ApplyKinematicContinuousCollisionHandoff(
        StiffBody target,
        Vector3d sourceDisplacement,
        Vector3d normalForSource,
        Vector3d targetPositionAtImpact,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        Vector3d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : -ResolveKinematicPushAxis(sourceDisplacement, sourceDisplacement);
        if (normal == Vector3d.Zero)
            return false;

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 inverseMass = target.EffectiveInverseMass;
        if (deltaTime <= Fixed64.Epsilon || inverseMass <= Fixed64.Epsilon)
            return false;

        Vector3d sourceVelocity = sourceDisplacement / deltaTime;
        Vector3d relativeVelocity = sourceVelocity - target.ResolveContinuousCollisionFrameVelocity();
        if (Vector3d.Dot(relativeVelocity, normal) > Fixed64.Zero)
            normal = -normal;

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / inverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return false;

        Fixed64 hitTime = sourceLength > Fixed64.Epsilon
            ? FixedMath.Clamp01(hitDistance / sourceLength)
            : Fixed64.Zero;
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -normal * (impulseScalar * inverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider3D: Collider);
        return true;
    }

    private bool ApplyKinematicContinuousCollisionHandoff(
        StiffBody2D target,
        Vector3d sourceDisplacement,
        Vector3d normalForSource,
        Vector2d targetPositionAtImpact,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        Vector3d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : -ResolveKinematicPushAxis(sourceDisplacement, sourceDisplacement);
        if (normal == Vector3d.Zero)
            return false;

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 inverseMass = target.EffectiveInverseMass;
        if (deltaTime <= Fixed64.Epsilon || inverseMass <= Fixed64.Epsilon)
            return false;

        Vector3d sourceVelocity = sourceDisplacement / deltaTime;
        Vector3d targetVelocity = target.ResolveContinuousCollisionFrameVelocity().ToVector3d(Fixed64.Zero);
        Vector3d relativeVelocity = sourceVelocity - targetVelocity;
        if (Vector3d.Dot(relativeVelocity, normal) > Fixed64.Zero)
            normal = -normal;

        Vector2d planarNormal = normal.ToVector2d();
        if (planarNormal == Vector2d.Zero)
            return false;

        Fixed64 planarScaleSquared = planarNormal.MagnitudeSquared;
        Fixed64 effectiveInverseMass = inverseMass * planarScaleSquared;
        if (effectiveInverseMass <= Fixed64.Epsilon)
            return false;

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / effectiveInverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return false;

        Fixed64 hitTime = sourceLength > Fixed64.Epsilon
            ? FixedMath.Clamp01(hitDistance / sourceLength)
            : Fixed64.Zero;
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -planarNormal * (impulseScalar * inverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider3D: Collider);
        return true;
    }

    private static Vector3d ResolveKinematicPushAxis(Vector3d candidate, Vector3d fallback)
    {
        if (candidate.MagnitudeSquared > Fixed64.Epsilon)
            return candidate.Normalized;

        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? fallback.Normalized
            : Vector3d.Zero;
    }

    private static Vector2d ResolveKinematicPushAxis(Vector2d candidate, Vector2d fallback)
    {
        if (candidate.MagnitudeSquared > Fixed64.Epsilon)
            return candidate.Normalized;

        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? fallback.Normalized
            : Vector2d.Zero;
    }

    internal void ApplyContinuousCollisionHandoff(
        Vector3d positionAtImpact,
        Vector3d velocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!CanTranslate)
            return;

        Position3d = positionAtImpact;
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
        Context.Physics.QueueContinuousCollisionHandoff(this);
    }

    internal bool TryConsumeContinuousCollisionHandoff(bool updateSleepState, bool updateColliderState)
    {
        if (!_continuousCollisionHandoffPending || _continuousCollisionHandoffToken != Context.LateSimulateToken)
            return false;

        Fixed64 remainingTime = _continuousCollisionHandoffRemainingTime;
        _continuousCollisionHandoffPending = false;
        _continuousCollisionHandoffRemainingTime = Fixed64.Zero;
        if (!CanTranslate || remainingTime <= Fixed64.Epsilon || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
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

        Position2d = _positionCorrection + proposedPosition.ToVector2d();
        _positionCorrection = Vector2d.Zero;
        HeightPos = proposedPosition.Y;

        CheckGroundForSimulation();
        if (_isGrounded)
            HeightPos = HitPoint.Y;
        else
            ResetGroundCalculations();

        CheckChangedValues();
        UpdateInertiaTensorOrientation();
        ApplyGyroscopicPrecession();

        if (updateColliderState)
            Collider.Simulate();
        else
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

        if (updateSleepState)
            UpdateSleepState();

        if (PositionChangePending || RotationChangePending)
            OnMoved?.Invoke();

        return true;
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

        Vector3d displacement = proposedPosition - startPosition;
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && displacement.MagnitudeSquared <= proxyRadius * proxyRadius))
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
            Vector3d segmentDisplacement = _linearVelocity * remainingTime;
            Fixed64 segmentLength = segmentDisplacement.Magnitude;
            if (segmentLength <= Fixed64.Epsilon)
                break;

            Vector3d segmentEnd = currentPosition + segmentDisplacement;
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

    private bool TryGetFirstContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out Vector3d normal,
        out Fixed64 distance,
        out ContinuousCollisionTargetKind targetKind,
        out LSCollider? target3D,
        out LSCollider2D? target2D)
    {
        Vector3d originalPosition = Position3d;
        bool originalPositionMutated = _positionMutated;
        try
        {
            Position3d = startPosition;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

            int hitCount = Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);
            int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
                ? Context.QueryMixed.SweepSphereAgainstStatic2DAll(
                    startPosition,
                    proposedPosition,
                    proxyRadius,
                    PhysicsLayerMask.All,
                    _continuousMixedCollisionHits,
                    Collider,
                    includeTriggers: false,
                    cacheTargetPartitions: true)
                : 0;

            bool found3D = TryGetFirstValidContinuousCollisionHit(startPosition, proposedPosition, hitCount, out Physics3DHit hit3D);
            ContinuousCollisionTargetKind hit3DKind = found3D
                ? ContinuousCollisionTargetKind.Static3D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamic3D = TryGetFirstDynamicContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                remainingFrameFraction,
                out Physics3DHit dynamicHit3D,
                out Fixed64 dynamicClosingSpeed3D);
            if (ShouldReplaceContinuousCollisionHit(dynamicHit3D, dynamicClosingSpeed3D, foundDynamic3D, found3D, hit3D, Fixed64.Zero))
            {
                hit3D = dynamicHit3D;
                found3D = true;
                hit3DKind = ContinuousCollisionTargetKind.Dynamic3D;
            }

            bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(startPosition, proposedPosition, mixedHitCount, out PhysicsMixedHit hitMixed);
            ContinuousCollisionTargetKind hitMixedKind = foundMixed
                ? ContinuousCollisionTargetKind.Static2D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamicMixed = TryGetFirstDynamicMixedContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                remainingFrameFraction,
                out PhysicsMixedHit dynamicHitMixed,
                out Fixed64 dynamicClosingSpeedMixed);
            if (ShouldReplaceMixedContinuousCollisionHit(dynamicHitMixed, dynamicClosingSpeedMixed, foundDynamicMixed, foundMixed, hitMixed, Fixed64.Zero))
            {
                hitMixed = dynamicHitMixed;
                foundMixed = true;
                hitMixedKind = ContinuousCollisionTargetKind.Dynamic2D;
            }

            if (found3D && (!foundMixed || hit3D.Distance <= hitMixed.Distance))
            {
                normal = hit3D.Normal;
                distance = hit3D.Distance;
                targetKind = hit3DKind;
                target3D = hit3D.Collider;
                target2D = null;
                return true;
            }

            if (foundMixed)
            {
                normal = hitMixed.NormalFor3DSource;
                distance = hitMixed.Distance;
                targetKind = hitMixedKind;
                target3D = null;
                target2D = hitMixed.Collider2D;
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
        }

        normal = Vector3d.Zero;
        distance = Fixed64.Zero;
        targetKind = ContinuousCollisionTargetKind.None;
        target3D = null;
        target2D = null;
        return false;
    }

    private bool TryGetFirstValidContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        int hitCount,
        out Physics3DHit hit)
    {
        Vector3d displacement = proposedPosition - startPosition;
        Vector3d direction = displacement.MagnitudeSquared > Fixed64.Epsilon ? displacement.Normalized : Vector3d.Zero;
        bool found = false;
        Physics3DHit best = default;
        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit candidate = _continuousCollisionHits[i];
            if (!IsValidContinuousCollisionHit(candidate))
                continue;

            Physics3DHit refined;
            if (TryRefineShapeExactContinuousCollisionHit(candidate, displacement, direction, out Physics3DHit exactHit, out bool exactSupported))
                refined = exactHit;
            else if (exactSupported)
                continue;
            else
                refined = candidate;

            if (!IsClosingContinuousCollisionHit(displacement, refined.Normal))
                continue;

            if (found && !ContinuousCollisionHitComesBefore(refined, best))
                continue;

            best = refined;
            found = true;
        }

        hit = best;
        return found;
    }

    private bool TryGetFirstValidMixedContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        int hitCount,
        out PhysicsMixedHit hit)
    {
        Vector3d displacement = proposedPosition - startPosition;
        for (int i = 0; i < hitCount; i++)
        {
            PhysicsMixedHit candidate = _continuousMixedCollisionHits[i];
            if (!IsValidMixedContinuousCollisionHit(candidate)
                || !IsClosingContinuousCollisionHit(displacement, candidate.NormalFor3DSource))
                continue;

            hit = candidate;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryRefineShapeExactContinuousCollisionHit(
        Physics3DHit candidate,
        Vector3d displacement,
        Vector3d direction,
        out Physics3DHit refined,
        out bool exactSupported)
    {
        refined = default;
        exactSupported = false;
        LSCollider? target = candidate.Collider;
        if (target == null || displacement.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        if (Collider is LSSphereCollider)
            return false;

        if (target is LSSphereCollider targetSphere)
            return TryRefineContinuousCollisionAgainstTargetSphere(targetSphere, displacement, direction, out refined, out exactSupported);

        if (!IsExactConvexSourceSupported(Collider))
            return false;

        exactSupported = true;
        PrepareExactConvexSourceSweep(displacement);
        if (!_shapeExactContinuousConvexSweepWorker.TrySweepPreparedSource(target, out Physics3DHit convexHit))
            return false;

        refined = ApplyShapeExactContinuousContactSlop(convexHit);
        return true;
    }

    private bool TryRefineContinuousCollisionAgainstTargetSphere(
        LSSphereCollider targetSphere,
        Vector3d displacement,
        Vector3d direction,
        out Physics3DHit refined,
        out bool exactSupported)
    {
        exactSupported = true;
        refined = default;

        Vector3d reverseStart = targetSphere.Center;
        Vector3d reverseEnd = targetSphere.Center - displacement;
        _shapeExactContinuousSweepWorker.Prepare(reverseStart, reverseEnd, targetSphere.ScaledRadius);
        if (!_shapeExactContinuousSweepWorker.TrySweep(Collider, out Vector3d reverseCenterAtImpact, out Fixed64 distance))
            return false;

        Vector3d sourcePoint = Collider.ClosestPointOnSurface(reverseCenterAtImpact);
        Vector3d normalDelta = sourcePoint - reverseCenterAtImpact;
        Vector3d normal = normalDelta.MagnitudeSquared > Fixed64.Epsilon
            ? normalDelta.Normalized
            : -direction;
        Vector3d point = targetSphere.Center + normal * targetSphere.ScaledRadius;
        refined = new Physics3DHit(targetSphere, point, normal, distance, direction);
        return true;
    }

    private static Physics3DHit ApplyShapeExactContinuousContactSlop(Physics3DHit hit)
    {
        Fixed64 distance = hit.Distance > ShapeExactContinuousContactSlop
            ? hit.Distance - ShapeExactContinuousContactSlop
            : Fixed64.Zero;
        return new Physics3DHit(hit.Collider, hit.Point, hit.Normal, distance, hit.Direction);
    }

    private void PrepareExactConvexSourceSweep(Vector3d displacement)
    {
        switch (Collider)
        {
            case LSMeshCollider mesh:
                _shapeExactContinuousConvexSweepWorker.PrepareConvexMeshSource(mesh, displacement);
                return;
            case LSCompoundCollider compound:
                _shapeExactContinuousConvexSweepWorker.PrepareCompoundSource(compound, displacement);
                return;
            default:
                _shapeExactContinuousConvexSweepWorker.PreparePrimitiveSource(Collider, displacement);
                return;
        }
    }

    private static bool IsExactConvexSourceSupported(LSCollider collider)
    {
        return collider switch
        {
            LSSphereCollider => true,
            LSCapsuleCollider => true,
            LSCuboidCollider => true,
            LSCylinderCollider => true,
            LSMeshCollider { Mode: MeshColliderMode.Convex } => true,
            LSCompoundCollider compound => AreExactConvexCompoundPartsSupported(compound),
            _ => false
        };
    }

    private static bool AreExactConvexCompoundPartsSupported(LSCompoundCollider compound)
    {
        for (int i = 0; i < compound.PartCount; i++)
        {
            if (!IsExactConvexSourceSupported(compound.GetPartCollider(i)))
                return false;
        }

        return true;
    }

    private static bool ContinuousCollisionHitComesBefore(Physics3DHit left, Physics3DHit right)
    {
        int distanceCompare = left.Distance.CompareTo(right.Distance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        return (left.Collider?.Id ?? -1) < (right.Collider?.Id ?? -1);
    }

    private bool TryGetFirstDynamicContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out Physics3DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        bool found = false;
        Physics3DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out StiffBody target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector3d targetStart = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector3d targetDisplacement = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Fixed64 targetRadius = ResolveContinuousCollisionProxyRadius(target.Collider);
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normal,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Physics3DHit candidate;
            if (TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    out Physics3DHit exactHit,
                    out Fixed64 exactClosingSpeed,
                    out bool exactSupported))
            {
                candidate = exactHit;
                candidateClosingSpeed = exactClosingSpeed;
            }
            else if (exactSupported)
            {
                continue;
            }
            else
            {
                Fixed64 distance = sourceLength * normalizedTime;
                Vector3d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
                Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
                Vector3d point = ResolveDynamicContactPoint(sourceCenter, targetCenter, normal, targetRadius);
                candidate = new Physics3DHit(target.Collider, point, normal, distance, sourceDirection);
            }

            if (!ShouldReplaceContinuousCollisionHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool TryGetExactDynamicRelativeContinuousCollisionHit(
        StiffBody target,
        Vector3d sourceStart,
        Vector3d sourceDisplacement,
        Vector3d targetStart,
        Vector3d targetDisplacement,
        Fixed64 sourceLength,
        out Physics3DHit hit,
        out Fixed64 closingSpeed,
        out bool exactSupported)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;
        exactSupported = false;

        Vector3d relativeDisplacement = sourceDisplacement - targetDisplacement;
        Fixed64 relativeLength = relativeDisplacement.Magnitude;
        if (relativeLength <= Fixed64.Epsilon || sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d relativeDirection = relativeDisplacement / relativeLength;
        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        Vector3d originalSourcePosition = Position3d;
        FixedQuaternion originalSourceRotation = Rotation;
        bool originalSourcePositionMutated = _positionMutated;
        bool originalSourceRotationMutated = _rotationMutated;
        Vector3d originalTargetPosition = target.Position3d;
        FixedQuaternion originalTargetRotation = target.Rotation;
        bool originalTargetPositionMutated = target._positionMutated;
        bool originalTargetRotationMutated = target._rotationMutated;

        try
        {
            Position3d = sourceStart;
            target.Position3d = targetStart;
            target.Rotation = target.ContinuousCollisionFrameRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            target.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

            Physics3DHit relativeHit;
            if (Collider is LSSphereCollider sourceSphere)
            {
                exactSupported = true;
                if (!TrySweepRelativeSourceSphere(sourceSphere, target.Collider, relativeDisplacement, relativeDirection, out relativeHit))
                    return false;
            }
            else if (target.Collider is LSSphereCollider targetSphere)
            {
                if (!TryRefineContinuousCollisionAgainstTargetSphere(targetSphere, relativeDisplacement, relativeDirection, out relativeHit, out exactSupported))
                    return false;
            }
            else if (IsExactConvexSourceSupported(Collider))
            {
                exactSupported = true;
                PrepareExactConvexSourceSweep(relativeDisplacement);
                if (!_shapeExactContinuousConvexSweepWorker.TrySweepPreparedSource(target.Collider, out Physics3DHit convexHit))
                    return false;

                relativeHit = ApplyShapeExactContinuousContactSlop(convexHit);
            }
            else
            {
                return false;
            }

            closingSpeed = -Vector3d.Dot(relativeDisplacement, relativeHit.Normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 normalizedTime = FixedMath.Clamp01(relativeHit.Distance / relativeLength);
            hit = new Physics3DHit(
                target.Collider,
                relativeHit.Point + targetDisplacement * normalizedTime,
                relativeHit.Normal,
                sourceLength * normalizedTime,
                sourceDirection);
            return true;
        }
        finally
        {
            Position3d = originalSourcePosition;
            Rotation = originalSourceRotation;
            target.Position3d = originalTargetPosition;
            target.Rotation = originalTargetRotation;
            target.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            target._positionMutated = originalTargetPositionMutated;
            target._rotationMutated = originalTargetRotationMutated;
            _positionMutated = originalSourcePositionMutated;
            _rotationMutated = originalSourceRotationMutated;
        }
    }

    private bool TrySweepRelativeSourceSphere(
        LSSphereCollider sourceSphere,
        LSCollider target,
        Vector3d relativeDisplacement,
        Vector3d relativeDirection,
        out Physics3DHit hit)
    {
        hit = default;
        _shapeExactContinuousSweepWorker.Prepare(
            sourceSphere.Center,
            sourceSphere.Center + relativeDisplacement,
            sourceSphere.ScaledRadius);
        if (!_shapeExactContinuousSweepWorker.TrySweep(target, out Vector3d sphereCenterAtImpact, out Fixed64 distance))
            return false;

        Vector3d point = ResolveSweptSphereContinuousPoint(target, sphereCenterAtImpact, relativeDirection);
        Vector3d normal = ResolveSweptSphereContinuousNormal(target, point, sphereCenterAtImpact, relativeDirection);
        hit = new Physics3DHit(target, point, normal, distance, relativeDirection);
        return true;
    }

    private static Vector3d ResolveSweptSphereContinuousPoint(
        LSCollider target,
        Vector3d sphereCenterAtImpact,
        Vector3d direction)
    {
        Vector3d centerDelta = sphereCenterAtImpact - target.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return target.Center - direction * target.ScaledRadius;

        return target.ClosestPointOnSurface(sphereCenterAtImpact);
    }

    private static Vector3d ResolveSweptSphereContinuousNormal(
        LSCollider target,
        Vector3d point,
        Vector3d sphereCenterAtImpact,
        Vector3d direction)
    {
        Vector3d fromPointToSphereCenter = sphereCenterAtImpact - point;
        if ((target is LSCuboidCollider || target is LSCylinderCollider)
            && fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
        {
            return fromPointToSphereCenter.Normalized;
        }

        Vector3d normal = target.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normal.Normalized;
            if (target is LSMeshCollider && Vector3d.Dot(normal, direction) > Fixed64.Zero)
                return -normal;

            return normal;
        }

        if (fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
            return fromPointToSphereCenter.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Zero;
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out PhysicsMixedHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryMixedContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out StiffBody2D target)
                || !IsEligibleDynamicMixed2DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = FixedMath.Max(
                target.ResolveContinuousCollisionProxyRadiusForDynamicTarget(),
                target.Collider.MixedHalfThickness);
            if (targetRadius <= Fixed64.Epsilon)
                continue;

            Vector2d targetStart2D = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector2d targetDisplacement2D = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Vector3d targetStart = new(targetStart2D.X, target.Collider.MixedSlabCenterY, targetStart2D.Y);
            Vector3d targetDisplacement = new(targetDisplacement2D.X, Fixed64.Zero, targetDisplacement2D.Y);
            if (!ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normalForSource,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            Vector3d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
            Vector3d point3D = sourceCenter - normalForSource * proxyRadius;
            Vector3d point2D = ResolveDynamicContactPoint(sourceCenter, targetCenter, normalForSource, targetRadius);
            var candidate = new PhysicsMixedHit(
                null,
                target.Collider,
                point3D,
                point2D,
                -normalForSource,
                PhysicsQueryReducerKind.ConservativeFallback,
                distance,
                sourceDirection);
            if (!ShouldReplaceMixedContinuousCollisionHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool IsEligibleDynamicContinuousCollisionTarget(StiffBody target)
    {
        if (ReferenceEquals(target, this)
            || !target.Active
            || target.Immovable
            || target.IsKinematic
            || target.Collider.IsTrigger
            || target.Collider.IsSibling(Collider)
            || Context.Physics.IsLayerCollisionDisabled(Collider.Layer, target.Collider.Layer))
        {
            return false;
        }

        return true;
    }

    private bool IsEligibleDynamicMixed2DTarget(StiffBody2D target)
    {
        return target.Active
            && !target.Immovable
            && !target.IsKinematic
            && !target.Collider.IsTrigger
            && Context.MixedCollisions.RequireCollisionPair(Collider, target.Collider);
    }

    private static bool ShouldReplaceContinuousCollisionHit(
        Physics3DHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        Physics3DHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        int candidateId = candidate.Collider?.Id ?? -1;
        int currentId = current.Collider?.Id ?? -1;
        return candidateId < currentId;
    }

    private static bool ShouldReplaceMixedContinuousCollisionHit(
        PhysicsMixedHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        PhysicsMixedHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        int candidate3D = candidate.Collider3D?.Id ?? -1;
        int current3D = current.Collider3D?.Id ?? -1;
        int collider3D = candidate3D.CompareTo(current3D);
        if (collider3D != 0)
            return collider3D < 0;

        int candidate2D = candidate.Collider2D?.Id ?? -1;
        int current2D = current.Collider2D?.Id ?? -1;
        return candidate2D < current2D;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveDynamicContactPoint(
        Vector3d sourceCenter,
        Vector3d targetCenter,
        Vector3d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector3d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool ShouldUseContinuousCollision(out ContinuousCollisionMode mode)
    {
        mode = ResolveContinuousCollisionMode();
        return mode == ContinuousCollisionMode.Continuous || mode == ContinuousCollisionMode.Auto;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ContinuousCollisionMode ResolveContinuousCollisionMode()
    {
        ContinuousCollisionMode mode = _continuousCollisionMode;
        if (mode != ContinuousCollisionMode.Inherit)
            return mode;

        StiffBody? parentBody = Collider.TopParent3D?.Body;
        if (parentBody != null && parentBody._continuousCollisionMode != ContinuousCollisionMode.Inherit)
            return parentBody._continuousCollisionMode;

        mode = Context.Settings.DefaultContinuousCollisionMode;
        return mode == ContinuousCollisionMode.Inherit
            ? ContinuousCollisionMode.Discrete
            : mode;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        return ResolveContinuousCollisionProxyRadius(Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadiusForDynamicTarget()
    {
        return ResolveContinuousCollisionProxyRadius(Collider);
    }

    internal Vector3d ResolveContinuousCollisionFrameVelocity()
    {
        Fixed64 deltaTime = Context.DeltaTime;
        return deltaTime > Fixed64.Epsilon
            ? _continuousCollisionFrameDisplacement / deltaTime
            : Vector3d.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveContinuousCollisionProxyRadius(LSCollider collider)
    {
        return collider switch
        {
            LSSphereCollider sphere => sphere.ScaledRadius,
            _ => ResolveBoundsProxyRadius(collider)
        };
    }

    private static Fixed64 ResolveBoundsProxyRadius(LSCollider collider)
    {
        Fixed64 radius = collider.Bounds.Scope.Magnitude;
        return radius > Fixed64.Epsilon ? radius : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsValidContinuousCollisionHit(Physics3DHit hit) =>
        IsValidContinuousCollisionTarget(hit.Collider);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClosingContinuousCollisionHit(Vector3d displacement, Vector3d normal) =>
        normal.MagnitudeSquared > Fixed64.Epsilon
        && Vector3d.Dot(displacement, normal) < -Fixed64.Epsilon;

    private bool IsValidContinuousCollisionTarget(LSCollider? hitCollider)
    {
        if (hitCollider == null
            || ReferenceEquals(hitCollider, Collider)
            || IsIgnoredContinuousCollisionTarget(hitCollider)
            || hitCollider.IsTrigger
            || hitCollider.IsSibling(Collider)
            || Context.Physics.IsLayerCollisionDisabled(Collider.Layer, hitCollider.Layer))
        {
            return false;
        }

        StiffBody? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private bool IsValidMixedContinuousCollisionHit(PhysicsMixedHit hit)
    {
        LSCollider2D? hitCollider = hit.Collider2D;
        if (hitCollider == null
            || IsIgnoredMixedContinuousCollisionTarget(hitCollider)
            || hitCollider.IsTrigger
            || !Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider))
        {
            return false;
        }

        StiffBody2D? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private bool IsIgnoredContinuousCollisionTarget(LSCollider hitCollider)
    {
        LSCollider? ignored = _continuousCollisionHandoffIgnoredCollider3D;
        if (ignored == null)
            return false;

        if (ReferenceEquals(hitCollider, ignored))
            return true;

        StiffBody? ignoredBody = ignored.Body;
        if (ignoredBody != null && ReferenceEquals(hitCollider.Body, ignoredBody))
            return true;

        LSCollider? hitTopParent = hitCollider.TopParent3D;
        LSCollider? ignoredTopParent = ignored.TopParent3D;
        return (hitTopParent != null && ReferenceEquals(hitTopParent, ignored))
            || (ignoredTopParent != null && ReferenceEquals(hitCollider, ignoredTopParent))
            || (hitTopParent != null && ignoredTopParent != null && ReferenceEquals(hitTopParent, ignoredTopParent));
    }

    private bool IsIgnoredMixedContinuousCollisionTarget(LSCollider2D hitCollider)
    {
        LSCollider2D? ignored = _continuousCollisionHandoffIgnoredCollider2D;
        if (ignored == null)
            return false;

        if (ReferenceEquals(hitCollider, ignored))
            return true;

        StiffBody2D? ignoredBody = ignored.Body;
        if (ignoredBody != null && ReferenceEquals(hitCollider.Body, ignoredBody))
            return true;

        LSCollider2D? hitTopParent = hitCollider.TopParent2D;
        LSCollider2D? ignoredTopParent = ignored.TopParent2D;
        return (hitTopParent != null && ReferenceEquals(hitTopParent, ignored))
            || (ignoredTopParent != null && ReferenceEquals(hitCollider, ignoredTopParent))
            || (hitTopParent != null && ignoredTopParent != null && ReferenceEquals(hitTopParent, ignoredTopParent));
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
            StiffBody? targetBody = target3D?.Body;
            if (targetBody == null)
                return false;

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
            StiffBody2D? targetBody = target2D?.Body;
            if (targetBody == null)
                return false;

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
        StiffBody target,
        Vector3d normalForSource,
        Vector3d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        Vector3d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : Vector3d.Zero;
        if (normal == Vector3d.Zero)
            return false;

        Fixed64 inverseMassA = EffectiveInverseMass;
        Fixed64 inverseMassB = target.EffectiveInverseMass;
        Fixed64 inverseMass = inverseMassA + inverseMassB;
        if (inverseMass <= Fixed64.Epsilon)
            return false;

        Fixed64 normalVelocity = Vector3d.Dot(_linearVelocity - target.ResolveContinuousCollisionFrameVelocity(), normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / inverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return false;

        Vector3d impulse = normal * impulseScalar;
        ApplyCollisionLinearVelocityDelta(impulse * inverseMassA);
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        target.ApplyContinuousCollisionHandoff(targetPositionAtImpact, -impulse * inverseMassB, remainingTime);
        return true;
    }

    private bool TryApplyContinuousCollisionMixed2DResponse(
        StiffBody2D target,
        Vector3d normalForSource,
        Vector3d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        Vector3d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : Vector3d.Zero;
        Vector2d planarNormal = normal.ToVector2d();
        if (normal == Vector3d.Zero || planarNormal == Vector2d.Zero)
            return false;

        Fixed64 inverseMassA = EffectiveInverseMass;
        Fixed64 inverseMassB = target.EffectiveInverseMass;
        Fixed64 planarScaleSquared = planarNormal.MagnitudeSquared;
        Fixed64 inverseMass = inverseMassA + inverseMassB * planarScaleSquared;
        if (inverseMass <= Fixed64.Epsilon)
            return false;

        Vector3d targetVelocity = target.ResolveContinuousCollisionFrameVelocity().ToVector3d(Fixed64.Zero);
        Fixed64 normalVelocity = Vector3d.Dot(_linearVelocity - targetVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / inverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return false;

        ApplyCollisionLinearVelocityDelta(normal * (impulseScalar * inverseMassA));
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -planarNormal * (impulseScalar * inverseMassB),
            remainingTime);
        return true;
    }

    private void UpdateContinuousCollisionFrameTrajectory(
        Vector3d positionAtElapsedTime,
        Vector3d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        if (deltaTime <= Fixed64.Epsilon)
            return;

        Fixed64 elapsedFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Vector3d frameDisplacement = velocity * deltaTime;
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameDisplacement = frameDisplacement;
        _continuousCollisionFrameStart = positionAtElapsedTime - frameDisplacement * elapsedFraction;
        _continuousCollisionFrameRotation = Rotation;
    }

    private Fixed64 ResolveContinuousCollisionFrameFraction(Fixed64 hitElapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        return deltaTime > Fixed64.Epsilon
            ? FixedMath.Clamp01(hitElapsedTime / deltaTime)
            : Fixed64.One;
    }

    private Fixed64 ResolveContinuousCollisionRestitution(StiffBody target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= CollisionResponse.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return FixedMath.Clamp(
            FixedMath.Min(RestitutionCoefficient, target.RestitutionCoefficient),
            Fixed64.Zero,
            Fixed64.One);
    }

    private Fixed64 ResolveContinuousCollisionRestitution(StiffBody2D target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= CollisionResponse.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return FixedMath.Clamp(
            FixedMath.Min(RestitutionCoefficient, target.RestitutionCoefficient),
            Fixed64.Zero,
            Fixed64.One);
    }

    private void RemoveClosingContinuousCollisionVelocity(Vector3d normal)
    {
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return;

        Fixed64 closingSpeed = Vector3d.Dot(_linearVelocity, normal);
        if (closingSpeed >= Fixed64.Zero)
            return;

        Vector3d lastVelocity = _linearVelocity;
        _linearVelocity -= normal * closingSpeed;
        RefreshLinearMotionState(lastVelocity);
        Context.Diagnostics.EmitLinearVelocityDelta(this, lastVelocity, _linearVelocity);
    }

    private bool TryResolveRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation)
    {
        if (!CanRotate || !ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDistance = _angularSpeed * Context.DeltaTime;
        if (angularDistance <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * proxyRadius;
        if (proxyRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= proxyRadius))
        {
            return false;
        }

        Vector3d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query3D.OverlapSphereAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDistance);
        if (stepCount <= 0)
            return false;

        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        try
        {
            for (int step = 1; step <= stepCount; step++)
            {
                Fixed64 lowerTime = (Fixed64)(step - 1) / (Fixed64)stepCount;
                Fixed64 sampleTime = (Fixed64)step / (Fixed64)stepCount;
                bool foundSampleHit = false;
                Fixed64 bestSafeTime = Fixed64.Zero;
                int bestTargetId = int.MaxValue;
                Vector3d bestContactNormal = Vector3d.Zero;

                for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
                {
                    SampleDynamicRotationalContinuousPose(startPosition, displacement, startRotation, sampleTime);
                    LSCollider? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Vector3d contactNormal))
                        continue;

                    LSCollider targetCollider = target!;
                    Fixed64 safeTime = RefineDynamicRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        lowerTime,
                        sampleTime,
                        contactNormal,
                        out Vector3d refinedNormal);
                    if (!ShouldReplaceRotationalContinuousCollisionHit(
                            safeTime,
                            targetCollider.Id,
                            foundSampleHit,
                            bestSafeTime,
                            bestTargetId))
                    {
                        continue;
                    }

                    foundSampleHit = true;
                    bestSafeTime = safeTime;
                    bestTargetId = targetCollider.Id;
                    bestContactNormal = refinedNormal;
                }

                if (!foundSampleHit)
                    continue;

                proposedPosition = startPosition + displacement * bestSafeTime;
                proposedRotation = IntegrateAngularRotation(startRotation, Context.DeltaTime * bestSafeTime);
                StopRotationalContinuousCollision(bestContactNormal);
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
        }

        return false;
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDistance = ResolveKinematicAngularDistanceRadians(startRotation, proposedRotation);
        if (angularDistance <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * proxyRadius;
        if (proxyRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= proxyRadius))
        {
            return false;
        }

        Vector3d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query3D.OverlapSphereAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDistance);
        if (stepCount <= 0)
            return false;

        FixedQuaternion targetRotation = proposedRotation;
        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        try
        {
            for (int step = 1; step <= stepCount; step++)
            {
                Fixed64 lowerTime = (Fixed64)(step - 1) / (Fixed64)stepCount;
                Fixed64 sampleTime = (Fixed64)step / (Fixed64)stepCount;
                bool foundSampleHit = false;
                Fixed64 bestSafeTime = Fixed64.Zero;
                int bestTargetId = int.MaxValue;

                for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
                {
                    SampleKinematicRotationalContinuousPose(startPosition, displacement, startRotation, targetRotation, sampleTime);
                    LSCollider? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Vector3d contactNormal))
                        continue;

                    LSCollider targetCollider = target!;
                    Fixed64 safeTime = RefineKinematicRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        targetRotation,
                        lowerTime,
                        sampleTime,
                        contactNormal,
                        out _);
                    if (!ShouldReplaceRotationalContinuousCollisionHit(
                            safeTime,
                            targetCollider.Id,
                            foundSampleHit,
                            bestSafeTime,
                            bestTargetId))
                    {
                        continue;
                    }

                    foundSampleHit = true;
                    bestSafeTime = safeTime;
                    bestTargetId = targetCollider.Id;
                }

                if (!foundSampleHit)
                    continue;

                proposedPosition = startPosition + displacement * bestSafeTime;
                proposedRotation = FixedQuaternion.Slerp(startRotation, targetRotation, bestSafeTime).Normalized;
                LastContinuousCollisionToiIterationCount++;
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
        }

        return false;
    }

    private void SampleDynamicRotationalContinuousPose(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        Fixed64 sampleTime)
    {
        Position3d = startPosition + displacement * sampleTime;
        Rotation = IntegrateAngularRotation(startRotation, Context.DeltaTime * sampleTime);
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

    private Fixed64 RefineDynamicRotationalContinuousCollisionSafeTime(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        Fixed64 lowerTime,
        Fixed64 upperTime,
        Vector3d upperContactNormal,
        out Vector3d contactNormal)
    {
        Fixed64 safeTime = lowerTime;
        Fixed64 hitTime = upperTime;
        contactNormal = upperContactNormal;

        for (int iteration = 0; iteration < ContinuousCollisionMath.RotationalToiRefinementIterations; iteration++)
        {
            Fixed64 sampleTime = (safeTime + hitTime) * Fixed64.Half;
            SampleDynamicRotationalContinuousPose(startPosition, displacement, startRotation, sampleTime);
            if (TrySampleRotationalContinuousCollision(target, out Vector3d sampleContactNormal))
            {
                hitTime = sampleTime;
                contactNormal = sampleContactNormal;
            }
            else
            {
                safeTime = sampleTime;
            }
        }

        return safeTime;
    }

    private Fixed64 RefineKinematicRotationalContinuousCollisionSafeTime(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 lowerTime,
        Fixed64 upperTime,
        Vector3d upperContactNormal,
        out Vector3d contactNormal)
    {
        Fixed64 safeTime = lowerTime;
        Fixed64 hitTime = upperTime;
        contactNormal = upperContactNormal;

        for (int iteration = 0; iteration < ContinuousCollisionMath.RotationalToiRefinementIterations; iteration++)
        {
            Fixed64 sampleTime = (safeTime + hitTime) * Fixed64.Half;
            SampleKinematicRotationalContinuousPose(startPosition, displacement, startRotation, targetRotation, sampleTime);
            if (TrySampleRotationalContinuousCollision(target, out Vector3d sampleContactNormal))
            {
                hitTime = sampleTime;
                contactNormal = sampleContactNormal;
            }
            else
            {
                safeTime = sampleTime;
            }
        }

        return safeTime;
    }

    private static bool ShouldReplaceRotationalContinuousCollisionHit(
        Fixed64 candidateSafeTime,
        int candidateTargetId,
        bool hasCurrent,
        Fixed64 currentSafeTime,
        int currentTargetId)
    {
        if (!hasCurrent)
            return true;

        int timeCompare = candidateSafeTime.CompareTo(currentSafeTime);
        if (timeCompare != 0)
            return timeCompare < 0;

        return candidateTargetId < currentTargetId;
    }

    private static Fixed64 ResolveKinematicAngularDistanceRadians(FixedQuaternion startRotation, FixedQuaternion proposedRotation)
    {
        Fixed64 angleDegrees = FixedQuaternion.Angle(startRotation, proposedRotation) * (Fixed64)2;
        return FixedMath.DegToRad(angleDegrees.Abs());
    }

    private bool TrySampleRotationalContinuousCollision(LSCollider? target, out Vector3d contactNormal)
    {
        contactNormal = Vector3d.Zero;
        if (!IsValidContinuousCollisionTarget(target))
            return false;

        OrderRotationalContinuousCollisionPair(target!, out LSCollider colliderA, out LSCollider colliderB, out bool sourceIsA);
        CollisionType collisionType = ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape);
        if (collisionType == CollisionType.None)
            return false;

        _rotationalContinuousCollisionManifold.BeginUpdate(Context.FrameCount);
        var workItem = new CollisionWorkItem(Context, colliderA, colliderB, collisionType, _rotationalContinuousCollisionManifold);
        if (!CollisionDetection.DoCollisionCheck(workItem) || !_rotationalContinuousCollisionManifold.HasContact)
            return false;

        contactNormal = _rotationalContinuousCollisionManifold.PrimaryContact.Normal;
        if (!sourceIsA)
            contactNormal = -contactNormal;

        return true;
    }

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
    {
        FixedQuaternion angularVelocityQuaternion = new(_angularVelocity.X, _angularVelocity.Y, _angularVelocity.Z, Fixed64.Zero);
        FixedQuaternion spin = angularVelocityQuaternion * startRotation * Fixed64.Half * deltaTime;
        return (startRotation + spin).Normalized;
    }

    private void StopRotationalContinuousCollision(Vector3d contactNormal)
    {
        Vector3d lastVelocity = _angularVelocity;
        _angularVelocity = Vector3d.Zero;
        _angularDirection = Vector3d.Zero;
        _angularAccelerationStore = Vector3d.Zero;
        _angularAcceleration = Vector3d.Zero;
        _deltaTorque = Vector3d.Zero;
        RefreshAngularMotionState(lastVelocity);
        Context.Diagnostics.EmitAngularVelocityDelta(this, lastVelocity, _angularVelocity);
        RemoveClosingContinuousCollisionVelocity(contactNormal);
    }

}
