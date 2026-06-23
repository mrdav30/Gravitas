//=======================================================================
// StiffBody2D.ContinuousCollision.cs
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

public sealed partial class StiffBody2D
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector2d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameStart;
    }

    internal Vector2d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameDisplacement;
    }

    internal Fixed64 ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameRotation;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        _continuousCollisionFrameToken = token;
        _continuousCollisionFrameStart = _position;
        _continuousCollisionFrameDisplacement = PredictContinuousCollisionDisplacement();
        _continuousCollisionFrameRotation = _rotation;
    }

    private Vector2d PredictContinuousCollisionDisplacement()
    {
        if (!CanTranslate || _isSleeping)
            return Vector2d.Zero;

        Fixed64 deltaTime = Context.DeltaTime;
        Vector2d predictedVelocity = _linearVelocity + (_deltaAcceleration + Gravity) * deltaTime;
        return predictedVelocity.MagnitudeSquared > Fixed64.Epsilon
            ? predictedVelocity * deltaTime
            : Vector2d.Zero;
    }

    private void CaptureKinematicContinuousCollisionFrame(Vector2d startPosition, Vector2d targetPosition, Fixed64 startRotation)
    {
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameStart = startPosition;
        _continuousCollisionFrameDisplacement = targetPosition - startPosition;
        _continuousCollisionFrameRotation = startRotation;
    }

    private bool TryResolveKinematicContinuousCollision(Vector2d startPosition, ref Vector2d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector2d displacement = proposedPosition - startPosition;
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
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        out Fixed64 distance)
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
            bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(startPosition, proposedPosition, mixedHitCount, out PhysicsMixedHit hitMixed);
            if (found2D && (!foundMixed || hit2D.Distance <= hitMixed.Distance))
            {
                distance = hit2D.Distance;
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
            _position = originalPosition;
            Collider.RebuildRuntimeShapeOnly();
        }

        distance = Fixed64.Zero;
        return false;
    }

    private bool TryApplyKinematicDynamicContinuousCollisionPushes(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        bool pushed = TryApplyKinematicDynamic2DContinuousCollisionPushes(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);
        pushed |= TryApplyKinematicDynamicMixed3DContinuousCollisionPushes(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);
        return pushed;
    }

    private bool TryApplyKinematicDynamic2DContinuousCollisionPushes(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        Vector2d sourceDisplacement = proposedPosition - startPosition;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        bool pushed = false;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out StiffBody2D target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeCircles(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    target.ContinuousCollisionFrameStart,
                    target.ContinuousCollisionFrameDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector2d normal,
                    out _))
            {
                continue;
            }

            Vector2d targetStart = target.ContinuousCollisionFrameStart;
            Vector2d targetDisplacement = target.ContinuousCollisionFrameDisplacement;
            if (TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    out Physics2DHit exactHit,
                    out _))
            {
                normal = exactHit.Normal;
                normalizedTime = sourceLength > Fixed64.Epsilon
                    ? FixedMath.Clamp01(exactHit.Distance / sourceLength)
                    : normalizedTime;
            }
            else
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            if (distance > maxDistance)
                continue;

            Fixed64 frameFraction = sourceLength > Fixed64.Epsilon
                ? FixedMath.Clamp01(distance / sourceLength)
                : Fixed64.Zero;
            Vector2d targetPositionAtImpact = targetStart + targetDisplacement * frameFraction;
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

    private bool TryApplyKinematicDynamicMixed3DContinuousCollisionPushes(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceStart = new(startPosition.X, Collider.MixedSlabCenterY, startPosition.Y);
        Vector3d sourceDisplacement = new(sourceDisplacement2D.X, Fixed64.Zero, sourceDisplacement2D.Y);
        Fixed64 sourceRadius = FixedMath.Max(proxyRadius, Collider.MixedHalfThickness);
        bool pushed = false;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(sourceStart, sourceDisplacement, sourceRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out StiffBody target)
                || !IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
                    target.ContinuousCollisionFrameStart,
                    target.ContinuousCollisionFrameDisplacement,
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
            Vector3d targetPositionAtImpact = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * frameFraction;
            if (ApplyKinematicContinuousCollisionHandoff(
                    target,
                    sourceDisplacement2D,
                    normalForSource.ToVector2d(),
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
        StiffBody2D target,
        Vector2d sourceDisplacement,
        Vector2d normalForSource,
        Vector2d targetPositionAtImpact,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        Vector2d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : -ResolveKinematicPushAxis(sourceDisplacement, sourceDisplacement);
        if (normal == Vector2d.Zero)
            return false;

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 inverseMass = target.EffectiveInverseMass;
        if (deltaTime <= Fixed64.Epsilon || inverseMass <= Fixed64.Epsilon)
            return false;

        Vector2d sourceVelocity = sourceDisplacement / deltaTime;
        Vector2d relativeVelocity = sourceVelocity - target.ResolveContinuousCollisionFrameVelocity();
        if (Vector2d.Dot(relativeVelocity, normal) > Fixed64.Zero)
            normal = -normal;

        Fixed64 normalVelocity = Vector2d.Dot(relativeVelocity, normal);
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
            ignoredCollider2D: Collider);
        return true;
    }

    private bool ApplyKinematicContinuousCollisionHandoff(
        StiffBody target,
        Vector2d sourceDisplacement,
        Vector2d normalForSource,
        Vector3d targetPositionAtImpact,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        Vector2d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : -ResolveKinematicPushAxis(sourceDisplacement, sourceDisplacement);
        if (normal == Vector2d.Zero)
            return false;

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 inverseMass = target.EffectiveInverseMass;
        if (deltaTime <= Fixed64.Epsilon || inverseMass <= Fixed64.Epsilon)
            return false;

        Vector3d normal3D = normal.ToVector3d(Fixed64.Zero);
        Vector3d sourceVelocity = (sourceDisplacement / deltaTime).ToVector3d(Fixed64.Zero);
        Vector3d relativeVelocity = sourceVelocity - target.ResolveContinuousCollisionFrameVelocity();
        if (Vector3d.Dot(relativeVelocity, normal3D) > Fixed64.Zero)
        {
            normal = -normal;
            normal3D = -normal3D;
        }

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal3D);
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
            -normal3D * (impulseScalar * inverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider2D: Collider);
        return true;
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
        Vector2d positionAtImpact,
        Vector2d velocityDelta,
        Fixed64 remainingTime,
        LSCollider? ignoredCollider3D = null,
        LSCollider2D? ignoredCollider2D = null)
    {
        if (!CanTranslate)
            return;

        _position = positionAtImpact;
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
        if (!CanTranslate || remainingTime <= Fixed64.Epsilon || _linearVelocity.MagnitudeSquared <= Fixed64.Epsilon)
        {
            _continuousCollisionHandoffIgnoredCollider3D = null;
            _continuousCollisionHandoffIgnoredCollider2D = null;
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

    internal void ApplyCollisionLinearVelocityDelta(Vector2d velocityDelta)
    {
        if (!CanTranslate || velocityDelta == Vector2d.Zero)
            return;

        WakeFromCollision();
        _linearVelocity += velocityDelta;
        RefreshLinearSpeed();
    }

    internal void ApplyCollisionAngularVelocityDelta(Fixed64 velocityDelta)
    {
        if (!CanRotate || velocityDelta == Fixed64.Zero)
            return;

        WakeFromCollision();
        _angularVelocity += velocityDelta;
        RefreshAngularSpeed();
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

        Vector2d displacement = proposedPosition - startPosition;
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && displacement.MagnitudeSquared <= proxyRadius * proxyRadius))
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
            Vector2d segmentDisplacement = _linearVelocity * remainingTime;
            Fixed64 segmentLength = segmentDisplacement.Magnitude;
            if (segmentLength <= Fixed64.Epsilon)
                break;

            Vector2d segmentEnd = currentPosition + segmentDisplacement;
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

    private bool TryGetFirstContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
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
                remainingFrameFraction,
                out Physics2DHit dynamicHit2D,
                out Fixed64 dynamicClosingSpeed2D);
            if (ShouldReplaceContinuousCollisionHit(dynamicHit2D, dynamicClosingSpeed2D, foundDynamic2D, found2D, hit2D, Fixed64.Zero))
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
                remainingFrameFraction,
                out PhysicsMixedHit dynamicHitMixed,
                out Fixed64 dynamicClosingSpeedMixed);
            if (ShouldReplaceMixedContinuousCollisionHit(dynamicHitMixed, dynamicClosingSpeedMixed, foundDynamicMixed, foundMixed, hitMixed, Fixed64.Zero))
            {
                hitMixed = dynamicHitMixed;
                foundMixed = true;
                hitMixedKind = ContinuousCollisionTargetKind.Dynamic3D;
            }

            if (found2D && (!foundMixed || hit2D.Distance <= hitMixed.Distance))
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

    private bool TryResolveRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation)
    {
        if (!CanRotate || !ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDelta = proposedRotation - startRotation;
        Fixed64 angularDistance = angularDelta.Abs();
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

        Vector2d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query2D.OverlapCircleAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDelta);
        if (stepCount <= 0)
            return false;

        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
        try
        {
            for (int step = 1; step <= stepCount; step++)
            {
                Fixed64 lowerTime = (Fixed64)(step - 1) / (Fixed64)stepCount;
                Fixed64 sampleTime = (Fixed64)step / (Fixed64)stepCount;
                bool foundSampleHit = false;
                Fixed64 bestSafeTime = Fixed64.Zero;
                int bestTargetId = int.MaxValue;
                Contact2D bestContact = default;

                for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
                {
                    SampleRotationalContinuousPose(startPosition, displacement, startRotation, angularDelta, sampleTime);
                    LSCollider2D? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Contact2D contact))
                        continue;

                    LSCollider2D targetCollider = target!;
                    Fixed64 safeTime = RefineRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        angularDelta,
                        lowerTime,
                        sampleTime,
                        contact,
                        out Contact2D refinedContact);
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
                    bestContact = refinedContact;
                }

                if (!foundSampleHit)
                    continue;

                proposedPosition = startPosition + displacement * bestSafeTime;
                proposedRotation = startRotation + angularDelta * bestSafeTime;
                StopRotationalContinuousCollision(bestContact.Normal);
                return true;
            }
        }
        finally
        {
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
        }

        return false;
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDelta = proposedRotation - startRotation;
        Fixed64 angularDistance = angularDelta.Abs();
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

        Vector2d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query2D.OverlapCircleAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDelta);
        if (stepCount <= 0)
            return false;

        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
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
                    SampleRotationalContinuousPose(startPosition, displacement, startRotation, angularDelta, sampleTime);
                    LSCollider2D? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Contact2D contact))
                        continue;

                    LSCollider2D targetCollider = target!;
                    Fixed64 safeTime = RefineRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        angularDelta,
                        lowerTime,
                        sampleTime,
                        contact,
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
                proposedRotation = startRotation + angularDelta * bestSafeTime;
                LastContinuousCollisionToiIterationCount++;
                return true;
            }
        }
        finally
        {
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
        }

        return false;
    }

    private void SampleRotationalContinuousPose(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 sampleTime)
    {
        _position = startPosition + displacement * sampleTime;
        _rotation = startRotation + angularDelta * sampleTime;
        Collider.RebuildRuntimeShapeOnly();
    }

    private bool TrySampleRotationalContinuousCollision(LSCollider2D? target, out Contact2D contact)
    {
        if (!IsValidContinuousCollisionTarget(target))
        {
            contact = default;
            return false;
        }

        return CollisionDetection2D.TryCollide(Collider, target!, out contact);
    }

    private Fixed64 RefineRotationalContinuousCollisionSafeTime(
        LSCollider2D target,
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 lowerTime,
        Fixed64 upperTime,
        Contact2D upperContact,
        out Contact2D contact)
    {
        Fixed64 safeTime = lowerTime;
        Fixed64 hitTime = upperTime;
        contact = upperContact;

        for (int iteration = 0; iteration < ContinuousCollisionMath.RotationalToiRefinementIterations; iteration++)
        {
            Fixed64 sampleTime = (safeTime + hitTime) * Fixed64.Half;
            SampleRotationalContinuousPose(startPosition, displacement, startRotation, angularDelta, sampleTime);
            if (TrySampleRotationalContinuousCollision(target, out Contact2D sampleContact))
            {
                hitTime = sampleTime;
                contact = sampleContact;
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
            if (!IsValidMixedContinuousCollisionHit(candidate)
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
        Fixed64 remainingFrameFraction,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector2d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        bool found = false;
        Physics2DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out StiffBody2D target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector2d targetStart = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector2d targetDisplacement = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeCircles(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out _,
                    out _,
                    out _))
            {
                continue;
            }

            if (!TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    out Physics2DHit candidate,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
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
        StiffBody2D target,
        Vector2d sourceStart,
        Vector2d sourceDisplacement,
        Vector2d targetStart,
        Vector2d targetDisplacement,
        Fixed64 sourceLength,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector2d relativeDisplacement = sourceDisplacement - targetDisplacement;
        Fixed64 relativeLength = relativeDisplacement.Magnitude;
        if (relativeLength <= Fixed64.Epsilon || sourceLength <= Fixed64.Epsilon)
            return false;

        Vector2d originalSourcePosition = _position;
        Fixed64 originalSourceRotation = _rotation;
        Vector2d originalTargetPosition = target._position;
        Fixed64 originalTargetRotation = target._rotation;
        try
        {
            _position = sourceStart;
            target._position = targetStart;
            target._rotation = target.ContinuousCollisionFrameRotation;
            Collider.RebuildRuntimeShapeOnly();
            target.Collider.RebuildRuntimeShapeOnly();

            if (!QueryDetection2D.TrySweepMoverShape(Collider, relativeDisplacement, target.Collider, out Physics2DHit relativeHit))
                return false;

            closingSpeed = -Vector2d.Dot(relativeDisplacement, relativeHit.Normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 normalizedTime = FixedMath.Clamp01(relativeHit.Distance / relativeLength);
            Vector2d point = relativeHit.Point + targetDisplacement * normalizedTime;
            hit = new Physics2DHit(
                target.Collider,
                point,
                relativeHit.Normal,
                sourceLength * normalizedTime);
            return true;
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
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
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

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement2D.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

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
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out StiffBody target)
                || !IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector3d targetStart = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector3d targetDisplacement = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
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
            Vector3d sourceCenter = sourceStart + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
            Vector3d point2D = sourceCenter - normalForSource * sourceRadius;
            Vector3d point3D = ResolveDynamicContactPoint(sourceCenter, targetCenter, normalForSource, targetRadius);
            var candidate = new PhysicsMixedHit(
                target.Collider,
                null,
                point3D,
                point2D,
                normalForSource,
                PhysicsQueryReducerKind.ConservativeFallback,
                distance,
                sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon ? sourceDisplacement.Normalized : Vector3d.Zero);
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

    private bool IsEligibleDynamicContinuousCollisionTarget(StiffBody2D target)
    {
        if (ReferenceEquals(target, this)
            || !target.Active
            || target.Immovable
            || target.IsKinematic
            || target.Collider.IsTrigger
            || !Context.Physics2D.RequireCollisionPair(Collider, target.Collider))
        {
            return false;
        }

        return true;
    }

    private bool IsEligibleDynamicMixed3DTarget(StiffBody target)
    {
        return target.Active
            && !target.Immovable
            && !target.IsKinematic
            && !target.Collider.IsTrigger
            && Context.MixedCollisions.RequireCollisionPair(target.Collider, Collider);
    }

    private static bool ShouldReplaceContinuousCollisionHit(
        Physics2DHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        Physics2DHit current,
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

        return candidate.Collider.Id < current.Collider.Id;
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
    private static Vector2d ResolveDynamicContactPoint(
        Vector2d sourceCenter,
        Vector2d targetCenter,
        Vector2d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector2d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
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

        StiffBody2D? parentBody = Collider.TopParent2D?.Body;
        if (parentBody != null && parentBody._continuousCollisionMode != ContinuousCollisionMode.Inherit)
            return parentBody._continuousCollisionMode;

        mode = Context.Settings.DefaultContinuousCollisionMode;
        return mode == ContinuousCollisionMode.Inherit
            ? ContinuousCollisionMode.Discrete
            : mode;
    }

    private Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        return Collider switch
        {
            LSCircleCollider2D circle => circle.ScaledRadius,
            LSAABBoxCollider2D box => box.ScaledHalfExtents.Magnitude,
            LSCompoundCollider2D compound => compound.ScaledRadius,
            _ => ResolveConvexContinuousCollisionProxyRadius()
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadiusForDynamicTarget()
    {
        return ResolveContinuousCollisionProxyRadius();
    }

    internal Vector2d ResolveContinuousCollisionFrameVelocity()
    {
        Fixed64 deltaTime = Context.DeltaTime;
        return deltaTime > Fixed64.Epsilon
            ? _continuousCollisionFrameDisplacement / deltaTime
            : Vector2d.Zero;
    }

    private Fixed64 ResolveConvexContinuousCollisionProxyRadius()
    {
        int vertexCount = Collider.VertexCount;
        if (vertexCount <= 0)
            return Fixed64.Zero;

        Vector2d center = Collider.Center;
        Fixed64 bestDistanceSquared = Fixed64.Zero;
        for (int i = 0; i < vertexCount; i++)
        {
            Fixed64 distanceSquared = Vector2d.DistanceSquared(center, Collider.GetVertexUnchecked(i));
            if (distanceSquared > bestDistanceSquared)
                bestDistanceSquared = distanceSquared;
        }

        return bestDistanceSquared > Fixed64.Zero
            ? FixedMath.Sqrt(bestDistanceSquared)
            : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsValidContinuousCollisionHit(Physics2DHit hit) =>
        IsValidContinuousCollisionTarget(hit.Collider);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClosingContinuousCollisionHit(Vector2d displacement, Vector2d normal) =>
        normal.MagnitudeSquared > Fixed64.Epsilon
        && Vector2d.Dot(displacement, normal) < -Fixed64.Epsilon;

    private bool IsValidContinuousCollisionTarget(LSCollider2D? hitCollider)
    {
        if (hitCollider == null
            || ReferenceEquals(hitCollider, Collider)
            || IsIgnoredContinuousCollisionTarget(hitCollider)
            || hitCollider.IsTrigger
            || !Context.Physics2D.RequireCollisionPair(Collider, hitCollider))
        {
            return false;
        }

        StiffBody2D? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private bool IsValidMixedContinuousCollisionHit(PhysicsMixedHit hit)
    {
        LSCollider? hitCollider = hit.Collider3D;
        if (hitCollider == null
            || IsIgnoredMixedContinuousCollisionTarget(hitCollider)
            || hitCollider.IsTrigger
            || !Context.MixedCollisions.RequireCollisionPair(hitCollider, Collider))
        {
            return false;
        }

        StiffBody? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private bool IsIgnoredContinuousCollisionTarget(LSCollider2D hitCollider)
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

    private bool IsIgnoredMixedContinuousCollisionTarget(LSCollider hitCollider)
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
            StiffBody2D? targetBody = target2D?.Body;
            if (targetBody == null)
                return false;

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
            StiffBody? targetBody = target3D?.Body;
            if (targetBody == null)
                return false;

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
        StiffBody2D target,
        Vector2d normalForSource,
        Vector2d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        Vector2d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : Vector2d.Zero;
        if (normal == Vector2d.Zero)
            return false;

        Fixed64 inverseMassA = EffectiveInverseMass;
        Fixed64 inverseMassB = target.EffectiveInverseMass;
        Fixed64 inverseMass = inverseMassA + inverseMassB;
        if (inverseMass <= Fixed64.Epsilon)
            return false;

        Fixed64 normalVelocity = Vector2d.Dot(_linearVelocity - target.ResolveContinuousCollisionFrameVelocity(), normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / inverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return false;

        Vector2d impulse = normal * impulseScalar;
        ApplyCollisionLinearVelocityDelta(impulse * inverseMassA);
        UpdateContinuousCollisionFrameTrajectory(sourcePositionAtImpact, _linearVelocity, hitElapsedTime);
        target.ApplyContinuousCollisionHandoff(targetPositionAtImpact, -impulse * inverseMassB, remainingTime);
        return true;
    }

    private bool TryApplyContinuousCollisionMixed3DResponse(
        StiffBody target,
        Vector2d normalForSource,
        Vector2d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        Vector2d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : Vector2d.Zero;
        if (normal == Vector2d.Zero)
            return false;

        Vector3d normal3D = normal.ToVector3d(Fixed64.Zero);
        Fixed64 inverseMassA = EffectiveInverseMass;
        Fixed64 inverseMassB = target.EffectiveInverseMass;
        Fixed64 inverseMass = inverseMassA + inverseMassB;
        if (inverseMass <= Fixed64.Epsilon)
            return false;

        Vector3d targetVelocity = target.ResolveContinuousCollisionFrameVelocity();
        Fixed64 normalVelocity = Vector3d.Dot(_linearVelocity.ToVector3d(Fixed64.Zero) - targetVelocity, normal3D);
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
            -normal3D * (impulseScalar * inverseMassB),
            remainingTime);
        return true;
    }

    private void UpdateContinuousCollisionFrameTrajectory(
        Vector2d positionAtElapsedTime,
        Vector2d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        if (deltaTime <= Fixed64.Epsilon)
            return;

        Fixed64 elapsedFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Vector2d frameDisplacement = velocity * deltaTime;
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameDisplacement = frameDisplacement;
        _continuousCollisionFrameStart = positionAtElapsedTime - frameDisplacement * elapsedFraction;
        _continuousCollisionFrameRotation = _rotation;
    }

    private Fixed64 ResolveContinuousCollisionFrameFraction(Fixed64 hitElapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        return deltaTime > Fixed64.Epsilon
            ? FixedMath.Clamp01(hitElapsedTime / deltaTime)
            : Fixed64.One;
    }

    private Fixed64 ResolveContinuousCollisionRestitution(StiffBody2D target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= CollisionResponse2D.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return FixedMath.Clamp(
            FixedMath.Min(RestitutionCoefficient, target.RestitutionCoefficient),
            Fixed64.Zero,
            Fixed64.One);
    }

    private Fixed64 ResolveContinuousCollisionRestitution(StiffBody target, Fixed64 closingSpeed)
    {
        if (closingSpeed <= CollisionResponse2D.RestitutionVelocityThreshold)
            return Fixed64.Zero;

        return FixedMath.Clamp(
            FixedMath.Min(RestitutionCoefficient, target.RestitutionCoefficient),
            Fixed64.Zero,
            Fixed64.One);
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

    private void StopRotationalContinuousCollision(Vector2d contactNormal)
    {
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        RefreshAngularSpeed();
        RemoveClosingContinuousCollisionVelocity(contactNormal);
    }

}
