//=======================================================================
// SolidBody2D.ContinuousCollision.Kinematic.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    private void CaptureKinematicContinuousCollisionFrame(Vector2d startPosition, Vector2d targetPosition, Fixed64 startRotation)
    {
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameStart = startPosition;
        _continuousCollisionFrameDisplacement = ProjectLinearMotion(targetPosition - startPosition);
        _continuousCollisionFrameRotation = startRotation;
    }

    private bool TryResolveKinematicContinuousCollision(Vector2d startPosition, ref Vector2d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector2d displacement = ProjectLinearMotion(proposedPosition - startPosition);
        proposedPosition = startPosition + displacement;
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
        bool pushed = false;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out SolidBody2D target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
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
                normalizedTime = FixedMath.Clamp01(exactHit.Distance / sourceLength);
            }
            else
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            if (distance > maxDistance)
                continue;

            Fixed64 frameFraction = FixedMath.Clamp01(distance / sourceLength);
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
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out SolidBody target)
                || !IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
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

            Fixed64 frameFraction = FixedMath.Clamp01(distance / sourceLength);
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
        SolidBody2D target,
        Vector2d sourceDisplacement,
        Vector2d normalForSource,
        Vector2d targetPositionAtImpact,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        Vector2d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon
                ? -sourceDisplacement.Normalized
                : Vector2d.Zero;
        if (normal == Vector2d.Zero)
            return false;

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal);
        if (constrainedInverseMass <= Fixed64.Epsilon)
            return false;

        Vector2d sourceVelocity = sourceDisplacement / deltaTime;
        Vector2d relativeVelocity = sourceVelocity - target.ResolveContinuousCollisionFrameVelocity();
        if (Vector2d.Dot(relativeVelocity, normal) > Fixed64.Zero)
            normal = -normal;

        Fixed64 normalVelocity = Vector2d.Dot(relativeVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / constrainedInverseMass;
        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -normal * (impulseScalar * target.EffectiveInverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider2D: Collider);
        return true;
    }

    private bool ApplyKinematicContinuousCollisionHandoff(
        SolidBody target,
        Vector2d sourceDisplacement,
        Vector2d normalForSource,
        Vector3d targetPositionAtImpact,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        Vector2d normal = normalForSource.MagnitudeSquared > Fixed64.Epsilon
            ? normalForSource.Normalized
            : sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon
                ? -sourceDisplacement.Normalized
                : Vector2d.Zero;
        if (normal == Vector2d.Zero)
            return false;

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal.ToVector3d(Fixed64.Zero));
        if (constrainedInverseMass <= Fixed64.Epsilon)
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
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / constrainedInverseMass;
        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -normal3D * (impulseScalar * target.EffectiveInverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider2D: Collider);
        return true;
    }

}
