//=======================================================================
// SolidBody.ContinuousCollision.Kinematic.cs
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

public partial class SolidBody
{
    private void CaptureKinematicContinuousCollisionFrame(
        Vector3d startPosition,
        Vector3d targetPosition,
        FixedQuaternion startRotation)
    {
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        _continuousCollisionFrameStart = startPosition;
        _continuousCollisionFrameDisplacement = ProjectLinearMotion(targetPosition - startPosition);
        _continuousCollisionFrameRotation = startRotation;
    }

    private bool TryResolveKinematicContinuousCollision(Vector3d startPosition, ref Vector3d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector3d displacement = ProjectLinearMotion(proposedPosition - startPosition);
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

            int hitCount = QueryStaticContinuousCollisionHits(
                startPosition,
                proposedPosition,
                proxyRadius,
                out bool staticHitsAreShapeExact);
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

            bool found3D = TryGetFirstValidContinuousCollisionHit(
                startPosition,
                proposedPosition,
                hitCount,
                staticHitsAreShapeExact,
                out Physics3DHit hit3D);
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
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out SolidBody target)
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
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out SolidBody2D target)
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
        SolidBody target,
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
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal);
        if (deltaTime <= Fixed64.Epsilon || constrainedInverseMass <= Fixed64.Epsilon)
            return false;

        Vector3d sourceVelocity = sourceDisplacement / deltaTime;
        Vector3d relativeVelocity = sourceVelocity - target.ResolveContinuousCollisionFrameVelocity();
        if (Vector3d.Dot(relativeVelocity, normal) > Fixed64.Zero)
            normal = -normal;

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / constrainedInverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return false;

        Fixed64 hitTime = sourceLength > Fixed64.Epsilon
            ? FixedMath.Clamp01(hitDistance / sourceLength)
            : Fixed64.Zero;
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -normal * (impulseScalar * target.EffectiveInverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider3D: Collider);
        return true;
    }

    private bool ApplyKinematicContinuousCollisionHandoff(
        SolidBody2D target,
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

        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(planarNormal) * planarNormal.MagnitudeSquared;
        if (constrainedInverseMass <= Fixed64.Epsilon)
            return false;

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);
        if (normalVelocity >= -Fixed64.Epsilon)
            return false;

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / constrainedInverseMass;
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

}
