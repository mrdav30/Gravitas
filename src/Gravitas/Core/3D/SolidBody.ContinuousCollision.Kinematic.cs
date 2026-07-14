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
            || (mode == ContinuousCollisionMode.Auto
                && ContinuousCollisionMath.IsWithinProxyRadius(displacement, displacementMagnitudeSquared, proxyRadius)))
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
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
            if (!ContinuousCollisionMath.TrySweepRelativeSpheres(
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
                normalizedTime = FixedMath.Clamp01(exactHit.Distance / sourceLength);
            }
            else if (exactSupported)
                continue;

            Fixed64 distance = sourceLength * normalizedTime;
            if (distance > maxDistance)
                continue;

            Fixed64 frameFraction = FixedMath.Clamp01(distance / sourceLength);
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
                target.ResolveContinuousCollisionProxyRadius(),
                target.Collider.MixedHalfThickness);
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

            Fixed64 frameFraction = FixedMath.Clamp01(distance / sourceLength);
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
        _ = ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(normalForSource, sourceDisplacement, out Vector3d normal);

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal);
        if (constrainedInverseMass <= Fixed64.Zero)
            return false;
        if (!ContinuousCollisionImpulsePolicy.IsResolvableMobility(target.EffectiveInverseMass, constrainedInverseMass))
            return false;

        Vector3d sourceVelocity = sourceDisplacement / deltaTime;
        Vector3d relativeVelocity = sourceVelocity - target.ResolveContinuousCollisionFrameVelocity();
        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 responseSpeed = -(Fixed64.One + restitution) * normalVelocity;
        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -ContinuousCollisionImpulsePolicy.ResolveVelocityDelta(
                normal,
                responseSpeed,
                target.EffectiveInverseMass,
                constrainedInverseMass),
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
        _ = ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(normalForSource, sourceDisplacement, out Vector3d normal);

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 inverseMass = target.EffectiveInverseMass;

        Vector3d sourceVelocity = sourceDisplacement / deltaTime;
        Vector3d targetVelocity = target.ResolveContinuousCollisionFrameVelocity().ToVector3d(Fixed64.Zero);
        Vector3d relativeVelocity = sourceVelocity - targetVelocity;

        Vector2d planarNormal = normal.ToVector2d();
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(planarNormal) * planarNormal.MagnitudeSquared;
        if (constrainedInverseMass <= Fixed64.Zero)
            return false;
        if (!ContinuousCollisionImpulsePolicy.IsResolvableMobility(inverseMass, constrainedInverseMass))
            return false;

        Fixed64 normalVelocity = Vector3d.Dot(relativeVelocity, normal);

        Fixed64 restitution = ResolveContinuousCollisionRestitution(target, -normalVelocity);
        Fixed64 responseSpeed = -(Fixed64.One + restitution) * normalVelocity;
        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        target.ApplyContinuousCollisionHandoff(
            targetPositionAtImpact,
            -ContinuousCollisionImpulsePolicy.ResolveVelocityDelta(
                planarNormal,
                responseSpeed,
                inverseMass,
                constrainedInverseMass),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider3D: Collider);
        return true;
    }

}
