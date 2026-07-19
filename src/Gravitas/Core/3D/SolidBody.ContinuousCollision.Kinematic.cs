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
    private bool TryResolveKinematicContinuousCollision(Vector3d startPosition, ref Vector3d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector3d displacement = ContinuousCollisionSweepRange.ValidateEndpoint(
            startPosition,
            proposedPosition,
            out Fixed64 sourceLength);
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
            proposedPosition = startPosition + displacement.Normalized * staticHitDistance;

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
        _continuousCollisionHits.FastClear();
        _continuousMixedCollisionHits.FastClear();
        GatherKinematicDynamic3DContinuousCollisionHits(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);
        GatherKinematicDynamicMixed2DContinuousCollisionHits(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);

        Physics3DHitSorter.SortByDistance(_continuousCollisionHits);
        PhysicsMixedHitSorter.SortByDistance(_continuousMixedCollisionHits);
        Vector3d sourceDisplacement = proposedPosition - startPosition;
        int hit3DIndex = 0;
        int hit2DIndex = 0;
        bool pushed = false;
        while (hit3DIndex < _continuousCollisionHits.Count
            || hit2DIndex < _continuousMixedCollisionHits.Count)
        {
            bool take2D = hit3DIndex >= _continuousCollisionHits.Count
                || (hit2DIndex < _continuousMixedCollisionHits.Count
                    && _continuousMixedCollisionHits[hit2DIndex].Distance
                        <= _continuousCollisionHits[hit3DIndex].Distance);
            if (take2D)
            {
                PhysicsMixedHit hit = _continuousMixedCollisionHits[hit2DIndex++];
                pushed |= ApplyKinematicContinuousCollisionHandoff(
                    hit.Body2D!,
                    sourceDisplacement,
                    hit.NormalFor3DSource,
                    hit.Distance,
                    sourceLength);

                continue;
            }

            Physics3DHit hit3D = _continuousCollisionHits[hit3DIndex++];
            pushed |= ApplyKinematicContinuousCollisionHandoff(
                hit3D.Body!,
                sourceDisplacement,
                hit3D.Normal,
                hit3D.Distance,
                sourceLength);
        }

        return pushed;
    }

    private void GatherKinematicDynamic3DContinuousCollisionHits(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        Vector3d sourceDisplacement = proposedPosition - startPosition;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            SolidBody target = Context.Physics.GetContinuousCollisionCandidate(dynamicId);
            if (!IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            target.TrySampleContinuousCollisionDisplacement(
                Fixed64.Zero,
                Fixed64.One,
                out Vector3d targetStart,
                out Vector3d targetDisplacement);
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

            Physics3DHit hit;
            if (TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    Fixed64.Zero,
                    out Physics3DHit exactHit,
                    out _,
                    out bool exactSupported))
            {
                hit = exactHit;
            }
            else if (exactSupported)
                continue;
            else
            {
                Fixed64 distance = sourceLength * normalizedTime;
                hit = new Physics3DHit(
                    target.Collider,
                    Vector3d.Zero,
                    normal,
                    distance,
                    sourceDisplacement.Normalized);
            }

            if (hit.Distance > maxDistance)
                continue;

            _continuousCollisionHits.Add(hit);
        }
    }

    private void GatherKinematicDynamicMixed2DContinuousCollisionHits(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryMixedContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            SolidBody2D target = Context.Physics2D.GetContinuousCollisionCandidate(dynamicId);
            if (!IsEligibleDynamicMixed2DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = FixedMath.Max(
                target.ResolveContinuousCollisionProxyRadius(),
                target.Collider.MixedHalfThickness);
            target.TrySampleContinuousCollisionDisplacement(
                Fixed64.Zero,
                Fixed64.One,
                out Vector2d targetStart2D,
                out Vector2d targetDisplacement2D);
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

            _continuousMixedCollisionHits.Add(new PhysicsMixedHit(
                Collider,
                target.Collider,
                Vector3d.Zero,
                Vector3d.Zero,
                -normalForSource,
                PhysicsQueryReducerKind.ConservativeFallback,
                distance,
                sourceDisplacement.Normalized));
        }
    }

    internal bool ApplyKinematicContinuousCollisionHandoff(
        SolidBody target,
        Vector3d sourceDisplacement,
        Vector3d normalForSource,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        _ = ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(normalForSource, sourceDisplacement, out Vector3d normal);

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal);
        if (constrainedInverseMass <= Fixed64.Zero)
            return false;

        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        Vector3d sourceVelocity = SampleContinuousCollisionLinearVelocity(hitTime);
        Vector3d targetVelocity = target.SampleContinuousCollisionLinearVelocity(hitTime);
        bool relativeVelocityResolved = Vector3d.TrySubtract(
            sourceVelocity,
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
        Vector3d targetResponseNormal = target.ProjectLinearMotion(-normal);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            target.EffectiveInverseMass,
            constrainedInverseMass,
            out Vector3d targetVelocityDelta);
        bool targetVelocityResolved = Vector3d.TryAdd(
            targetVelocity,
            target.ProjectLinearMotion(targetVelocityDelta),
            out Vector3d targetPostVelocity);
        if (!(relativeVelocityResolved
            & normalVelocity < -Fixed64.Epsilon
            & responseFactorResolved
            & closingSpeedResolved
            & responseSpeedResolved
            & targetDeltaResolved
            & targetVelocityResolved))
        {
            return false;
        }

        return target.ApplyContinuousCollisionHandoff(
            target.SampleContinuousCollisionPosition(hitTime),
            target.SampleContinuousCollisionRotation(hitTime),
            targetPostVelocity,
            target.SampleContinuousCollisionAngularVelocity(hitTime),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider3D: Collider);
    }

    internal bool ApplyKinematicContinuousCollisionHandoff(
        SolidBody2D target,
        Vector3d sourceDisplacement,
        Vector3d normalForSource,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        _ = ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(normalForSource, sourceDisplacement, out Vector3d normal);

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 inverseMass = target.EffectiveInverseMass;

        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        Vector3d sourceVelocity = SampleContinuousCollisionLinearVelocity(hitTime);
        Vector2d targetLinearVelocity = target.SampleContinuousCollisionLinearVelocity(hitTime);
        Vector3d targetVelocity = targetLinearVelocity.ToVector3d(Fixed64.Zero);
        bool relativeVelocityResolved = Vector3d.TrySubtract(
            sourceVelocity,
            targetVelocity,
            out Vector3d relativeVelocity);

        Vector2d planarNormal = normal.ToVector2d();
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(planarNormal) * planarNormal.MagnitudeSquared;
        if (constrainedInverseMass <= Fixed64.Zero)
            return false;

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
        Vector2d targetResponseNormal = target.ProjectLinearMotion(-planarNormal);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            inverseMass,
            constrainedInverseMass,
            out Vector2d targetVelocityDelta);
        bool targetVelocityResolved = Vector2d.TryAdd(
            targetLinearVelocity,
            target.ProjectLinearMotion(targetVelocityDelta),
            out Vector2d targetPostVelocity);
        if (!(relativeVelocityResolved
            & normalVelocity < -Fixed64.Epsilon
            & responseFactorResolved
            & closingSpeedResolved
            & responseSpeedResolved
            & targetDeltaResolved
            & targetVelocityResolved))
        {
            return false;
        }

        return target.ApplyContinuousCollisionHandoffState(
            target.SampleContinuousCollisionPosition(hitTime),
            target.SampleContinuousCollisionRotation(hitTime),
            targetPostVelocity,
            target.SampleContinuousCollisionAngularVelocity(hitTime),
            deltaTime * (Fixed64.One - hitTime),
            ignoredCollider3D: Collider);
    }

}
