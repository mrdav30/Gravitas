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
    private bool TryResolveKinematicContinuousCollision(Vector2d startPosition, ref Vector2d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector2d displacement = ContinuousCollisionSweepRange.ValidateEndpoint(
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
            sourceLength,
            out bool hasUnresolvedMixedLimit,
            out Fixed64 unresolvedMixedDistance);
        staticHitDistance = FixedMath.Min(
            dynamicPushLimit,
            unresolvedMixedDistance);
        foundStatic |= hasUnresolvedMixedLimit;
        if (!foundStatic && !pushedDynamic)
            return false;

        LastContinuousCollisionToiIterationCount++;
        if (foundStatic)
            proposedPosition = startPosition + displacement.Normalized * staticHitDistance;

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
            if (found2D
                && (!foundMixed
                    || ContinuousCollisionCandidateOrdering.Is2DHitFirst(
                        hit2D.Distance,
                        hitMixed.Distance)))
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
        Fixed64 sourceLength,
        out bool hasUnresolvedMixedLimit,
        out Fixed64 unresolvedMixedDistance)
    {
        hasUnresolvedMixedLimit = false;
        unresolvedMixedDistance = maxDistance;
        _continuousCollisionHits.FastClear();
        _continuousMixedCollisionHits.FastClear();
        GatherKinematicDynamic2DContinuousCollisionHits(
            startPosition,
            proposedPosition,
            proxyRadius,
            maxDistance,
            sourceLength);
        GatherKinematicDynamicMixed3DContinuousCollisionHits(
            startPosition,
            proposedPosition,
            maxDistance,
            sourceLength,
            ref hasUnresolvedMixedLimit,
            ref unresolvedMixedDistance);

        Physics2DHitSorter.SortByDistance(_continuousCollisionHits);
        PhysicsMixedHitSorter.SortByDistance(_continuousMixedCollisionHits);
        Vector2d sourceDisplacement = proposedPosition - startPosition;
        int hit2DIndex = 0;
        int hit3DIndex = 0;
        bool pushed = false;
        while (hit2DIndex < _continuousCollisionHits.Count
            || hit3DIndex < _continuousMixedCollisionHits.Count)
        {
            bool take2D = hit3DIndex >= _continuousMixedCollisionHits.Count
                || (hit2DIndex < _continuousCollisionHits.Count
                    && _continuousCollisionHits[hit2DIndex].Distance
                        <= _continuousMixedCollisionHits[hit3DIndex].Distance);
            if (take2D)
            {
                Physics2DHit hit = _continuousCollisionHits[hit2DIndex++];
                pushed |= ApplyKinematicContinuousCollisionHandoff(
                    hit.Body!,
                    sourceDisplacement,
                    hit.Normal,
                    hit.Distance,
                    sourceLength);

                continue;
            }

            PhysicsMixedHit hit3D = _continuousMixedCollisionHits[hit3DIndex++];
            pushed |= ApplyKinematicContinuousCollisionHandoff(
                hit3D.Body3D!,
                sourceDisplacement,
                hit3D.NormalFor2DSource,
                hit3D.Distance,
                sourceLength);
        }

        return pushed;
    }

    private void GatherKinematicDynamic2DContinuousCollisionHits(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 maxDistance,
        Fixed64 sourceLength)
    {
        Vector2d sourceDisplacement = proposedPosition - startPosition;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            SolidBody2D target = Context.Physics2D.GetContinuousCollisionCandidate(dynamicId);
            if (!IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            if (!TryGetDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    sourceLength,
                    Fixed64.Zero,
                    out Physics2DHit exactHit,
                    out _))
            {
                continue;
            }

            if (exactHit.Distance > maxDistance)
                continue;

            _continuousCollisionHits.Add(exactHit);
        }
    }

    private void GatherKinematicDynamicMixed3DContinuousCollisionHits(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 maxDistance,
        Fixed64 sourceLength,
        ref bool hasUnresolvedMixedLimit,
        ref Fixed64 unresolvedMixedDistance)
    {
        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return;

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        Vector3d sourceStart = new(startPosition.X, Collider.MixedSlabCenterY, startPosition.Y);
        Vector3d sourceDisplacement = new(sourceDisplacement2D.X, Fixed64.Zero, sourceDisplacement2D.Y);
        Fixed64 sourceRadius = ResolveMixedContinuousCollisionProxyRadius();
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(sourceStart, sourceDisplacement, sourceRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            SolidBody target = Context.Physics.GetContinuousCollisionCandidate(dynamicId);
            if (!IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            ContinuousCollisionMath.IntervalSearchStatus status =
                TryGetDynamicMixed3DContinuousCollisionHit(
                    target,
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
                    sourceLength,
                    Fixed64.Zero,
                    out DynamicMixedIntervalHit candidate);
            if (status
                    == ContinuousCollisionMath.IntervalSearchStatus.CertifiedNoHit
                || candidate.SafeDistance > maxDistance)
            {
                continue;
            }

            if (status
                == ContinuousCollisionMath.IntervalSearchStatus.Unresolved)
            {
                hasUnresolvedMixedLimit = true;
                unresolvedMixedDistance = FixedMath.Min(
                    unresolvedMixedDistance,
                    candidate.SafeDistance);
                continue;
            }

            _continuousMixedCollisionHits.Add(candidate.ExactHit);
        }
    }

    internal bool ApplyKinematicContinuousCollisionHandoff(
        SolidBody2D target,
        Vector2d sourceDisplacement,
        Vector2d normalForSource,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        _ = ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(normalForSource, sourceDisplacement, out Vector2d normal);

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal);
        if (constrainedInverseMass <= Fixed64.Zero)
            return false;

        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        Vector2d sourceVelocity = SampleContinuousCollisionLinearVelocity(hitTime);
        Vector2d targetVelocity = target.SampleContinuousCollisionLinearVelocity(hitTime);
        bool relativeVelocityResolved = Vector2d.TrySubtract(
            sourceVelocity,
            targetVelocity,
            out Vector2d relativeVelocity);
        Fixed64 normalVelocity = Vector2d.Dot(relativeVelocity, normal);
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
        Vector2d targetResponseNormal = target.ProjectLinearMotion(-normal);
        bool targetDeltaResolved = ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            targetResponseNormal,
            responseSpeed,
            target.EffectiveInverseMass,
            constrainedInverseMass,
            out Vector2d targetVelocityDelta);
        bool targetVelocityResolved = Vector2d.TryAdd(
            targetVelocity,
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
            ignoredCollider2D: Collider);
    }

    internal bool ApplyKinematicContinuousCollisionHandoff(
        SolidBody target,
        Vector2d sourceDisplacement,
        Vector2d normalForSource,
        Fixed64 hitDistance,
        Fixed64 sourceLength)
    {
        if (!ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
                normalForSource,
                out Vector2d normal))
        {
            return false;
        }

        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 constrainedInverseMass = target.GetConstrainedInverseMass(normal.ToVector3d(Fixed64.Zero));
        if (constrainedInverseMass <= Fixed64.Zero)
            return false;

        Vector3d normal3D = normal.ToVector3d(Fixed64.Zero);
        Fixed64 hitTime = FixedMath.Clamp01(hitDistance / sourceLength);
        Vector3d sourceVelocity = SampleContinuousCollisionLinearVelocity(hitTime)
            .ToVector3d(Fixed64.Zero);
        Vector3d targetVelocity = target.SampleContinuousCollisionLinearVelocity(hitTime);
        bool relativeVelocityResolved = Vector3d.TrySubtract(
            sourceVelocity,
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
        Vector3d targetResponseNormal = target.ProjectLinearMotion(-normal3D);
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
            ignoredCollider2D: Collider);
    }

}
