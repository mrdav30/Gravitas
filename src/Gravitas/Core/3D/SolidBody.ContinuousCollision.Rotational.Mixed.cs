//=======================================================================
// SolidBody.ContinuousCollision.Rotational.Mixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;

namespace Gravitas;

public partial class SolidBody
{
    private readonly SwiftList<LSCollider2D> _rotationalMixedStatic2DCandidates =
        new(DefaultBodyHitBufferCapacity);
    private readonly SwiftList<int> _rotationalMixedMoving2DCandidateIds =
        new(DefaultBodyHitBufferCapacity);

    internal void SetMixedContinuousCollisionSamplePose(
        Vector3d position,
        FixedQuaternion rotation)
    {
        _position2dUnmarked = position.ToVector2d();
        _heightPosUnmarked = position.Y;
        _rotation = rotation;
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
    }

    private readonly struct RotationalMixed2DHit
    {
        public RotationalMixed2DHit(
            Fixed64 safeTime,
            MixedContact contact,
            bool hasContact,
            Fixed64 contactTime,
            LSCollider2D target)
        {
            SafeTime = safeTime;
            Contact = contact;
            HasContact = hasContact;
            ContactTime = contactTime;
            Target = target;
        }

        public Fixed64 SafeTime { get; }

        public MixedContact Contact { get; }

        public bool HasContact { get; }

        public Fixed64 ContactTime { get; }

        public LSCollider2D Target { get; }
    }

    private bool TryFindEarliestMixedRotationalContinuousCollision(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool sourceIsKinematic,
        out RotationalMixed2DHit hit)
    {
        hit = default;
        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        GatherMixedRotationalContinuousCollisionCandidates(
            startPosition,
            displacement,
            pivotRadius);

        bool found = false;
        Fixed64 earliestTime = Fixed64.One;
        int earliestId = int.MaxValue;
        MixedContact earliestContact = default;
        bool earliestHasContact = false;
        Fixed64 earliestContactTime = Fixed64.Zero;
        LSCollider2D? earliestTarget = null;

        for (int i = 0; i < _rotationalMixedStatic2DCandidates.Count; i++)
        {
            LSCollider2D target = _rotationalMixedStatic2DCandidates[i];
            ConsiderMixedRotationalContinuousCollisionCandidate(
                target,
                startPosition,
                displacement,
                startRotation,
                targetRotation,
                angularDistance,
                pivotRadius,
                elapsedTime,
                remainingTime,
                sourceIsKinematic,
                ref found,
                ref earliestTime,
                ref earliestId,
                ref earliestContact,
                ref earliestHasContact,
                ref earliestContactTime,
                ref earliestTarget);
        }

        for (int i = 0; i < _rotationalMixedMoving2DCandidateIds.Count; i++)
        {
            SolidBody2D targetBody = Context.Physics2D.GetContinuousCollisionCandidate(
                _rotationalMixedMoving2DCandidateIds[i]);
            if (!IsMovingMixedRotationalContinuousCollisionTarget(targetBody))
            {
                continue;
            }

            ConsiderMixedRotationalContinuousCollisionCandidate(
                targetBody.Collider,
                startPosition,
                displacement,
                startRotation,
                targetRotation,
                angularDistance,
                pivotRadius,
                elapsedTime,
                remainingTime,
                sourceIsKinematic,
                ref found,
                ref earliestTime,
                ref earliestId,
                ref earliestContact,
                ref earliestHasContact,
                ref earliestContactTime,
                ref earliestTarget);
        }

        if (!found)
            return false;

        hit = new RotationalMixed2DHit(
            earliestTime,
            earliestContact,
            earliestHasContact,
            earliestContactTime,
            earliestTarget!);
        return true;
    }

    private void GatherMixedRotationalContinuousCollisionCandidates(
        Vector3d startPosition,
        Vector3d displacement,
        Fixed64 pivotRadius)
    {
        _rotationalMixedStatic2DCandidates.FastClear();
        _rotationalMixedMoving2DCandidateIds.FastClear();
        if (pivotRadius == Fixed64.MaxValue)
        {
            int count = Context.Physics2D.ColliderCount;
            _rotationalMixedStatic2DCandidates.EnsureCapacity(count);
            _rotationalMixedMoving2DCandidateIds.EnsureCapacity(count);
            for (int i = 0; i < count; i++)
            {
                LSCollider2D target = Context.Physics2D.GetColliderByServiceIndex(i);
                if (target.Body is SolidBody2D targetBody
                    && IsMovingMixedRotationalContinuousCollisionTarget(targetBody))
                {
                    _rotationalMixedMoving2DCandidateIds.Add(targetBody.DynamicId);
                    continue;
                }

                _rotationalMixedStatic2DCandidates.Add(target);
            }

            return;
        }

        FixedBoundVolume bounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            startPosition,
            displacement,
            pivotRadius);
        Context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            bounds.Min,
            bounds.Max,
            PhysicsLayerMask.All,
            _rotationalMixedStatic2DCandidates,
            staticStyleOnly: true,
            cachePartitionRefresh: true);

        SwiftList<int> candidateIds =
            Context.Physics2D.QueryMixedContinuousCollisionCandidates(bounds);
        _rotationalMixedMoving2DCandidateIds.EnsureCapacity(candidateIds.Count);
        for (int i = 0; i < candidateIds.Count; i++)
            _rotationalMixedMoving2DCandidateIds.Add(candidateIds[i]);
    }

    private void ConsiderMixedRotationalContinuousCollisionCandidate(
        LSCollider2D target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool sourceIsKinematic,
        ref bool found,
        ref Fixed64 earliestTime,
        ref int earliestId,
        ref MixedContact earliestContact,
        ref bool earliestHasContact,
        ref Fixed64 earliestContactTime,
        ref LSCollider2D? earliestTarget)
    {
        if (!IsValidMixedRotationalContinuousCollisionTarget(target)
            || !TryFindEarliestMixedRotationalContinuousCollisionAgainstTarget(
                target,
                startPosition,
                displacement,
                startRotation,
                targetRotation,
                angularDistance,
                pivotRadius,
                elapsedTime,
                remainingTime,
                sourceIsKinematic,
                out Fixed64 candidateTime,
                out MixedContact candidateContact,
                out bool candidateHasContact,
                out Fixed64 candidateContactTime)
            || !ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateTime,
                target.Id,
                found,
                earliestTime,
                earliestId))
        {
            return;
        }

        found = true;
        earliestTime = candidateTime;
        earliestId = target.Id;
        earliestContact = candidateContact;
        earliestHasContact = candidateHasContact;
        earliestContactTime = candidateContactTime;
        earliestTarget = target;
    }

    private bool TryFindEarliestMixedRotationalContinuousCollisionAgainstTarget(
        LSCollider2D target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool sourceIsKinematic,
        out Fixed64 safeTime,
        out MixedContact contact,
        out bool hasContact,
        out Fixed64 contactTime)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        SolidBody2D? targetBody = target.Body;
        bool samplesTargetMotion = targetBody != null
            && IsMovingMixedRotationalContinuousCollisionTarget(targetBody);
        Vector2d originalTargetPosition = targetBody?.Position ?? Vector2d.Zero;
        Fixed64 originalTargetRotation = targetBody?.Rotation ?? Fixed64.Zero;
        if (samplesTargetMotion)
            targetBody!.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

        try
        {
            Span<ContinuousCollisionMath.RotationalInterval> intervals =
                stackalloc ContinuousCollisionMath.RotationalInterval[
                    ContinuousCollisionMath.RotationalIntervalMaxDepth + 2];
            int intervalCount = 1;
            int processedNodeCount = 0;
            bool hasKnownContact = false;
            Fixed64 knownContactTime = Fixed64.One;
            MixedContact knownContact = default;
            intervals[0] = new ContinuousCollisionMath.RotationalInterval(
                Fixed64.Zero,
                Fixed64.One,
                depth: 0);

            while (intervalCount > 0)
            {
                ContinuousCollisionMath.RotationalInterval interval = intervals[--intervalCount];
                Fixed64 midpoint =
                    (interval.LowerTime + interval.UpperTime) * Fixed64.Half;
                Fixed64 intervalSpan = interval.UpperTime - interval.LowerTime;
                SampleMixedRotationalContinuousPairPose(
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    midpoint,
                    elapsedTime,
                    remainingTime,
                    sourceIsKinematic,
                    targetBody,
                    samplesTargetMotion);
                processedNodeCount++;

                bool hasMotionBound =
                    ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                        displacement,
                        angularDistance,
                        pivotRadius,
                        intervalSpan,
                        out Fixed64 motionBound);
                if (hasMotionBound && samplesTargetMotion)
                {
                    Fixed64 lowerFraction = ResolveRotationalFrameFraction(
                        elapsedTime,
                        remainingTime,
                        interval.LowerTime);
                    Fixed64 upperFraction = ResolveRotationalFrameFraction(
                        elapsedTime,
                        remainingTime,
                        interval.UpperTime);
                    Fixed64 targetRadius = targetBody!.ResolveContinuousCollisionProxyRadius();
                    bool targetBoundResolved = targetBody.TryResolveContinuousCollisionMotionBound(
                        lowerFraction,
                        upperFraction,
                        targetRadius,
                        out Fixed64 targetMotionBound);
                    bool combinedBoundResolved = Fixed64.TryAdd(
                        motionBound,
                        targetMotionBound,
                        out motionBound);
                    hasMotionBound = targetBoundResolved & combinedBoundResolved;
                }

                bool sampleHasContact = CollisionDetectionMixed.TryCollide(
                    Collider,
                    target,
                    out MixedContact sampleContact);
                if (sampleHasContact
                    && (!hasKnownContact || midpoint < knownContactTime))
                {
                    hasKnownContact = true;
                    knownContactTime = midpoint;
                    knownContact = sampleContact;
                }

                bool hasShapeGap = CollisionDetectionMixed.TryGetRotationalSeparationGap(
                    Collider,
                    target,
                    out Fixed64 shapeGap,
                    out bool shapeGapSupported);
                Fixed64 characteristicScale = target is LSCircleCollider2D circle
                    ? FixedMath.Max(pivotRadius, circle.ScaledRadius)
                    : pivotRadius;
                bool shapeGapCertifiesSeparation = shapeGapSupported
                    && hasShapeGap
                    && ContinuousCollisionMath.TrySubtractClosestFeatureUncertainty(
                        shapeGap,
                        characteristicScale,
                        out Fixed64 conservativeShapeGap)
                    && conservativeShapeGap > motionBound;
                bool fallbackBoundsCertifySeparation = !shapeGapSupported
                    && ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(
                        Collider.Bounds,
                        target.MixedBounds3D,
                        motionBound);
                if (!sampleHasContact
                    && hasMotionBound
                    && (shapeGapCertifiesSeparation || fallbackBoundsCertifySeparation))
                {
                    continue;
                }

                if (ContinuousCollisionMath.TryResolveRotationalSearchLimit(
                        interval,
                        processedNodeCount,
                        hasKnownContact,
                        knownContactTime,
                        out safeTime,
                        out contactTime,
                        out hasContact))
                {
                    contact = knownContact;
                    return true;
                }

                int childDepth = interval.Depth + 1;
                if (sampleHasContact)
                {
                    intervals[intervalCount++] =
                        new ContinuousCollisionMath.RotationalInterval(
                            interval.LowerTime,
                            midpoint,
                            childDepth);
                    continue;
                }

                intervals[intervalCount++] =
                    new ContinuousCollisionMath.RotationalInterval(
                        midpoint,
                        interval.UpperTime,
                        childDepth);
                intervals[intervalCount++] =
                    new ContinuousCollisionMath.RotationalInterval(
                        interval.LowerTime,
                        midpoint,
                        childDepth);
            }

            return false;
        }
        finally
        {
            if (targetBody != null)
                targetBody.SetMixedContinuousCollisionSamplePose(
                    originalTargetPosition,
                    originalTargetRotation);
        }
    }

    private void SampleMixedRotationalContinuousPairPose(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 sampleTime,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool sourceIsKinematic,
        SolidBody2D? targetBody,
        bool samplesTargetMotion)
    {
        Position3d = startPosition + displacement * sampleTime;
        Rotation = sourceIsKinematic
            ? FixedQuaternion.Slerp(startRotation, targetRotation, sampleTime).Normalized
            : IntegrateAngularRotation(
                startRotation,
                _angularVelocity,
                remainingTime * sampleTime);
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

        if (!samplesTargetMotion)
            return;

        Fixed64 frameFraction = ResolveRotationalFrameFraction(
            elapsedTime,
            remainingTime,
            sampleTime);
        SolidBody2D movingTarget = targetBody!;
        movingTarget.SetMixedContinuousCollisionSamplePose(
            movingTarget.SampleContinuousCollisionPosition(frameFraction),
            movingTarget.SampleContinuousCollisionRotation(frameFraction));
    }

    private bool IsValidMixedRotationalContinuousCollisionTarget(LSCollider2D target)
    {
        return target.IsActive
            && !target.IsTrigger
            && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
                target,
                _continuousCollisionHandoffIgnoredCollider2D)
            && Context.MixedCollisions.RequireCollisionPair(Collider, target);
    }

    private bool IsMovingMixedRotationalContinuousCollisionTarget(SolidBody2D target)
    {
        if (target.IsKinematic)
            target.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

        return (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
                HasContinuousCollisionRotationalMotion))
            && IsValidMixedRotationalContinuousCollisionTarget(target.Collider)
            && (target.IsKinematic
                ? target.HasContinuousCollisionMotion
                : target.IsDynamic);
    }

    internal bool TryApplyMixedRotationalContinuousCollisionResponse(
        LSCollider2D targetCollider,
        MixedContact contact,
        Fixed64 localContactTime,
        Vector3d sourceSegmentStart,
        Vector3d sourceDisplacement,
        FixedQuaternion sourceSegmentStartRotation,
        FixedQuaternion sourceSegmentTargetRotation,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool sourceIsKinematic)
    {
        Fixed64 frameFraction = ResolveRotationalFrameFraction(
            elapsedTime,
            remainingTime,
            localContactTime);
        Fixed64 consumedTime = remainingTime * localContactTime;
        Fixed64 impactElapsedTime = elapsedTime + consumedTime;
        Fixed64 remainingAfterImpact = remainingTime - consumedTime;
        Vector3d sourcePosition = sourceSegmentStart
            + sourceDisplacement * localContactTime;
        FixedQuaternion sourceRotation = sourceIsKinematic
            ? FixedQuaternion.Slerp(
                sourceSegmentStartRotation,
                sourceSegmentTargetRotation,
                localContactTime).Normalized
            : IntegrateAngularRotation(
                sourceSegmentStartRotation,
                _angularVelocity,
                consumedTime);
        Vector3d sourceLinearVelocity = sourceIsKinematic
            ? SampleContinuousCollisionLinearVelocity(frameFraction)
            : _linearVelocity;
        Vector3d sourceAngularVelocity = sourceIsKinematic
            ? SampleContinuousCollisionAngularVelocity(frameFraction)
            : _angularVelocity;

        SolidBody2D? target = targetCollider.Body;
        Vector2d targetPosition = target?.SampleContinuousCollisionPosition(frameFraction)
            ?? targetCollider.Center;
        Fixed64 targetRotation = target?.SampleContinuousCollisionRotation(frameFraction)
            ?? Fixed64.Zero;
        Vector2d targetLinearVelocity = target?.SampleContinuousCollisionLinearVelocity(frameFraction)
            ?? Vector2d.Zero;
        Fixed64 targetAngularVelocity = target?.SampleContinuousCollisionAngularVelocity(frameFraction)
            ?? Fixed64.Zero;
        SolidBody? sourceResponseBody = HasSolverMobility ? this : null;
        SolidBody2D? targetResponseBody = target?.HasSolverMobility == true ? target : null;
        if (!Vector3d.TryAdd(
                sourcePosition,
                sourceRotation * LocalCenterOfMassOffset,
                out Vector3d sourceCenterOfMass))
        {
            return false;
        }

        Vector2d targetCenterOfMass = targetPosition;
        if (target != null
            && !Vector2d.TryAdd(
                targetPosition,
                Vector2d.Rotate(target.LocalCenterOfMassOffset, targetRotation),
                out targetCenterOfMass))
        {
            return false;
        }

        Fixed64 restitution = PhysicsMaterial.CombineRestitution(
            Collider.Material,
            targetCollider.Material);
        bool sourceContactArmResolved = Vector3d.TrySubtract(
            contact.Point3D,
            sourceCenterOfMass,
            out Vector3d sourceContactArm);
        bool targetContactArmResolved = Vector2d.TrySubtract(
            contact.Point2D.ToVector2d(),
            targetCenterOfMass,
            out Vector2d targetContactArm);
        bool responseResolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
            sourceResponseBody,
            sourceLinearVelocity,
            sourceAngularVelocity,
            sourceContactArm,
            targetResponseBody,
            targetLinearVelocity,
            targetAngularVelocity,
            targetContactArm,
            contact.Normal3DTo2D,
            restitution,
            Context.Settings.RestitutionVelocityThreshold,
            out ContactNormalVelocityDeltaResultMixed response);
        bool responseAdmissible = sourceContactArmResolved
            & targetContactArmResolved
            & responseResolved
            & response.NormalVelocity < -Fixed64.Epsilon;
        if (!responseAdmissible)
        {
            return false;
        }

        Vector3d postSourceLinearVelocity = sourceLinearVelocity;
        Vector3d postSourceAngularVelocity = sourceAngularVelocity;
        Vector2d postTargetLinearVelocity = targetLinearVelocity;
        Fixed64 postTargetAngularVelocity = targetAngularVelocity;
        bool sourceStateAvailable = true;
        if (sourceResponseBody != null)
        {
            bool sourceLinearVelocityResolved = Vector3d.TryAdd(
                sourceLinearVelocity,
                response.LinearVelocityDelta3D,
                out postSourceLinearVelocity);
            bool sourceAngularVelocityResolved = Vector3d.TryAdd(
                sourceAngularVelocity,
                response.AngularVelocityDelta3D,
                out postSourceAngularVelocity);
            bool sourceTrajectoryAvailable = CanAppendContinuousCollisionSegment(
                impactElapsedTime);
            sourceStateAvailable = sourceLinearVelocityResolved
                & sourceAngularVelocityResolved
                & sourceTrajectoryAvailable;
        }

        bool targetStateAvailable = true;
        if (targetResponseBody != null)
        {
            bool targetLinearVelocityResolved = Vector2d.TryAdd(
                targetLinearVelocity,
                response.LinearVelocityDelta2D,
                out postTargetLinearVelocity);
            bool targetAngularVelocityResolved = Fixed64.TryAdd(
                targetAngularVelocity,
                response.AngularVelocityDelta2D,
                out postTargetAngularVelocity);
            bool targetTrajectoryAvailable = targetResponseBody.CanApplyContinuousCollisionHandoffState(
                targetPosition,
                remainingAfterImpact,
                out targetPosition);
            targetStateAvailable = targetLinearVelocityResolved
                & targetAngularVelocityResolved
                & targetTrajectoryAvailable;
        }

        if (!(sourceStateAvailable & targetStateAvailable))
        {
            return false;
        }

        if (sourceResponseBody != null)
            _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this);

        if (targetResponseBody != null)
        {
            _ = Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(targetResponseBody);
            targetResponseBody.ApplyContinuousCollisionHandoffStateReserved(
                targetPosition,
                targetRotation,
                postTargetLinearVelocity,
                postTargetAngularVelocity,
                remainingAfterImpact,
                ignoredCollider3D: Collider);
        }

        if (sourceResponseBody != null)
        {
            ApplyCollisionVelocityState(
                postSourceLinearVelocity,
                postSourceAngularVelocity);
            AppendContinuousCollisionSegment(
                sourcePosition,
                sourceRotation,
                _linearVelocity,
                _angularVelocity,
                impactElapsedTime);
            Context.Physics.RefreshContinuousCollisionCandidate(this);
        }

        return true;
    }
}
