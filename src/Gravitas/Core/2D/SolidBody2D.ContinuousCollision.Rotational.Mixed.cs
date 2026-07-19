//=======================================================================
// SolidBody2D.ContinuousCollision.Rotational.Mixed.cs
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

public sealed partial class SolidBody2D
{
    private readonly SwiftList<LSCollider> _rotationalMixedStatic3DCandidates = new();
    private readonly SwiftList<int> _rotationalMixedMoving3DCandidateIds = new();

    internal void SetMixedContinuousCollisionSamplePose(
        Vector2d position,
        Fixed64 rotation)
    {
        _position = position;
        _rotation = rotation;
        Collider.RebuildRuntimeShapeOnly();
    }

    internal FixedBoundVolume ResolveMixedContinuousCollisionTrajectoryBounds(Fixed64 radius)
    {
        DynamicCcdPlanarBounds planarBounds =
            ResolveContinuousCollisionTrajectoryBounds(radius);
        return new FixedBoundVolume(
            new Vector3d(
                planarBounds.MinX,
                Collider.MixedSlabCenterY - Collider.MixedHalfThickness,
                planarBounds.MinZ),
            new Vector3d(
                planarBounds.MaxX,
                Collider.MixedSlabCenterY + Collider.MixedHalfThickness,
                planarBounds.MaxZ));
    }

    private readonly struct RotationalMixed3DHit
    {
        public RotationalMixed3DHit(
            Fixed64 safeTime,
            MixedContact contact,
            bool hasContact,
            Fixed64 contactTime,
            LSCollider target)
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

        public LSCollider Target { get; }
    }

    private bool TryFindEarliestMixedRotationalContinuousCollision(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        out RotationalMixed3DHit hit)
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
        LSCollider? earliestTarget = null;

        for (int i = 0; i < _rotationalMixedStatic3DCandidates.Count; i++)
        {
            LSCollider target = _rotationalMixedStatic3DCandidates[i];
            ConsiderMixedRotationalContinuousCollisionCandidate(
                target,
                startPosition,
                displacement,
                startRotation,
                angularDelta,
                angularDistance,
                pivotRadius,
                elapsedTime,
                remainingTime,
                ref found,
                ref earliestTime,
                ref earliestId,
                ref earliestContact,
                ref earliestHasContact,
                ref earliestContactTime,
                ref earliestTarget);
        }

        for (int i = 0; i < _rotationalMixedMoving3DCandidateIds.Count; i++)
        {
            SolidBody targetBody = Context.Physics.GetContinuousCollisionCandidate(
                _rotationalMixedMoving3DCandidateIds[i]);
            if (!IsMovingMixedRotationalContinuousCollisionTarget(targetBody))
            {
                continue;
            }

            ConsiderMixedRotationalContinuousCollisionCandidate(
                targetBody.Collider,
                startPosition,
                displacement,
                startRotation,
                angularDelta,
                angularDistance,
                pivotRadius,
                elapsedTime,
                remainingTime,
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

        hit = new RotationalMixed3DHit(
            earliestTime,
            earliestContact,
            earliestHasContact,
            earliestContactTime,
            earliestTarget!);
        return true;
    }

    private void GatherMixedRotationalContinuousCollisionCandidates(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 pivotRadius)
    {
        _rotationalMixedStatic3DCandidates.FastClear();
        _rotationalMixedMoving3DCandidateIds.FastClear();
        if (pivotRadius == Fixed64.MaxValue)
        {
            int count = Context.Physics.ColliderCount;
            _rotationalMixedStatic3DCandidates.EnsureCapacity(count);
            _rotationalMixedMoving3DCandidateIds.EnsureCapacity(count);
            for (int i = 0; i < count; i++)
            {
                LSCollider target = Context.Physics.GetColliderByServiceIndex(i);
                if (target.Body is SolidBody targetBody
                    && IsMovingMixedRotationalContinuousCollisionTarget(targetBody))
                {
                    _rotationalMixedMoving3DCandidateIds.Add(targetBody.DynamicId);
                    continue;
                }

                _rotationalMixedStatic3DCandidates.Add(target);
            }

            return;
        }

        Vector3d start3D = new(
            startPosition.X,
            Collider.MixedSlabCenterY,
            startPosition.Y);
        Vector3d end3D = new(
            startPosition.X + displacement.X,
            Collider.MixedSlabCenterY,
            startPosition.Y + displacement.Y);
        FixedBoundVolume bounds = DynamicCcdCandidateIndex.CreateBoundsBetween(
            start3D,
            end3D,
            new Vector3d(
                pivotRadius,
                Collider.MixedHalfThickness,
                pivotRadius));
        Context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            bounds.Min,
            bounds.Max,
            PhysicsLayerMask.All,
            _rotationalMixedStatic3DCandidates,
            staticStyleOnly: true,
            cachePartitionRefresh: true);

        SwiftList<int> candidateIds =
            Context.Physics.QueryContinuousCollisionCandidates(bounds);
        _rotationalMixedMoving3DCandidateIds.EnsureCapacity(candidateIds.Count);
        for (int i = 0; i < candidateIds.Count; i++)
            _rotationalMixedMoving3DCandidateIds.Add(candidateIds[i]);
    }

    private void ConsiderMixedRotationalContinuousCollisionCandidate(
        LSCollider target,
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        ref bool found,
        ref Fixed64 earliestTime,
        ref int earliestId,
        ref MixedContact earliestContact,
        ref bool earliestHasContact,
        ref Fixed64 earliestContactTime,
        ref LSCollider? earliestTarget)
    {
        if (!IsValidMixedRotationalContinuousCollisionTarget(target)
            || !TryFindEarliestMixedRotationalContinuousCollisionAgainstTarget(
                target,
                startPosition,
                displacement,
                startRotation,
                angularDelta,
                angularDistance,
                pivotRadius,
                elapsedTime,
                remainingTime,
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
        LSCollider target,
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        out Fixed64 safeTime,
        out MixedContact contact,
        out bool hasContact,
        out Fixed64 contactTime)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        SolidBody? targetBody = target.Body;
        bool samplesTargetMotion = targetBody != null
            && IsMovingMixedRotationalContinuousCollisionTarget(targetBody);
        Vector3d originalTargetPosition = targetBody?.Position3d ?? Vector3d.Zero;
        FixedQuaternion originalTargetRotation = targetBody?.Rotation
            ?? FixedQuaternion.Identity;
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
                if (hasKnownContact && interval.LowerTime >= knownContactTime)
                    continue;

                Fixed64 midpoint =
                    (interval.LowerTime + interval.UpperTime) * Fixed64.Half;
                Fixed64 intervalSpan = interval.UpperTime - interval.LowerTime;
                SampleMixedRotationalContinuousPairPose(
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    midpoint,
                    elapsedTime,
                    remainingTime,
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
                    target,
                    Collider,
                    out MixedContact sampleContact);
                if (sampleHasContact
                    && (!hasKnownContact || midpoint < knownContactTime))
                {
                    hasKnownContact = true;
                    knownContactTime = midpoint;
                    knownContact = sampleContact;
                }

                bool hasShapeGap = CollisionDetectionMixed.TryGetRotationalSeparationGap(
                    target,
                    Collider,
                    out Fixed64 shapeGap,
                    out bool shapeGapSupported);
                Fixed64 characteristicScale = target is LSSphereCollider sphere
                    ? FixedMath.Max(pivotRadius, sphere.ScaledRadius)
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
                        target.Bounds,
                        Collider.MixedBounds3D,
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

            safeTime = knownContactTime;
            contact = knownContact;
            hasContact = hasKnownContact;
            contactTime = knownContactTime;
            return hasKnownContact;
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
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 sampleTime,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        SolidBody? targetBody,
        bool samplesTargetMotion)
    {
        _position = startPosition + displacement * sampleTime;
        _rotation = startRotation + angularDelta * sampleTime;
        Collider.RebuildRuntimeShapeOnly();

        if (!samplesTargetMotion)
            return;

        Fixed64 frameFraction = ResolveRotationalFrameFraction(
            elapsedTime,
            remainingTime,
            sampleTime);
        targetBody!.SetMixedContinuousCollisionSamplePose(
            targetBody.SampleContinuousCollisionPosition(frameFraction),
            targetBody.SampleContinuousCollisionRotation(frameFraction));
    }

    private bool IsValidMixedRotationalContinuousCollisionTarget(LSCollider target)
    {
        return target.IsActive
            && !target.IsTrigger
            && !ContinuousCollisionCandidateOrdering.IsIgnoredTarget(
                target,
                _continuousCollisionHandoffIgnoredCollider3D)
            && Context.MixedCollisions.RequireCollisionPair(target, Collider);
    }

    private bool IsMovingMixedRotationalContinuousCollisionTarget(SolidBody target)
    {
        return target.Active
            && (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
                HasContinuousCollisionRotationalMotion))
            && IsValidMixedRotationalContinuousCollisionTarget(target.Collider)
            && (target.IsKinematic || !target.IsPositionFullyFrozen);
    }

    internal bool TryApplyMixedRotationalContinuousCollisionResponse(
        LSCollider targetCollider,
        MixedContact contact,
        Fixed64 localContactTime,
        Vector2d sourceSegmentStart,
        Vector2d sourceDisplacement,
        Fixed64 sourceSegmentStartRotation,
        Fixed64 sourceAngularDelta,
        Fixed64 elapsedTime,
        Fixed64 remainingTime)
    {
        Fixed64 frameFraction = ResolveRotationalFrameFraction(
            elapsedTime,
            remainingTime,
            localContactTime);
        Fixed64 consumedTime = remainingTime * localContactTime;
        Fixed64 impactElapsedTime = elapsedTime + consumedTime;
        Fixed64 remainingAfterImpact = remainingTime - consumedTime;
        Vector2d sourcePosition = sourceSegmentStart
            + sourceDisplacement * localContactTime;
        Fixed64 sourceRotation = CanonicalizeRotation(
            sourceSegmentStartRotation + sourceAngularDelta * localContactTime);
        Vector2d sourceLinearVelocity = IsKinematic
            ? SampleContinuousCollisionLinearVelocity(frameFraction)
            : _linearVelocity;
        Fixed64 sourceAngularVelocity = IsKinematic
            ? SampleContinuousCollisionAngularVelocity(frameFraction)
            : _angularVelocity;

        SolidBody? target = targetCollider.Body;
        Vector3d targetPosition = target?.SampleContinuousCollisionPosition(frameFraction)
            ?? targetCollider.Center;
        FixedQuaternion targetRotation = target?.SampleContinuousCollisionRotation(frameFraction)
            ?? FixedQuaternion.Identity;
        Vector3d targetLinearVelocity = target?.SampleContinuousCollisionLinearVelocity(frameFraction)
            ?? Vector3d.Zero;
        Vector3d targetAngularVelocity = target?.SampleContinuousCollisionAngularVelocity(frameFraction)
            ?? Vector3d.Zero;
        SolidBody? targetResponseBody = target?.IsKinematic == false ? target : null;
        SolidBody2D? sourceResponseBody = IsKinematic ? null : this;
        Vector3d targetCenterOfMass = targetPosition;
        if (target != null)
        {
            if (!Vector3d.TryAdd(
                    targetPosition,
                    targetRotation * target.LocalCenterOfMassOffset,
                    out targetCenterOfMass))
            {
                return false;
            }
        }

        bool sourceCenterResolved = Vector2d.TryAdd(
            sourcePosition,
            ClampNearZero(Vector2d.Rotate(LocalCenterOfMassOffset, sourceRotation)),
            out Vector2d sourceCenterOfMass);

        Fixed64 restitution = PhysicsMaterial.CombineRestitution(
            targetCollider.Material,
            Collider.Material);
        bool targetContactArmResolved = Vector3d.TrySubtract(
            contact.Point3D,
            targetCenterOfMass,
            out Vector3d targetContactArm);
        bool sourceContactArmResolved = Vector2d.TrySubtract(
            contact.Point2D.ToVector2d(),
            sourceCenterOfMass,
            out Vector2d sourceContactArm);
        bool responseResolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                targetResponseBody,
                targetLinearVelocity,
                targetAngularVelocity,
                targetContactArm,
                sourceResponseBody,
                sourceLinearVelocity,
                sourceAngularVelocity,
                sourceContactArm,
                contact.Normal3DTo2D,
                restitution,
                Context.Settings.RestitutionVelocityThreshold,
                out ContactNormalVelocityDeltaResultMixed response);
        if (!(sourceCenterResolved
                & targetContactArmResolved
                & sourceContactArmResolved
                & responseResolved)
            || response.NormalVelocity >= -Fixed64.Epsilon)
        {
            return false;
        }

        Vector3d postTargetLinearVelocity = targetLinearVelocity;
        Vector3d postTargetAngularVelocity = targetAngularVelocity;
        Vector3d targetResolvedPosition = default;
        if (!IsKinematic
            && !(CanApplyCollisionVelocityDeltas(
                    response.LinearVelocityDelta2D,
                    response.AngularVelocityDelta2D)
                & CanAppendContinuousCollisionFrameSegment(impactElapsedTime)
                & Context.Physics2D.CanAdmitContinuousCollisionCandidateRefresh(this)))
        {
            return false;
        }

        if (targetResponseBody != null)
        {
            bool targetLinearVelocityResolved = Vector3d.TryAdd(
                targetLinearVelocity,
                response.LinearVelocityDelta3D,
                out postTargetLinearVelocity);
            bool targetAngularVelocityResolved = Vector3d.TryAdd(
                targetAngularVelocity,
                response.AngularVelocityDelta3D,
                out postTargetAngularVelocity);
            bool targetHandoffAdmissible = targetResponseBody.CanApplyContinuousCollisionHandoff(
                targetPosition,
                remainingAfterImpact,
                out targetResolvedPosition);
            if (!(targetLinearVelocityResolved
                    & targetAngularVelocityResolved
                    & targetHandoffAdmissible))
            {
                return false;
            }
        }

        if (!IsKinematic)
            _ = Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this);

        if (targetResponseBody != null)
        {
            // Dirty overlay capacity is the registered-body count, so a valid
            // registered target cannot exhaust this reservation.
            _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(
                targetResponseBody);
            targetResponseBody.ApplyContinuousCollisionHandoffReserved(
                targetResolvedPosition,
                targetRotation,
                postTargetLinearVelocity,
                postTargetAngularVelocity,
                remainingAfterImpact,
                ignoredCollider2D: Collider);
        }

        if (!IsKinematic)
        {
            ApplyCollisionLinearVelocityDelta(response.LinearVelocityDelta2D);
            ApplyCollisionAngularVelocityDelta(response.AngularVelocityDelta2D);
        }

        return true;
    }
}
