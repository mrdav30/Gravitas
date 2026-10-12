//=======================================================================
// SolidBody2D.ContinuousCollision.Rotational.Mixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
    private readonly ContactManifold _rotationalMixedKnownManifold = new();
    private readonly ContactManifold _rotationalMixedEarliestManifold = new();
    private bool _rotationalMixedKnownContactIsClosing;
    private bool _rotationalMixedEarliestContactIsClosing;
    private ContactWarmStartCache _mixedContinuousCollisionImpulses;
    private ExactContactImpulseState?[]? _mixedContinuousCollisionExactImpulses;
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
            bool hasClosingContact,
            Fixed64 contactTime,
            LSCollider target,
            ContactManifold manifold)
        {
            SafeTime = safeTime;
            Contact = contact;
            HasContact = hasContact;
            HasClosingContact = hasClosingContact;
            ContactTime = contactTime;
            Target = target;
            Manifold = manifold;
        }

        public Fixed64 SafeTime { get; }

        public MixedContact Contact { get; }

        public bool HasContact { get; }
        public bool HasClosingContact { get; }

        public Fixed64 ContactTime { get; }

        public LSCollider Target { get; }
        public ContactManifold Manifold { get; }
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
            _rotationalMixedEarliestContactIsClosing,
            earliestContactTime,
            earliestTarget!,
            _rotationalMixedEarliestManifold);
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
                Fixed64.Zero,
                Fixed64.One,
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
        _rotationalMixedEarliestManifold.CopyFrom(_rotationalMixedKnownManifold, Context.FrameCount);
        _rotationalMixedEarliestContactIsClosing = _rotationalMixedKnownContactIsClosing;
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
        Fixed64 searchStart,
        Fixed64 searchEnd,
        out Fixed64 safeTime,
        out MixedContact contact,
        out bool hasContact,
        out Fixed64 contactTime)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        _rotationalMixedKnownManifold.BeginUpdate(Context.FrameCount);
        _rotationalMixedKnownContactIsClosing = false;
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
                searchStart,
                searchEnd,
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

                bool sampleHasContact = TrySampleMixedRotationalContinuousContact(target,
                    ResolveRotationalFrameFraction(elapsedTime, remainingTime, midpoint),
                    out MixedContact sampleContact, out bool sampleContactIsClosing);
                if (sampleHasContact
                    && (!hasKnownContact || midpoint < knownContactTime))
                {
                    hasKnownContact = true;
                    knownContactTime = midpoint;
                    knownContact = sampleContact;
                    _rotationalMixedKnownManifold.CopyFrom(Context.CollisionScratch.MixedManifold, Context.FrameCount);
                    _rotationalMixedKnownContactIsClosing = sampleContactIsClosing;
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

    private bool TryGetExactSphereCircleTranslationalContact(
        SolidBody target,
        ContinuousCollisionMotionSegment3D targetSegment,
        FixedSegment targetPath,
        Vector3d targetDisplacement,
        Fixed64 targetTravelDistance,
        FixedSegment2d sourcePath,
        Vector2d sourceDisplacement,
        Fixed64 sourceTravelDistance,
        Fixed64 sourceStartRotation,
        Fixed64 sourceAngularDelta,
        out bool requiresRotationalFallback,
        out Fixed64 relativeContactDistance,
        out Fixed64 sourceContactDistance,
        out Fixed64 localClosingSpeed,
        out MixedContact contact)
    {
        requiresRotationalFallback = true;
        relativeContactDistance = default;
        sourceContactDistance = default;
        localClosingSpeed = default;
        contact = default;
        if (Collider is not LSCircleCollider2D circle
            || target.Collider is not LSSphereCollider sphere
            || (targetSegment.AngularDistance != Fixed64.Zero
                && sphere.ScaledOffset != Vector3d.Zero)
            || (sourceAngularDelta != Fixed64.Zero
                && circle.ScaledLocalOffset != Vector2d.Zero))
        {
            return false;
        }

        Vector2d originalSourcePosition = _position;
        Fixed64 originalSourceRotation = _rotation;
        Vector3d originalTargetPosition = target.Position3d;
        FixedQuaternion originalTargetRotation = target.Rotation;
        try
        {
            _position = sourcePath.Start;
            _rotation = sourceStartRotation;
            Collider.RebuildRuntimeShapeOnly();
            target.SetMixedContinuousCollisionSamplePose(
                targetPath.Start,
                targetSegment.StartRotation);
            Vector3d cylinderStart = new(
                circle.Center.X,
                circle.MixedSlabCenterY,
                circle.Center.Y);
            Vector3d sphereStart = sphere.Center;
            Vector3d cylinderDisplacement = sourceDisplacement.ToVector3d(Fixed64.Zero);
            Vector3d relativeDisplacement =
                ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
                    targetDisplacement,
                    cylinderDisplacement,
                    out Fixed64 totalDistance);
            if (!WideFiniteAxisIntersection
                    .TryGetSphericallyExpandedFiniteCylinderDirectionFirstDistanceFromHalfAxisLength(
                        sphereStart,
                        relativeDisplacement,
                        cylinderStart,
                        Vector3d.Up,
                        circle.MixedHalfThickness,
                        circle.ScaledRadius,
                        sphere.ScaledRadius,
                        totalDistance,
                        out Fixed64 contactDistance))
            {
                requiresRotationalFallback = false;
                return false;
            }

            relativeContactDistance = contactDistance;
            _ = Fixed64.TryMultiplyDivide(
                sourceTravelDistance,
                contactDistance,
                totalDistance,
                out sourceContactDistance);
            Fixed64 candidateDistance = contactDistance;
            for (int attempt = 0; attempt < 2; attempt++)
            {
                _ = Fixed64.TryMultiplyDivide(
                    sourceTravelDistance,
                    candidateDistance,
                    totalDistance,
                    out Fixed64 sampleSourceDistance);
                _ = Fixed64.TryMultiplyDivide(
                    targetTravelDistance,
                    candidateDistance,
                    totalDistance,
                    out Fixed64 targetContactDistance);
                _position = sourcePath.GetPointAtDistance(
                    sampleSourceDistance,
                    sourceTravelDistance);
                _rotation = sourceStartRotation;
                Collider.RebuildRuntimeShapeOnly();
                target.SetMixedContinuousCollisionSamplePose(
                    targetPath.GetPointAtDistance(
                        targetContactDistance,
                        targetTravelDistance),
                    targetSegment.StartRotation);
                ContactManifold manifold = Context.CollisionScratch.MixedManifold;
                if (CollisionDetectionMixed.TryBuildManifold(target.Collider, Collider, manifold))
                {
                    // Relative displacement is sphere-minus-circle even when the source is 2D.
                    contact = MixedContact.FromManifoldContact(manifold.PrimaryContact);
                    bool closingResolved = Vector3d.TryDot(
                            relativeDisplacement,
                            contact.Normal3DTo2D,
                            out localClosingSpeed);
                    if (closingResolved
                        && localClosingSpeed > Fixed64.Epsilon)
                    {
                        return true;
                    }

                    if (closingResolved)
                        requiresRotationalFallback = false;

                    break;
                }

                if (candidateDistance >= totalDistance)
                    break;

                candidateDistance = FixedMath.Min(
                    totalDistance,
                    candidateDistance + Fixed64.MinIncrement);
            }

            relativeContactDistance = default;
            sourceContactDistance = default;
            localClosingSpeed = default;
            contact = default;
            return false;
        }
        finally
        {
            _position = originalSourcePosition;
            _rotation = originalSourceRotation;
            Collider.RebuildRuntimeShapeOnly();
            target.SetMixedContinuousCollisionSamplePose(
                originalTargetPosition,
                originalTargetRotation);
        }
    }

    private bool TrySampleMixedRotationalContinuousContact(LSCollider target,
        Fixed64 frameFraction, out MixedContact contact, out bool contactIsClosing)
    {
        ContactManifold manifold = Context.CollisionScratch.MixedManifold;
        contact = default;
        contactIsClosing = false;
        if (!CollisionDetectionMixed.TryBuildManifold(target, Collider, manifold)) return false;
        SolidBody? other = target.Body;
        ContactAnchor center3D = other?.GetCenterOfMassAnchor() ?? ContactAnchor.FromWorldPoint(target.Center);
        var center2D = new ContactAnchor(new Vector3d(_position.X, Collider.MixedSlabCenterY, _position.Y),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -_rotation),
            ExactContactLever2D.ToSpatial(LocalCenterOfMassOffset));
        contactIsClosing = MixedContact.TrySelectClosingContact(manifold, center3D,
            other?.SampleContinuousCollisionLinearVelocity(frameFraction) ?? Vector3d.Zero,
            other?.SampleContinuousCollisionAngularVelocity(frameFraction) ?? Vector3d.Zero,
            center2D, IsKinematic ? SampleContinuousCollisionLinearVelocity(frameFraction) : _linearVelocity,
            IsKinematic ? SampleContinuousCollisionAngularVelocity(frameFraction) : _angularVelocity, out contact, Collider);
        return true;
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
        if (target.IsKinematic)
            target.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

        return (IsKinematic || !target.ShouldOwnContinuousCollisionMovingPair(
                HasContinuousCollisionRotationalMotion))
            && IsValidMixedRotationalContinuousCollisionTarget(target.Collider)
            && (target.IsKinematic
                ? target.HasContinuousCollisionMotion
                : target.IsDynamic);
    }

    private bool TryApplyMixedTranslationalManifoldResponse(SolidBody target,
        Vector2d sourcePosition, Vector3d targetPosition, Fixed64 elapsedTime, Fixed64 remainingTime)
    {
        Fixed64 fraction = ResolveContinuousCollisionFrameFraction(elapsedTime);
        Fixed64 sourceRotation = SampleContinuousCollisionRotation(fraction);
        FixedQuaternion targetRotation = target.SampleContinuousCollisionRotation(fraction);
        Vector2d originalSourcePosition = Position;
        Fixed64 originalSourceRotation = Rotation;
        Vector3d originalTargetPosition = target.Position3d;
        FixedQuaternion originalTargetRotation = target.Rotation;
        try
        {
            SetMixedContinuousCollisionSamplePose(sourcePosition, sourceRotation);
            target.SetMixedContinuousCollisionSamplePose(targetPosition, targetRotation);
            if (!CollisionDetectionMixed.TryBuildManifold(target.Collider, Collider, _rotationalMixedKnownManifold))
                return false;
        }
        finally
        {
            SetMixedContinuousCollisionSamplePose(originalSourcePosition, originalSourceRotation);
            target.SetMixedContinuousCollisionSamplePose(originalTargetPosition, originalTargetRotation);
        }
        if (!TryApplyMixedRotationalContinuousCollisionResponse(target.Collider,
                MixedContact.FromManifoldContact(_rotationalMixedKnownManifold.PrimaryContact), Fixed64.Zero,
                sourcePosition, Vector2d.Zero, sourceRotation, Fixed64.Zero, elapsedTime, remainingTime,
                _rotationalMixedKnownManifold)) return false;
        // The rotational outer loop appends this segment itself; the translational owner needs it here.
        if (HasSolverMobility)
        {
            UpdateContinuousCollisionFrameTrajectory(sourcePosition, _linearVelocity, elapsedTime);
            Context.Physics2D.RefreshContinuousCollisionCandidate(this);
        }
        return true;
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
        Fixed64 remainingTime,
        ContactManifold? manifold = null)
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
        SolidBody? targetResponseBody = target?.HasSolverMobility == true ? target : null;
        SolidBody2D? sourceResponseBody = HasSolverMobility ? this : null;
        ContactAnchor targetCenter = new(targetPosition, targetRotation,
            target?.LocalCenterOfMassOffset ?? Vector3d.Zero);
        Vector3d sourcePosition3D = new(sourcePosition.X, Collider.MixedSlabCenterY, sourcePosition.Y);
        var sourceCenter = new ContactAnchor(sourcePosition3D,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -sourceRotation),
            ExactContactLever2D.ToSpatial(LocalCenterOfMassOffset));
        if (manifold == null)
        {
            _rotationalMixedKnownManifold.BeginUpdate(Context.FrameCount);
            _rotationalMixedKnownManifold.AddContact(default, contact.ToManifoldContact());
            manifold = _rotationalMixedKnownManifold;
        }
        if (!MixedContact.TrySelectClosingContact(manifold, targetCenter, targetLinearVelocity,
                targetAngularVelocity, sourceCenter, sourceLinearVelocity, sourceAngularVelocity, out _, Collider))
            return false;
        var impact = new ContactResponseSnapshot(targetPosition, sourcePosition3D, targetCenter, sourceCenter,
            targetLinearVelocity, targetAngularVelocity, ExactContactLever2D.ToSpatial(sourceLinearVelocity),
            new Vector3d(Fixed64.Zero, -sourceAngularVelocity, Fixed64.Zero), sourceAngularVelocity);
        if (!CollisionResponseMixed.TrySolveManifoldVelocity(targetCollider, Collider, manifold, impact,
                ref _mixedContinuousCollisionImpulses, ref _mixedContinuousCollisionExactImpulses,
                out ContactResponseSnapshot solved)) return false;
        Vector3d postTargetLinearVelocity = solved.LinearA;
        Vector3d postTargetAngularVelocity = solved.AngularA;
        Vector2d postSourceLinearVelocity = solved.LinearB.ToVector2d();
        Fixed64 postSourceAngularVelocity = solved.PlanarAngularB;
        Vector3d targetResolvedPosition = default;
        bool sourceStateAvailable = sourceResponseBody == null
            || CanAppendContinuousCollisionFrameSegment(impactElapsedTime)
                && Context.Physics2D.CanAdmitContinuousCollisionCandidateRefresh(this);
        bool targetStateAvailable = targetResponseBody == null
            || targetResponseBody.CanApplyContinuousCollisionHandoff(targetPosition, targetRotation,
                remainingAfterImpact, out targetResolvedPosition);
        if (!(sourceStateAvailable & targetStateAvailable)) return false;
        if (sourceResponseBody != null)
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

        if (sourceResponseBody != null)
        {
            WakeFromCollision();
            _linearVelocity = postSourceLinearVelocity;
            _angularVelocity = postSourceAngularVelocity;
            RefreshLinearSpeed();
            RefreshAngularSpeed();
        }

        return true;
    }
}
