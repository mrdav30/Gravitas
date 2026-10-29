//=======================================================================
// SolidBody.ContinuousCollision.Rotational.Mixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
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
    private readonly ContactManifold _rotationalMixedKnownManifold = new();
    private readonly ContactManifold _rotationalMixedEarliestManifold = new();
    private bool _rotationalMixedKnownContactIsClosing;
    private bool _rotationalMixedEarliestContactIsClosing;
    private ContactWarmStartCache _mixedContinuousCollisionImpulses;
    private ExactContactImpulseState?[]? _mixedContinuousCollisionExactImpulses;
    private SwiftHashSet<StaticSurface2D>? _rotationalMixedSurfaceCandidates;

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
            bool hasClosingContact,
            Fixed64 contactTime,
            LSCollider2D target,
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

        public LSCollider2D Target { get; }
        public ContactManifold Manifold { get; }
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
        bool reuseCandidates,
        out RotationalMixed2DHit hit)
    {
        _rotationalMixedSurfaceCandidates?.Clear();
        try
        {
            hit = default;
            if (!Context.Settings.RuntimeMode.RunsMixedContacts())
                return false;

            if (!reuseCandidates)
                GatherMixedRotationalContinuousCollisionCandidates(
                    startPosition,
                    displacement,
                    pivotRadius);

            bool found = false;
            Fixed64 earliestTime = Fixed64.One;
            int earliestOrder = int.MaxValue;
            MixedContact earliestContact = default;
            bool earliestHasContact = false;
            Fixed64 earliestContactTime = Fixed64.Zero;
            LSCollider2D? earliestTarget = null;

            for (int i = 0; i < _rotationalMixedStatic2DCandidates.Count; i++)
            {
                LSCollider2D target = _rotationalMixedStatic2DCandidates[i];
                if (target.StaticSurface is { } surface
                    && (!IsValidMixedRotationalContinuousCollisionTarget(target)
                        || !(_rotationalMixedSurfaceCandidates ??= new()).Add(surface))) continue;
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
                    ref earliestOrder,
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
                    ref earliestOrder,
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
                _rotationalMixedEarliestContactIsClosing,
                earliestContactTime,
                earliestTarget!,
                _rotationalMixedEarliestManifold);
            return true;
        }
        finally { _rotationalMixedSurfaceCandidates?.Clear(); }
    }

    private void GatherMixedRotationalContinuousCollisionCandidates(
        Vector3d startPosition,
        Vector3d displacement,
        Fixed64 pivotRadius)
    {
        _rotationalMixedStatic2DCandidates.FastClear();
        _rotationalMixedMoving2DCandidateIds.FastClear();
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
        _rotationalMixedStatic2DCandidates.SortInPlace(default(ColliderRegistry<LSCollider2D>.ReplayOrderComparer));

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
        ref int earliestOrder,
        ref MixedContact earliestContact,
        ref bool earliestHasContact,
        ref Fixed64 earliestContactTime,
        ref LSCollider2D? earliestTarget)
    {
        // Kinematic clamps against immovable targets consume geometry/time only;
        // dynamic stopping and movable-target response still require full contacts.
        int candidateOrder = target.StaticSurface?.ContinuousOrder ?? ((IPhysicsColliderRegistryItem)target).ReplayOrder;
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
                Fixed64.Zero,
                Fixed64.One,
                needsResponseContact: !IsKinematic || target.Body?.HasSolverMobility == true,
                out Fixed64 candidateTime,
                out MixedContact candidateContact,
                out bool candidateHasContact,
                out Fixed64 candidateContactTime)
            || !ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateTime,
                candidateOrder,
                found,
                earliestTime,
                earliestOrder))
        {
            return;
        }

        found = true;
        earliestTime = candidateTime;
        earliestOrder = candidateOrder;
        earliestContact = candidateContact;
        earliestHasContact = candidateHasContact;
        earliestContactTime = candidateContactTime;
        earliestTarget = target;
        _rotationalMixedEarliestManifold.CopyFrom(_rotationalMixedKnownManifold, Context.FrameCount);
        _rotationalMixedEarliestContactIsClosing = _rotationalMixedKnownContactIsClosing;
    }

    private bool TryGetInvariantMixedPlane(Vector3d initialNormal,
        Vector3d displacement, FixedQuaternion startRotation, FixedQuaternion targetRotation, bool isKinematic,
        out Vector3d outward, out RationalPoint3d plane)
    {
        outward = default; plane = default;
        if (Collider is not LSCapsuleCollider) return false;
        System.Diagnostics.Debug.Assert(initialNormal != Vector3d.Zero);
        outward = -initialNormal;
        if (!HasInvariantContinuousProjection(outward, displacement, startRotation, targetRotation, isKinematic))
            return false;
        // The source is already sampled at T0. This exact minimum remains
        // unchanged by rotation and cannot move toward any member behind it.
        // Penetrating members never qualify for this separation certificate.
        plane = RationalPoint3d.FromAnchor(new ConvexShape(Collider, Vector3d.Zero).GetSupportAnchor(-outward));
        return true;
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
        Fixed64 searchStart,
        Fixed64 searchEnd,
        bool needsResponseContact,
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
        StaticSurface2D? surface = target.StaticSurface;
        Vector3d? separationOutward = null;
        RationalPoint3d separationPlane = default;
        SolidBody2D? targetBody = target.Body;
        bool samplesTargetMotion = targetBody != null
            && IsMovingMixedRotationalContinuousCollisionTarget(targetBody);
        Vector2d originalTargetPosition = targetBody?.Position ?? Vector2d.Zero;
        Fixed64 originalTargetRotation = targetBody?.Rotation ?? Fixed64.Zero;
        if (samplesTargetMotion)
            targetBody!.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

        try
        {
            SampleMixedRotationalContinuousPairPose(startPosition, displacement, startRotation, targetRotation,
                searchStart, elapsedTime, remainingTime, sourceIsKinematic, targetBody, samplesTargetMotion);
            // A touching start can already supply a closing contact. Waiting
            // for the first midpoint would invent penetration before response.
            if (needsResponseContact && (surface?.IntersectsMixedContinuous(Collider)
                    ?? CollisionDetectionMixed.IntersectsForContinuousCollision(Collider, target)))
            {
                BuildMixedRotationalContinuousContact(target,
                    ResolveRotationalFrameFraction(elapsedTime, remainingTime, searchStart),
                    out contact, out _rotationalMixedKnownContactIsClosing);
                ContactManifold initial = Context.CollisionScratch.MixedManifold;
                if (_rotationalMixedKnownContactIsClosing)
                {
                    _rotationalMixedKnownManifold.CopyFrom(initial, Context.FrameCount);
                    safeTime = contactTime = searchStart;
                    hasContact = true;
                    return true;
                }
                if (surface != null && initial.HasContact
                    && TryGetInvariantMixedPlane(initial.PrimaryContact.Normal, displacement, startRotation,
                        targetRotation, sourceIsKinematic, out Vector3d axis, out separationPlane)) separationOutward = axis;
            }
            Span<ContinuousCollisionMath.RotationalInterval> intervals =
                stackalloc ContinuousCollisionMath.RotationalInterval[
                    ContinuousCollisionMath.RotationalIntervalMaxDepth + 2];
            int intervalCount = 1;
            int processedNodeCount = 0;
            bool hasKnownContact = false;
            Fixed64 knownContactTime = Fixed64.One;
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

                bool sampleHasContact = surface?.IntersectsMixedContinuous(Collider, separationOutward: separationOutward, separationPlane: separationPlane)
                    ?? CollisionDetectionMixed.IntersectsForContinuousCollision(Collider, target);
                if (sampleHasContact
                    && (!hasKnownContact || midpoint < knownContactTime))
                {
                    hasKnownContact = true;
                    knownContactTime = midpoint;
                }

                Fixed64 shapeGap = default;
                bool shapeGapSupported = false;
                bool hasShapeGap = surface == null && CollisionDetectionMixed.TryGetRotationalSeparationGap(
                    Collider,
                    target,
                    out shapeGap,
                    out shapeGapSupported);
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
                bool fallbackBoundsCertifySeparation = !sampleHasContact && hasMotionBound && !shapeGapSupported
                    && (surface != null ? !surface.IntersectsMixedContinuous(Collider, motionBound, separationOutward, separationPlane)
                        : ContinuousCollisionBounds.AreSeparated(Collider, target, motionBound)
                            || !CollisionDetectionMixed.IntersectsForContinuousCollision(Collider, target, motionBound));
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
                    if (hasContact && needsResponseContact)
                    {
                        // A budget-limited safe frontier can precede the retained witness.
                        // Rebuild at contactTime, preserving its complete matched manifold.
                        SampleMixedRotationalContinuousPairPose(startPosition, displacement, startRotation, targetRotation, contactTime,
                            elapsedTime, remainingTime, sourceIsKinematic, targetBody, samplesTargetMotion);
                        BuildMixedRotationalContinuousContact(target,
                            ResolveRotationalFrameFraction(elapsedTime, remainingTime, contactTime),
                            out contact, out _rotationalMixedKnownContactIsClosing, separationOutward, separationPlane);
                        _rotationalMixedKnownManifold.CopyFrom(Context.CollisionScratch.MixedManifold, Context.FrameCount);
                    }
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

    private bool TryGetExactSphereCircleTranslationalContact(
        SolidBody2D target,
        ContinuousCollisionMotionSegment2D targetSegment,
        FixedSegment2d targetPath,
        Vector2d targetDisplacement,
        Fixed64 targetTravelDistance,
        FixedSegment sourcePath,
        Vector3d sourceDisplacement,
        Fixed64 sourceTravelDistance,
        FixedQuaternion sourceStartRotation,
        Fixed64 sourceAngularDistance,
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
        if (Collider is not LSSphereCollider sphere
            || target.Collider is not LSCircleCollider2D circle
            || (targetSegment.AngularDistance != Fixed64.Zero
                && circle.ScaledLocalOffset != Vector2d.Zero)
            || (sourceAngularDistance != Fixed64.Zero
                && sphere.ScaledOffset != Vector3d.Zero))
        {
            return false;
        }

        Vector3d originalSourcePosition = Position3d;
        FixedQuaternion originalSourceRotation = Rotation;
        Vector2d originalTargetPosition = target.Position;
        Fixed64 originalTargetRotation = target.Rotation;
        try
        {
            SetMixedContinuousCollisionSamplePose(
                sourcePath.Start,
                sourceStartRotation);
            target.SetMixedContinuousCollisionSamplePose(
                targetPath.Start,
                targetSegment.StartRotation);
            Vector3d sphereStart = sphere.Center;
            Vector3d cylinderStart = new(
                circle.Center.X,
                circle.MixedSlabCenterY,
                circle.Center.Y);
            Vector3d cylinderDisplacement =
                targetDisplacement.ToVector3d(Fixed64.Zero);
            Vector3d relativeDisplacement =
                ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
                    sourceDisplacement,
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
                SetMixedContinuousCollisionSamplePose(
                    sourcePath.GetPointAtDistance(
                        sampleSourceDistance,
                        sourceTravelDistance),
                    sourceStartRotation);
                target.SetMixedContinuousCollisionSamplePose(
                    targetPath.GetPointAtDistance(
                        targetContactDistance,
                        targetTravelDistance),
                    targetSegment.StartRotation);
                ContactManifold manifold = Context.CollisionScratch.MixedManifold;
                if (CollisionDetectionMixed.TryBuildManifold(Collider, target.Collider, manifold))
                {
                    // This analytic owner admits only a single sphere/circle leaf pair.
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
            SetMixedContinuousCollisionSamplePose(
                originalSourcePosition,
                originalSourceRotation);
            target.SetMixedContinuousCollisionSamplePose(
                originalTargetPosition,
                originalTargetRotation);
        }
    }

    private void BuildMixedRotationalContinuousContact(LSCollider2D target,
        Fixed64 frameFraction, out MixedContact contact, out bool contactIsClosing,
        Vector3d? separationOutward = null, RationalPoint3d separationPlane = default)
    {
        ContactManifold manifold = Context.CollisionScratch.MixedManifold;
        contact = default;
        contactIsClosing = false;
        // Admission already retained this geometric witness; closing rows only authorize response.
        if (target.StaticSurface is { } surface)
        {
            manifold.BeginUpdate(Context.FrameCount);
            surface.AppendMixedContinuousContacts(Collider, manifold, separationOutward, separationPlane);
        }
        else CollisionDetectionMixed.TryBuildDiscreteManifold(Collider, target, manifold);
        SolidBody2D? other = target.Body;
        ContactAnchor center2D = other == null
            ? ContactAnchor.FromWorldPoint(MixedEmbedded2DGeometry.GetCenter3D(target))
            : new ContactAnchor(new Vector3d(other.Position.X, target.MixedSlabCenterY, other.Position.Y),
                FixedQuaternion.FromAxisAngle(Vector3d.Up, -other.Rotation),
                ExactContactLever2D.ToSpatial(other.LocalCenterOfMassOffset));
        contactIsClosing = MixedContact.TrySelectClosingContact(manifold, GetCenterOfMassAnchor(),
            IsKinematic ? SampleContinuousCollisionLinearVelocity(frameFraction) : _linearVelocity,
            IsKinematic ? SampleContinuousCollisionAngularVelocity(frameFraction) : _angularVelocity,
            center2D, other?.SampleContinuousCollisionLinearVelocity(frameFraction) ?? Vector2d.Zero,
            other?.SampleContinuousCollisionAngularVelocity(frameFraction) ?? Fixed64.Zero, out contact, target);
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

    private bool TryApplyMixedTranslationalManifoldResponse(SolidBody2D target,
        Vector3d sourcePosition, Vector2d targetPosition, Fixed64 elapsedTime, Fixed64 remainingTime)
    {
        Fixed64 fraction = ResolveContinuousCollisionFrameFraction(elapsedTime);
        FixedQuaternion sourceRotation = SampleContinuousCollisionRotation(fraction);
        Fixed64 targetRotation = target.SampleContinuousCollisionRotation(fraction);
        Vector3d originalSourcePosition = Position3d;
        FixedQuaternion originalSourceRotation = Rotation;
        Vector2d originalTargetPosition = target.Position;
        Fixed64 originalTargetRotation = target.Rotation;
        try
        {
            // Sample geometry only; velocity and trajectory changes remain atomic after the pure solve.
            SetMixedContinuousCollisionSamplePose(sourcePosition, sourceRotation);
            target.SetMixedContinuousCollisionSamplePose(targetPosition, targetRotation);
            if (!CollisionDetectionMixed.TryBuildManifold(Collider, target.Collider, _rotationalMixedKnownManifold))
                return false;
        }
        finally
        {
            SetMixedContinuousCollisionSamplePose(originalSourcePosition, originalSourceRotation);
            target.SetMixedContinuousCollisionSamplePose(originalTargetPosition, originalTargetRotation);
        }
        return TryApplyMixedRotationalContinuousCollisionResponse(target.Collider,
            MixedContact.FromManifoldContact(_rotationalMixedKnownManifold.PrimaryContact), Fixed64.Zero,
            sourcePosition, Vector3d.Zero, sourceRotation, sourceRotation, elapsedTime, remainingTime,
            IsKinematic, _rotationalMixedKnownManifold);
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
        bool sourceIsKinematic,
        ContactManifold? manifold = null)
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
        Vector2d targetLocalCenterOfMassOffset =
            target?.LocalCenterOfMassOffset ?? Vector2d.Zero;
        SolidBody? sourceResponseBody = HasSolverMobility ? this : null;
        SolidBody2D? targetResponseBody = target?.HasSolverMobility == true ? target : null;
        var sourceCenter = new ContactAnchor(sourcePosition, sourceRotation, LocalCenterOfMassOffset);
        Vector3d targetPosition3D = new(targetPosition.X, targetCollider.MixedSlabCenterY, targetPosition.Y);
        var targetCenter = new ContactAnchor(targetPosition3D,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -targetRotation),
            ExactContactLever2D.ToSpatial(targetLocalCenterOfMassOffset));
        if (manifold == null)
        {
            // Direct one-row callers retain the same response contract without publishing a runtime pair.
            _rotationalMixedKnownManifold.BeginUpdate(Context.FrameCount);
            _rotationalMixedKnownManifold.AddContact(default, contact.ToManifoldContact());
            manifold = _rotationalMixedKnownManifold;
        }
        if (!MixedContact.TrySelectClosingContact(manifold, sourceCenter, sourceLinearVelocity,
                sourceAngularVelocity, targetCenter, targetLinearVelocity, targetAngularVelocity, out _, targetCollider))
            return false;
        var impact = new ContactResponseSnapshot(sourcePosition, targetPosition3D, sourceCenter, targetCenter,
            sourceLinearVelocity, sourceAngularVelocity, ExactContactLever2D.ToSpatial(targetLinearVelocity),
            new Vector3d(Fixed64.Zero, -targetAngularVelocity, Fixed64.Zero), targetAngularVelocity);
        if (!CollisionResponseMixed.TrySolveManifoldVelocity(Collider, targetCollider, manifold, impact,
                ref _mixedContinuousCollisionImpulses, ref _mixedContinuousCollisionExactImpulses,
                out ContactResponseSnapshot solved)) return false;
        Vector3d postSourceLinearVelocity = solved.LinearA;
        Vector3d postSourceAngularVelocity = solved.AngularA;
        Vector2d postTargetLinearVelocity = solved.LinearB.ToVector2d();
        Fixed64 postTargetAngularVelocity = solved.PlanarAngularB;
        bool sourceStateAvailable = sourceResponseBody == null
            || CanAppendContinuousCollisionSegment(impactElapsedTime);
        bool targetStateAvailable = targetResponseBody == null
            || targetResponseBody.CanApplyContinuousCollisionHandoffState(
                targetPosition, targetRotation, remainingAfterImpact, out targetPosition);
        if (!(sourceStateAvailable & targetStateAvailable))
        {
            return false;
        }

        if (sourceResponseBody != null)
            ValidateContinuousCollisionPosition(sourcePosition, postSourceLinearVelocity, Context.DeltaTime - impactElapsedTime);
        if (targetResponseBody != null)
            targetResponseBody.ValidateContinuousCollisionHandoffPosition(targetPosition, postTargetLinearVelocity, postTargetAngularVelocity, remainingAfterImpact);

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
