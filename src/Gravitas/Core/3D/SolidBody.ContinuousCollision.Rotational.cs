//=======================================================================
// SolidBody.ContinuousCollision.Rotational.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using System;

namespace Gravitas;

public partial class SolidBody
{
    private bool TryResolveRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDistance = _angularSpeed * Context.DeltaTime;
        if (angularDistance <= Fixed64.Epsilon)
            return false;

        Fixed64 pivotRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * pivotRadius;
        if (pivotRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= pivotRadius))
        {
            return false;
        }

        Vector3d displacement = proposedPosition - startPosition;
        int hitCount = GatherRotationalContinuousCollisionCandidates(
            startPosition,
            proposedPosition,
            displacement,
            pivotRadius);
        if (hitCount == 0)
            return false;

        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        try
        {
            if (!TryFindEarliestRotationalContinuousCollision(
                    startPosition,
                    displacement,
                    startRotation,
                    proposedRotation,
                    angularDistance,
                    pivotRadius,
                    hitCount,
                    isKinematic: false,
                    out Fixed64 safeTime,
                    out Vector3d contactNormal,
                    out bool hasContact))
            {
                return false;
            }

            proposedPosition = startPosition + displacement * safeTime;
            proposedRotation = IntegrateAngularRotation(startRotation, Context.DeltaTime * safeTime);
            LastContinuousCollisionToiIterationCount++;
            StopRotationalContinuousCollision(hasContact ? contactNormal : Vector3d.Zero);
            return true;
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
        }
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDistance = ResolveKinematicAngularDistanceRadians(startRotation, proposedRotation);
        if (angularDistance <= Fixed64.Epsilon)
            return false;

        Fixed64 pivotRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * pivotRadius;
        if (pivotRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= pivotRadius))
        {
            return false;
        }

        Vector3d displacement = proposedPosition - startPosition;
        int hitCount = GatherRotationalContinuousCollisionCandidates(
            startPosition,
            proposedPosition,
            displacement,
            pivotRadius);
        if (hitCount == 0)
            return false;

        FixedQuaternion targetRotation = proposedRotation;
        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        try
        {
            if (!TryFindEarliestRotationalContinuousCollision(
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    angularDistance,
                    pivotRadius,
                    hitCount,
                    isKinematic: true,
                    out Fixed64 safeTime,
                    out _,
                    out _))
            {
                return false;
            }

            proposedPosition = startPosition + displacement * safeTime;
            proposedRotation = FixedQuaternion.Slerp(startRotation, targetRotation, safeTime).Normalized;
            LastContinuousCollisionToiIterationCount++;
            return true;
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
        }
    }

    internal int GatherRotationalContinuousCollisionCandidates(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Vector3d displacement,
        Fixed64 pivotRadius)
    {
        if (pivotRadius == Fixed64.MaxValue)
            return GatherAllRegisteredRotationalContinuousCollisionCandidates();

        return displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query3D.OverlapSphereAgainstStaticAll(
                startPosition,
                pivotRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                pivotRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);
    }

    private int GatherAllRegisteredRotationalContinuousCollisionCandidates()
    {
        _continuousCollisionHits.FastClear();
        int colliderCount = Context.Physics.ColliderCount;
        _continuousCollisionHits.EnsureCapacity(colliderCount);
        for (int serviceIndex = 0; serviceIndex < colliderCount; serviceIndex++)
        {
            LSCollider target = Context.Physics.GetColliderByServiceIndex(serviceIndex);
            if (!IsValidContinuousCollisionTarget(target))
                continue;

            _continuousCollisionHits.Add(new Physics3DHit(
                target,
                target.Center,
                Vector3d.Zero,
                Fixed64.Zero,
                Vector3d.Zero));
        }

        return _continuousCollisionHits.Count;
    }

    private bool TryFindEarliestRotationalContinuousCollision(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        int hitCount,
        bool isKinematic,
        out Fixed64 safeTime,
        out Vector3d contactNormal,
        out bool hasContact)
    {
        safeTime = Fixed64.Zero;
        contactNormal = Vector3d.Zero;
        hasContact = false;

        bool foundCollision = false;
        Fixed64 earliestTime = Fixed64.One;
        int earliestTargetId = int.MaxValue;
        Vector3d earliestContactNormal = Vector3d.Zero;
        bool earliestHasContact = false;

        for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
        {
            LSCollider target = _continuousCollisionHits[hitIndex].Collider!;
            if (!IsValidContinuousCollisionTarget(target)
                || ColliderSettings.GetCollisionType(Collider.Shape, target.Shape) == CollisionType.None
                || !TryFindEarliestRotationalContinuousCollisionAgainstTarget(
                    target,
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    angularDistance,
                    pivotRadius,
                    isKinematic,
                    out Fixed64 candidateTime,
                    out Vector3d candidateContactNormal,
                    out bool candidateHasContact))
            {
                continue;
            }

            if (!ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                    candidateTime,
                    target.Id,
                    foundCollision,
                    earliestTime,
                    earliestTargetId))
            {
                continue;
            }

            foundCollision = true;
            earliestTime = candidateTime;
            earliestTargetId = target.Id;
            earliestContactNormal = candidateContactNormal;
            earliestHasContact = candidateHasContact;
        }

        if (!foundCollision)
            return false;

        safeTime = earliestTime;
        contactNormal = earliestContactNormal;
        hasContact = earliestHasContact;
        return true;
    }

    private bool TryFindEarliestRotationalContinuousCollisionAgainstTarget(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        bool isKinematic,
        out Fixed64 safeTime,
        out Vector3d contactNormal,
        out bool hasContact)
    {
        safeTime = Fixed64.Zero;
        contactNormal = Vector3d.Zero;
        hasContact = false;
        Span<ContinuousCollisionMath.RotationalInterval> intervals =
            stackalloc ContinuousCollisionMath.RotationalInterval[
                ContinuousCollisionMath.RotationalIntervalMaxDepth + 2];
        int intervalCount = 1;
        int processedNodeCount = 0;
        bool hasKnownContact = false;
        Fixed64 knownContactTime = Fixed64.One;
        Vector3d knownContactNormal = Vector3d.Zero;
        intervals[0] = new ContinuousCollisionMath.RotationalInterval(
            Fixed64.Zero,
            Fixed64.One,
            depth: 0);

        while (intervalCount > 0)
        {
            ContinuousCollisionMath.RotationalInterval interval = intervals[--intervalCount];
            if (hasKnownContact && interval.LowerTime >= knownContactTime)
                continue;

            Fixed64 midpoint = (interval.LowerTime + interval.UpperTime) * Fixed64.Half;
            Fixed64 intervalSpan = interval.UpperTime - interval.LowerTime;
            if (isKinematic)
            {
                SampleKinematicRotationalContinuousPose(
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    midpoint);
            }
            else
            {
                SampleDynamicRotationalContinuousPose(
                    startPosition,
                    displacement,
                    startRotation,
                    midpoint);
            }

            processedNodeCount++;
            bool hasMotionBound = ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                displacement,
                angularDistance,
                pivotRadius,
                intervalSpan,
                out Fixed64 motionBound);
            bool sampleHasContact = TrySampleRotationalContinuousCollision(
                target,
                out Vector3d sampleContactNormal);
            if (sampleHasContact
                && (!hasKnownContact || midpoint < knownContactTime))
            {
                hasKnownContact = true;
                knownContactTime = midpoint;
                knownContactNormal = sampleContactNormal;
            }

            if (!sampleHasContact
                && hasMotionBound
                && IsRotationalIntervalSeparated(target, motionBound))
            {
                continue;
            }

            if (interval.Depth >= ContinuousCollisionMath.RotationalIntervalMaxDepth
                || processedNodeCount >= ContinuousCollisionMath.RotationalIntervalNodeBudget)
            {
                safeTime = interval.LowerTime;
                // The search is target-local. A witnessed later contact remains
                // the valid upper-bound normal when an earlier interval cannot
                // be certified empty within the deterministic work budget.
                contactNormal = hasKnownContact ? knownContactNormal : sampleContactNormal;
                hasContact = hasKnownContact || sampleHasContact;
                return true;
            }

            int childDepth = interval.Depth + 1;
            if (sampleHasContact)
            {
                // A real midpoint contact is already an upper bound. Only the
                // earlier half can contain an earlier first contact.
                intervals[intervalCount++] = new ContinuousCollisionMath.RotationalInterval(
                    interval.LowerTime,
                    midpoint,
                    childDepth);
                continue;
            }

            intervals[intervalCount++] = new ContinuousCollisionMath.RotationalInterval(
                midpoint,
                interval.UpperTime,
                childDepth);
            intervals[intervalCount++] = new ContinuousCollisionMath.RotationalInterval(
                interval.LowerTime,
                midpoint,
                childDepth);
        }

        if (!hasKnownContact)
            return false;

        safeTime = knownContactTime;
        contactNormal = knownContactNormal;
        hasContact = true;
        return true;
    }

    private bool IsRotationalIntervalSeparated(LSCollider target, Fixed64 motionBound)
    {
        if (TryGetSpherePairSeparationGap(target, out Fixed64 closestPointGap))
            return closestPointGap > motionBound;

        return ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(
            Collider.Bounds,
            target.Bounds,
            motionBound);
    }

    private bool TryGetSpherePairSeparationGap(LSCollider target, out Fixed64 separationGap)
    {
        if (Collider is LSSphereCollider sourceSphere)
            return TryGetSphereSeparationGap(sourceSphere, target, out separationGap);

        if (target is LSSphereCollider targetSphere)
            return TryGetSphereSeparationGap(targetSphere, Collider, out separationGap);

        separationGap = default;
        return false;
    }

    private static bool TryGetSphereSeparationGap(
        LSSphereCollider sphere,
        LSCollider other,
        out Fixed64 separationGap)
    {
        Vector3d closestPoint;
        Fixed64 otherRadius;
        if (other is LSSphereCollider otherSphere)
        {
            closestPoint = otherSphere.Center;
            otherRadius = otherSphere.ScaledRadius;
        }
        else if (other is LSCuboidCollider cuboid)
        {
            closestPoint = cuboid.ClosestPointOnSurface(sphere.Center);
            otherRadius = Fixed64.Zero;
        }
        else
        {
            separationGap = default;
            return false;
        }

        if (!Vector3d.TrySubtract(sphere.Center, closestPoint, out Vector3d separation)
            || !Vector3d.TryGetMagnitude(separation, out Fixed64 distance)
            || !Fixed64.TryAdd(sphere.ScaledRadius, otherRadius, out Fixed64 combinedRadius)
            || !Fixed64.TrySubtract(distance, combinedRadius, out Fixed64 rawGap)
            || !Fixed64.TryAdd(distance, combinedRadius, out Fixed64 characteristicScale)
            || !Fixed64.TryAdd(characteristicScale, other.ScaledRadius, out characteristicScale)
            || !ContinuousCollisionMath.TrySubtractClosestFeatureUncertainty(
                rawGap,
                characteristicScale,
                out separationGap))
        {
            separationGap = default;
            return false;
        }

        return separationGap > Fixed64.Zero;
    }

    private void SampleDynamicRotationalContinuousPose(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        Fixed64 sampleTime)
    {
        Position3d = startPosition + displacement * sampleTime;
        Rotation = IntegrateAngularRotation(startRotation, Context.DeltaTime * sampleTime);
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
    }

    private void SampleKinematicRotationalContinuousPose(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 sampleTime)
    {
        Position3d = startPosition + displacement * sampleTime;
        Rotation = FixedQuaternion.Slerp(startRotation, targetRotation, sampleTime).Normalized;
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
    }

    private static Fixed64 ResolveKinematicAngularDistanceRadians(
        FixedQuaternion startRotation,
        FixedQuaternion proposedRotation)
    {
        Fixed64 angleDegrees = FixedQuaternion.Angle(startRotation, proposedRotation);
        return FixedMath.DegToRad(angleDegrees.Abs());
    }

    private bool TrySampleRotationalContinuousCollision(LSCollider target, out Vector3d contactNormal)
    {
        contactNormal = Vector3d.Zero;
        OrderRotationalContinuousCollisionPair(
            target,
            out LSCollider colliderA,
            out LSCollider colliderB,
            out bool sourceIsA);
        CollisionType collisionType = ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape);
        _rotationalContinuousCollisionManifold.BeginUpdate(Context.FrameCount);
        var workItem = new CollisionWorkItem(
            Context,
            colliderA,
            colliderB,
            collisionType,
            _rotationalContinuousCollisionManifold);
        if (!CollisionDetection.DoCollisionCheck(workItem)
            || !_rotationalContinuousCollisionManifold.HasContact)
        {
            return false;
        }

        contactNormal = _rotationalContinuousCollisionManifold.PrimaryContact.Normal;
        if (!sourceIsA)
            contactNormal = -contactNormal;

        return true;
    }

    private void OrderRotationalContinuousCollisionPair(
        LSCollider target,
        out LSCollider colliderA,
        out LSCollider colliderB,
        out bool sourceIsA)
    {
        if (Collider.Priority >= target.Priority)
        {
            colliderA = Collider;
            colliderB = target;
            sourceIsA = true;
            return;
        }

        colliderA = target;
        colliderB = Collider;
        sourceIsA = false;
    }

    private FixedQuaternion IntegrateAngularRotation(FixedQuaternion startRotation, Fixed64 deltaTime)
    {
        FixedQuaternion angularVelocityQuaternion = new(
            _angularVelocity.X,
            _angularVelocity.Y,
            _angularVelocity.Z,
            Fixed64.Zero);
        FixedQuaternion spin = angularVelocityQuaternion * startRotation * Fixed64.Half * deltaTime;
        return (startRotation + spin).Normalized;
    }

    private void StopRotationalContinuousCollision(Vector3d contactNormal)
    {
        Vector3d lastVelocity = _angularVelocity;
        _angularVelocity = Vector3d.Zero;
        _angularDirection = Vector3d.Zero;
        _angularAccelerationStore = Vector3d.Zero;
        _angularAcceleration = Vector3d.Zero;
        _deltaTorque = Vector3d.Zero;
        RefreshAngularMotionState(lastVelocity);
        Context.Diagnostics.EmitAngularVelocityDelta(this, lastVelocity, _angularVelocity);
        RemoveClosingContinuousCollisionVelocity(contactNormal);
    }
}
