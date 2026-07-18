//=======================================================================
// SolidBody2D.ContinuousCollision.Rotational.cs
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

public sealed partial class SolidBody2D
{
    private bool TryResolveRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation)
    {
        if (!CanRotate || !ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDelta = proposedRotation - startRotation;
        Fixed64 angularDistance = angularDelta.Abs();
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

        Vector2d displacement = proposedPosition - startPosition;
        int hitCount = GatherRotationalContinuousCollisionCandidates(
            startPosition,
            proposedPosition,
            displacement,
            pivotRadius);
        if (hitCount == 0)
            return false;

        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
        try
        {
            if (!TryFindEarliestRotationalContinuousCollision(
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    angularDistance,
                    pivotRadius,
                    hitCount,
                    out Fixed64 safeTime,
                    out Contact2D contact,
                    out bool hasContact))
            {
                return false;
            }

            proposedPosition = startPosition + displacement * safeTime;
            proposedRotation = startRotation + angularDelta * safeTime;
            LastContinuousCollisionToiIterationCount++;
            StopRotationalContinuousCollision(hasContact ? contact.Normal : Vector2d.Zero);
            return true;
        }
        finally
        {
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
        }
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDelta = CanonicalizeRotation(proposedRotation - startRotation);
        Fixed64 angularDistance = angularDelta.Abs();
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

        Vector2d displacement = proposedPosition - startPosition;
        int hitCount = GatherRotationalContinuousCollisionCandidates(
            startPosition,
            proposedPosition,
            displacement,
            pivotRadius);
        if (hitCount == 0)
            return false;

        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
        try
        {
            if (!TryFindEarliestRotationalContinuousCollision(
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    angularDistance,
                    pivotRadius,
                    hitCount,
                    out Fixed64 safeTime,
                    out _,
                    out _))
            {
                return false;
            }

            proposedPosition = startPosition + displacement * safeTime;
            proposedRotation = startRotation + angularDelta * safeTime;
            LastContinuousCollisionToiIterationCount++;
            return true;
        }
        finally
        {
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
        }
    }

    internal int GatherRotationalContinuousCollisionCandidates(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Vector2d displacement,
        Fixed64 pivotRadius)
    {
        if (pivotRadius == Fixed64.MaxValue)
            return GatherAllRegisteredRotationalContinuousCollisionCandidates();

        return displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query2D.OverlapCircleAgainstStaticAll(
                startPosition,
                pivotRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query2D.SweepCircleAgainstStaticAll(
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
        int colliderCount = Context.Physics2D.ColliderCount;
        _continuousCollisionHits.EnsureCapacity(colliderCount);
        for (int serviceIndex = 0; serviceIndex < colliderCount; serviceIndex++)
        {
            LSCollider2D target = Context.Physics2D.GetColliderByServiceIndex(serviceIndex);
            if (!IsValidContinuousCollisionTarget(target))
                continue;

            _continuousCollisionHits.Add(new Physics2DHit(
                target,
                target.Center,
                Vector2d.Zero,
                Fixed64.Zero));
        }

        return _continuousCollisionHits.Count;
    }

    private bool TryFindEarliestRotationalContinuousCollision(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        int hitCount,
        out Fixed64 safeTime,
        out Contact2D contact,
        out bool hasContact)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;

        bool foundCollision = false;
        Fixed64 earliestTime = Fixed64.One;
        int earliestTargetId = int.MaxValue;
        Contact2D earliestContact = default;
        bool earliestHasContact = false;

        for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
        {
            LSCollider2D target = _continuousCollisionHits[hitIndex].Collider;
            if (!IsValidContinuousCollisionTarget(target)
                || ColliderSettings2D.GetCollisionType(Collider.Shape, target.Shape) == CollisionType2D.None
                || !TryFindEarliestRotationalContinuousCollisionAgainstTarget(
                    target,
                    startPosition,
                    displacement,
                    startRotation,
                    angularDelta,
                    angularDistance,
                    pivotRadius,
                    out Fixed64 candidateTime,
                    out Contact2D candidateContact,
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
            earliestContact = candidateContact;
            earliestHasContact = candidateHasContact;
        }

        if (!foundCollision)
            return false;

        safeTime = earliestTime;
        contact = earliestContact;
        hasContact = earliestHasContact;
        return true;
    }

    private bool TryFindEarliestRotationalContinuousCollisionAgainstTarget(
        LSCollider2D target,
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        out Fixed64 safeTime,
        out Contact2D contact,
        out bool hasContact)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        Span<ContinuousCollisionMath.RotationalInterval> intervals =
            stackalloc ContinuousCollisionMath.RotationalInterval[
                ContinuousCollisionMath.RotationalIntervalMaxDepth + 2];
        int intervalCount = 1;
        int processedNodeCount = 0;
        bool hasKnownContact = false;
        Fixed64 knownContactTime = Fixed64.One;
        Contact2D knownContact = default;
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
            SampleRotationalContinuousPose(
                startPosition,
                displacement,
                startRotation,
                angularDelta,
                midpoint);
            processedNodeCount++;

            bool hasMotionBound = ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                displacement,
                angularDistance,
                pivotRadius,
                intervalSpan,
                out Fixed64 motionBound);
            bool sampleHasContact = CollisionDetection2D.TryCollide(
                Collider,
                target,
                out Contact2D sampleContact);
            if (sampleHasContact
                && (!hasKnownContact || midpoint < knownContactTime))
            {
                hasKnownContact = true;
                knownContactTime = midpoint;
                knownContact = sampleContact;
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
                contact = hasKnownContact ? knownContact : sampleContact;
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
        contact = knownContact;
        hasContact = true;
        return true;
    }

    private bool IsRotationalIntervalSeparated(LSCollider2D target, Fixed64 motionBound)
    {
        if (TryGetCirclePairSeparationGap(target, out Fixed64 closestPointGap))
            return closestPointGap > motionBound;

        return ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(
            Collider.Bounds,
            target.Bounds,
            motionBound);
    }

    private bool TryGetCirclePairSeparationGap(LSCollider2D target, out Fixed64 separationGap)
    {
        if (Collider is LSCircleCollider2D sourceCircle)
            return TryGetCircleSeparationGap(sourceCircle, target, out separationGap);

        if (target is LSCircleCollider2D targetCircle)
            return TryGetCircleSeparationGap(targetCircle, Collider, out separationGap);

        separationGap = default;
        return false;
    }

    private static bool TryGetCircleSeparationGap(
        LSCircleCollider2D circle,
        LSCollider2D other,
        out Fixed64 separationGap)
    {
        Vector2d closestPoint;
        Fixed64 otherRadius;
        switch (other)
        {
            case LSCircleCollider2D otherCircle:
                closestPoint = otherCircle.Center;
                otherRadius = otherCircle.ScaledRadius;
                break;
            case LSAABBoxCollider2D:
                closestPoint = other.GetClosestPoint(circle.Center);
                otherRadius = Fixed64.Zero;
                break;
            case LSCapsuleCollider2D:
            case LSPolygonCollider2D:
                closestPoint = other.GetClosestPoint(circle.Center);
                otherRadius = Fixed64.Zero;
                break;
            default:
                separationGap = default;
                return false;
        }

        if (!Vector2d.TrySubtract(circle.Center, closestPoint, out Vector2d separation)
            || !Vector2d.TryGetMagnitude(separation, out Fixed64 distance)
            || !Fixed64.TryAdd(circle.ScaledRadius, otherRadius, out Fixed64 combinedRadius)
            || !Fixed64.TrySubtract(distance, combinedRadius, out Fixed64 rawGap)
            || !Vector2d.TryGetMagnitude(other.Bounds.Size, out Fixed64 boundsDiagonal)
            || !Fixed64.TryAdd(distance, combinedRadius, out Fixed64 characteristicScale)
            || !Fixed64.TryAdd(characteristicScale, boundsDiagonal, out characteristicScale)
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

    private void SampleRotationalContinuousPose(
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 sampleTime)
    {
        _position = startPosition + displacement * sampleTime;
        _rotation = startRotation + angularDelta * sampleTime;
        Collider.RebuildRuntimeShapeOnly();
    }

    private void StopRotationalContinuousCollision(Vector2d contactNormal)
    {
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        RefreshAngularSpeed();
        RemoveClosingContinuousCollisionVelocity(contactNormal);
    }
}
