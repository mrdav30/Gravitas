//=======================================================================
// SolidBody.ContinuousCollision.Rotational.Search.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas;

public partial class SolidBody
{
    internal bool HasNearbyRotationalContinuousCollisionTarget(
        Vector3d startPosition,
        Vector3d displacement,
        Fixed64 pivotRadius)
    {
        if (pivotRadius == Fixed64.MaxValue)
        {
            int colliderCount = Context.Physics.ColliderCount;
            for (int i = 0; i < colliderCount; i++)
            {
                if (IsRotatingContinuousCollisionTarget(
                    Context.Physics.GetColliderByServiceIndex(i).Body))
                {
                    return true;
                }
            }

            if (Context.Settings.RuntimeMode.RunsMixedContacts())
            {
                int mixedColliderCount = Context.Physics2D.ColliderCount;
                for (int i = 0; i < mixedColliderCount; i++)
                {
                    if (IsRotatingContinuousCollisionTarget(
                        Context.Physics2D.GetColliderByServiceIndex(i).Body))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        FixedBoundVolume bounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            startPosition,
            displacement,
            pivotRadius);
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(bounds);
        for (int i = 0; i < candidateIds.Count; i++)
        {
            SolidBody target = Context.Physics.GetContinuousCollisionCandidate(candidateIds[i]);
            if (IsRotatingContinuousCollisionTarget(target))
            {
                return true;
            }
        }

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        candidateIds = Context.Physics2D.QueryMixedContinuousCollisionCandidates(bounds);
        for (int i = 0; i < candidateIds.Count; i++)
        {
            SolidBody2D target = Context.Physics2D.GetContinuousCollisionCandidate(candidateIds[i]);
            if (IsRotatingContinuousCollisionTarget(target))
                return true;
        }

        return false;
    }

    private bool IsRotatingContinuousCollisionTarget(SolidBody? target)
    {
        if (target == null
            || ReferenceEquals(target, this)
            || !IsMovingRotationalContinuousCollisionTarget(target))
        {
            return false;
        }

        target.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        return !target.HasRotationInvariantCollider && target.HasContinuousCollisionRotationalMotion;
    }

    private bool IsRotatingContinuousCollisionTarget(SolidBody2D? target)
    {
        if (target == null || !IsMovingMixedRotationalContinuousCollisionTarget(target))
            return false;

        target.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        return !target.HasRotationInvariantCollider && target.HasContinuousCollisionRotationalMotion;
    }

    internal int GatherRotationalContinuousCollisionCandidates(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Vector3d displacement,
        Fixed64 pivotRadius)
    {
        _rotationalContinuousCollisionCandidateIds.FastClear();
        if (pivotRadius == Fixed64.MaxValue)
            return GatherAllRegisteredRotationalContinuousCollisionCandidates();

        int staticHitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
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

        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                startPosition,
                displacement,
                pivotRadius));
        _rotationalContinuousCollisionCandidateIds.EnsureCapacity(candidateIds.Count);
        for (int i = 0; i < candidateIds.Count; i++)
            _rotationalContinuousCollisionCandidateIds.Add(candidateIds[i]);

        return staticHitCount + _rotationalContinuousCollisionCandidateIds.Count;
    }

    private int GatherAllRegisteredRotationalContinuousCollisionCandidates()
    {
        _continuousCollisionHits.FastClear();
        _rotationalContinuousCollisionCandidateIds.FastClear();
        int colliderCount = Context.Physics.ColliderCount;
        _continuousCollisionHits.EnsureCapacity(colliderCount);
        for (int serviceIndex = 0; serviceIndex < colliderCount; serviceIndex++)
        {
            LSCollider target = Context.Physics.GetColliderByServiceIndex(serviceIndex);
            if (target.Body is SolidBody targetBody
                && IsMovingRotationalContinuousCollisionTarget(targetBody))
            {
                _rotationalContinuousCollisionCandidateIds.Add(targetBody.DynamicId);
                continue;
            }

            if (!IsValidContinuousCollisionTarget(target))
                continue;

            _continuousCollisionHits.Add(new Physics3DHit(
                target,
                target.Center,
                Vector3d.Zero,
                Fixed64.Zero,
                Vector3d.Zero));
        }

        return _continuousCollisionHits.Count
            + _rotationalContinuousCollisionCandidateIds.Count;
    }

    private bool TryFindEarliestRotationalContinuousCollision(
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool isKinematic,
        LSCollider? ignoredTarget,
        out Fixed64 safeTime,
        out ManifoldContact contact,
        out bool hasContact,
        out Fixed64 contactTime,
        out LSCollider? hitTarget)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        hitTarget = null;

        bool foundCollision = false;
        Fixed64 earliestTime = Fixed64.One;
        int earliestTargetId = int.MaxValue;
        ManifoldContact earliestContact = default;
        bool earliestHasContact = false;
        Fixed64 earliestContactTime = Fixed64.Zero;
        LSCollider? earliestTarget = null;

        for (int hitIndex = 0; hitIndex < _continuousCollisionHits.Count; hitIndex++)
        {
            Physics3DHit hit = _continuousCollisionHits[hitIndex];
            LSCollider target = hit.Collider!;
            if (target == ignoredTarget
                || !IsValidContinuousCollisionTarget(target)
                || ColliderSettings.GetCollisionType(Collider.Shape, target.Shape) == CollisionType.None
                || CanExcludeInvariantTangentialContact(hit, displacement)
                || !TryFindEarliestRotationalContinuousCollisionAgainstTarget(
                    target,
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    isKinematic,
                    out Fixed64 candidateTime,
                    out ManifoldContact candidateContact,
                    out bool candidateHasContact,
                    out Fixed64 candidateContactTime))
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
            earliestContactTime = candidateContactTime;
            earliestTarget = target;
        }

        for (int candidateIndex = 0;
            candidateIndex < _rotationalContinuousCollisionCandidateIds.Count;
            candidateIndex++)
        {
            int dynamicId = _rotationalContinuousCollisionCandidateIds[candidateIndex];
            SolidBody targetBody = Context.Physics.GetContinuousCollisionCandidate(dynamicId);
            if (targetBody.Collider == ignoredTarget
                || !IsMovingRotationalContinuousCollisionTarget(targetBody)
                || ColliderSettings.GetCollisionType(
                    Collider.Shape,
                    targetBody.Collider.Shape) == CollisionType.None
                || !TryFindEarliestRotationalContinuousCollisionAgainstTarget(
                    targetBody.Collider,
                    startPosition,
                    displacement,
                    startRotation,
                    targetRotation,
                    angularDistance,
                    pivotRadius,
                    elapsedTime,
                    remainingTime,
                    isKinematic,
                    out Fixed64 candidateTime,
                    out ManifoldContact candidateContact,
                    out bool candidateHasContact,
                    out Fixed64 candidateContactTime))
            {
                continue;
            }

            if (!ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                    candidateTime,
                    targetBody.Collider.Id,
                    foundCollision,
                    earliestTime,
                    earliestTargetId))
            {
                continue;
            }

            foundCollision = true;
            earliestTime = candidateTime;
            earliestTargetId = targetBody.Collider.Id;
            earliestContact = candidateContact;
            earliestHasContact = candidateHasContact;
            earliestContactTime = candidateContactTime;
            earliestTarget = targetBody.Collider;
        }

        if (!foundCollision)
            return false;

        safeTime = earliestTime;
        contact = earliestContact;
        hasContact = earliestHasContact;
        contactTime = earliestContactTime;
        hitTarget = earliestTarget;
        return true;
    }

    private bool CanExcludeInvariantTangentialContact(Physics3DHit hit, Vector3d displacement)
    {
        // A supporting plane certifies non-closing straight motion only for one
        // convex shape. A compound/concave target may hide a later blocking part.
        if (!HasRotationInvariantCollider
            || hit.Collider is not (LSCuboidCollider or LSSphereCollider or LSCapsuleCollider
                or LSCylinderCollider or LSConeCollider)
            || hit.Normal == Vector3d.Zero)
            return false;

        Vector3d normal = ResolveShapeExactContinuousClosingNormal(hit);
        // These sealed primitives produce a nonzero outward normal at their
        // surface anchor; an unrepresentable anchor retains the admitted normal.
        return !IsClosingContinuousCollisionHit(displacement, normal);
    }
}
