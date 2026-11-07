//=======================================================================
// SolidBody.ContinuousCollision.Rotational.Search.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
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

    private bool IsRotatingContinuousCollisionTarget(SolidBody target)
    {
        if (ReferenceEquals(target, this)
            || !IsMovingRotationalContinuousCollisionTarget(target))
        {
            return false;
        }

        target.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        return !target.HasRotationInvariantCollider && target.HasContinuousCollisionRotationalMotion;
    }

    private bool IsRotatingContinuousCollisionTarget(SolidBody2D target)
    {
        if (!IsMovingMixedRotationalContinuousCollisionTarget(target))
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
        // The live root registry cannot supply another target when it contains only this source.
        if (Context.Physics.ColliderCount == 1
            && ReferenceEquals(Context.Physics.GetColliderByServiceIndex(0), Collider))
        {
            _continuousCollisionHits.Clear();
            return 0;
        }

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
        {
            int candidateId = candidateIds[i];
            if (candidateId != DynamicId)
                _rotationalContinuousCollisionCandidateIds.Add(candidateId);
        }

        return staticHitCount + _rotationalContinuousCollisionCandidateIds.Count;
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
                || CanExcludeInvariantTangentialContact(hit, startPosition, displacement,
                    startRotation, targetRotation, isKinematic)
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

    internal bool CanExcludeInvariantTangentialContact(Physics3DHit hit,
        Vector3d startPosition, Vector3d displacement, FixedQuaternion startRotation,
        FixedQuaternion targetRotation, bool isKinematic)
    {
        if (hit.Normal == Vector3d.Zero)
            return false;

        // Keep the existing centered-sphere rule: these primitive normals are
        // global convex supporting planes even for an initially overlapping cast.
        if (HasRotationInvariantCollider
            && hit.Collider is (LSCuboidCollider or LSSphereCollider or LSCapsuleCollider
                or LSCylinderCollider or LSConeCollider))
            return !IsClosingContinuousCollisionHit(displacement, ResolveShapeExactContinuousClosingNormal(hit));

        if (Collider is LSCompoundCollider
            || (hit.Collider!.Body is SolidBody targetBody && !targetBody.IsStatic))
            return false;
        Vector3d normal = hit.Normal;
        if (!HasInvariantContinuousProjection(normal, displacement, startRotation, targetRotation, isKinematic))
            return false;

        // Earlier candidate searches leave the source at their last sampled
        // pose. The separating plane must be proved from this segment's start.
        Position3d = startPosition;
        Rotation = startRotation;
        Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
        // Production collider types are closed; compound sources were excluded
        // above. A global supporting plane is valid for every mesh vertex even
        // when that source mesh uses concave triangle-surface collision policy.
        System.Diagnostics.Debug.Assert(Collider is LSMeshCollider || ConvexColliderSupport.IsSupported(Collider));
        RationalPoint3d sourceMin = Collider is LSMeshCollider
            ? GetExactMeshSupportPoint((LSMeshCollider)Collider, -normal)
            : RationalPoint3d.FromAnchor(new ConvexShape(Collider, Vector3d.Zero).GetSupportAnchor(-normal));
        if (!TryGetFullColliderSupportPoint(hit.Collider, normal, out RationalPoint3d targetMax))
            return false;
        // Full-target support includes every concave vertex and compound leaf;
        // a later wall cannot hide behind the initial floor hit. A nonnegative
        // exact gap plus invariant support projection proves the entire path safe.
        return RationalPoint3d.CompareProjection(sourceMin, targetMax, normal) >= 0;
    }

    private bool HasInvariantContinuousProjection(Vector3d normal, Vector3d displacement,
        FixedQuaternion startRotation, FixedQuaternion targetRotation, bool isKinematic)
    {
        int axis = normal.X != Fixed64.Zero ? 0 : normal.Y != Fixed64.Zero ? 1 : 2;
        // This is a certificate for every rounded intermediate pose, rather
        // than an approximate parallel-axis or endpoint-only check. Integration,
        // Slerp and normalization preserve these literal zero components. General
        // rotations still require the conservative interval search.
        if (Vector3d.CompareProjection(displacement, Vector3d.Zero, normal) < 0)
            return false;
        for (int offset = 1; offset <= 2; offset++)
        {
            int transverse = (axis + offset) % 3;
            bool preservesProjection = normal[transverse] == Fixed64.Zero
                & startRotation[transverse] == Fixed64.Zero
                & targetRotation[transverse] == Fixed64.Zero
                & (isKinematic | _angularVelocity[transverse] == Fixed64.Zero);
            if (!preservesProjection)
                return false;
        }

        return true;
    }

    private static bool TryGetFullColliderSupportPoint(LSCollider collider,
        Vector3d direction, out RationalPoint3d point)
    {
        if (collider is LSCompoundCollider compound)
        {
            if (!TryGetLeafColliderSupportPoint(compound.GetPartCollider(0), direction, out point)) return false;
            for (int index = 1; index < compound.PartCount; index++)
            {
                if (!TryGetLeafColliderSupportPoint(compound.GetPartCollider(index), direction,
                        out RationalPoint3d candidate)) return false;
                if (RationalPoint3d.CompareProjection(candidate, point, direction) > 0) point = candidate;
            }
            return true;
        }
        return TryGetLeafColliderSupportPoint(collider, direction, out point);
    }

    private static bool TryGetLeafColliderSupportPoint(LSCollider collider,
        Vector3d direction, out RationalPoint3d point)
    {
        if (collider is LSMeshCollider mesh)
        { point = GetExactMeshSupportPoint(mesh, direction); return true; }
        if (collider is LSCuboidCollider)
        {
            point = RationalPoint3d.FromAnchor(new ConvexShape(collider, Vector3d.Zero).GetSupportAnchor(direction));
            return true;
        }
        point = default;
        return false;
    }

    private static RationalPoint3d GetExactMeshSupportPoint(LSMeshCollider collider, Vector3d direction)
    {
        PhysicsMesh mesh = collider.Mesh;
        System.ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        RationalPoint3d support = mesh.GeometryFrame.GetPoint(vertices[0]);
        // Exclusion needs the exact global extremum: rounding an inverse
        // direction can select a lower vertex even on the compact rigid path.
        for (int index = 1; index < vertices.Length; index++)
        {
            RationalPoint3d candidate = mesh.GeometryFrame.GetPoint(vertices[index]);
            if (RationalPoint3d.CompareProjection(candidate, support, direction) > 0)
                support = candidate;
        }
        return support;
    }

    private bool TryFindEarliestRotationalContinuousCollisionAgainstTarget(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        Fixed64 pivotRadius,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool isKinematic,
        out Fixed64 safeTime,
        out ManifoldContact contact,
        out bool hasContact,
        out Fixed64 contactTime)
    {
        safeTime = Fixed64.Zero;
        contact = default;
        hasContact = false;
        contactTime = Fixed64.Zero;
        SolidBody? targetBody = target.Body;
        bool samplesTargetMotion = targetBody != null
            && IsMovingRotationalContinuousCollisionTarget(targetBody);
        Vector3d originalTargetPosition = targetBody?.Position3d ?? Vector3d.Zero;
        FixedQuaternion originalTargetRotation = targetBody?.Rotation ?? FixedQuaternion.Identity;
        bool originalTargetPositionMutated = targetBody?._positionMutated ?? false;
        bool originalTargetRotationMutated = targetBody?._rotationMutated ?? false;
        if (samplesTargetMotion)
            targetBody!.EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        LSMeshCollider? mesh = Collider as LSMeshCollider ?? target as LSMeshCollider;
        LSConeCollider? cone = Collider as LSConeCollider ?? target as LSConeCollider;
        bool deferConeContact = mesh != null && cone != null && mesh.StaticSurface == null
            && (mesh.Mode == MeshColliderMode.Concave || !mesh.IsClosedSurface);

        try
        {
            Span<ContinuousCollisionMath.RotationalInterval> intervals =
                stackalloc ContinuousCollisionMath.RotationalInterval[
                    ContinuousCollisionMath.RotationalIntervalMaxDepth + 2];
            int intervalCount = 1;
            int processedNodeCount = 0;
            bool hasKnownContact = false;
            bool knownContactIsClosing = false;
            Fixed64 knownContactTime = Fixed64.One;
            ManifoldContact knownContact = default;
            intervals[0] = new ContinuousCollisionMath.RotationalInterval(
                Fixed64.Zero,
                Fixed64.One,
                depth: 0);

            while (intervalCount > 0)
            {
                ContinuousCollisionMath.RotationalInterval interval = intervals[--intervalCount];
                Fixed64 midpoint = (interval.LowerTime + interval.UpperTime) * Fixed64.Half;
                Fixed64 intervalSpan = interval.UpperTime - interval.LowerTime;
                Fixed64 midpointFrameFraction = SampleRotationalContinuousPairPose(startPosition, displacement,
                    startRotation, targetRotation, midpoint, elapsedTime, remainingTime, isKinematic,
                    targetBody, samplesTargetMotion);

                processedNodeCount++;
                bool hasMotionBound = ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
                    displacement,
                    angularDistance,
                    pivotRadius,
                    intervalSpan,
                    out Fixed64 motionBound);
                if (hasMotionBound && samplesTargetMotion)
                {
                    Fixed64 lowerFrameFraction = ResolveRotationalFrameFraction(
                        elapsedTime,
                        remainingTime,
                        interval.LowerTime);
                    Fixed64 upperFrameFraction = ResolveRotationalFrameFraction(
                        elapsedTime,
                        remainingTime,
                        interval.UpperTime);
                    Fixed64 targetRadius = targetBody!.ResolveContinuousCollisionProxyRadius();
                    bool targetBoundResolved = targetBody.TryResolveContinuousCollisionMotionBound(
                        lowerFrameFraction,
                        upperFrameFraction,
                        targetRadius,
                        out Fixed64 targetMotionBound);
                    bool combinedBoundResolved = Fixed64.TryAdd(
                        motionBound,
                        targetMotionBound,
                        out motionBound);
                    hasMotionBound = targetBoundResolved & combinedBoundResolved;
                }
                ManifoldContact sampleContact = default;
                bool sampleContactIsClosing = false;
                bool sampleHasContact = deferConeContact
                    ? CollisionDetection.IntersectsMeshConeSurface(mesh!, cone!, Context.CollisionScratch)
                    : TrySampleRotationalContinuousCollision(target, midpointFrameFraction,
                        out sampleContact, out sampleContactIsClosing);
                if (sampleHasContact
                    && (!hasKnownContact || midpoint < knownContactTime))
                {
                    hasKnownContact = true;
                    knownContactTime = midpoint;
                    knownContact = sampleContact;
                    knownContactIsClosing = sampleContactIsClosing;
                }

                if (!sampleHasContact
                    && hasMotionBound
                    && IsRotationalIntervalSeparated(target, motionBound))
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
                    if (hasContact && deferConeContact && contactTime == safeTime)
                    {
                        // Closing admission needs the complete manifold at the retained witness, not the safe frontier.
                        Fixed64 witnessFraction = SampleRotationalContinuousPairPose(startPosition, displacement,
                            startRotation, targetRotation, contactTime, elapsedTime, remainingTime, isKinematic,
                            targetBody, samplesTargetMotion);
                        _ = TrySampleRotationalContinuousCollision(target, witnessFraction,
                            out knownContact, out knownContactIsClosing);
                    }
                    // Geometric overlap remains the interval's certified upper
                    // bound. Only its closing witness can authorize a response;
                    // absence of one does not prove the intervening path separated.
                    hasContact &= knownContactIsClosing;
                    contact = knownContact;
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

            return false;
        }
        finally
        {
            if (samplesTargetMotion)
            {
                targetBody!.Position3d = originalTargetPosition;
                targetBody.Rotation = originalTargetRotation;
                targetBody.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
                targetBody._positionMutated = originalTargetPositionMutated;
                targetBody._rotationMutated = originalTargetRotationMutated;
            }
        }
    }


    private Fixed64 SampleRotationalContinuousPairPose(Vector3d startPosition, Vector3d displacement,
        FixedQuaternion startRotation, FixedQuaternion targetRotation, Fixed64 time, Fixed64 elapsedTime,
        Fixed64 remainingTime, bool isKinematic, SolidBody? targetBody, bool samplesTargetMotion)
    {
        if (isKinematic)
            SampleKinematicRotationalContinuousPose(startPosition, displacement, startRotation, targetRotation, time);
        else
            SampleDynamicRotationalContinuousPose(startPosition, displacement, startRotation, remainingTime, time);
        Fixed64 frameFraction = ResolveRotationalFrameFraction(elapsedTime, remainingTime, time);
        if (samplesTargetMotion)
        {
            targetBody!.Position3d = targetBody.SampleContinuousCollisionPosition(frameFraction);
            targetBody.Rotation = targetBody.SampleContinuousCollisionRotation(frameFraction);
            targetBody.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
        }
        return frameFraction;
    }

}
