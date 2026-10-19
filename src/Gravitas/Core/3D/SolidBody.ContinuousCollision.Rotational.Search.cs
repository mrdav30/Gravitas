//=======================================================================
// SolidBody.ContinuousCollision.Rotational.Search.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

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
}
