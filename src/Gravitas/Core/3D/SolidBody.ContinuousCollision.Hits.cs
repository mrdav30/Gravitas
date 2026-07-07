//=======================================================================
// SolidBody.ContinuousCollision.Hits.cs
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

namespace Gravitas;

public partial class SolidBody
{
    private bool TryGetFirstContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out Vector3d normal,
        out Fixed64 distance,
        out ContinuousCollisionTargetKind targetKind,
        out LSCollider? target3D,
        out LSCollider2D? target2D)
    {
        Vector3d originalPosition = Position3d;
        bool originalPositionMutated = _positionMutated;
        try
        {
            Position3d = startPosition;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

            int hitCount = QueryStaticContinuousCollisionHits(
                startPosition,
                proposedPosition,
                proxyRadius,
                out bool staticHitsAreShapeExact);
            int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
                ? Context.QueryMixed.SweepSphereAgainstStatic2DAll(
                    startPosition,
                    proposedPosition,
                    proxyRadius,
                    PhysicsLayerMask.All,
                    _continuousMixedCollisionHits,
                    Collider,
                    includeTriggers: false,
                    cacheTargetPartitions: true)
                : 0;

            bool found3D = TryGetFirstValidContinuousCollisionHit(
                startPosition,
                proposedPosition,
                hitCount,
                staticHitsAreShapeExact,
                out Physics3DHit hit3D);
            ContinuousCollisionTargetKind hit3DKind = found3D
                ? ContinuousCollisionTargetKind.Static3D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamic3D = TryGetFirstDynamicContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                remainingFrameFraction,
                out Physics3DHit dynamicHit3D,
                out Fixed64 dynamicClosingSpeed3D);
            if (ContinuousCollisionCandidateOrdering.ShouldReplaceHit(dynamicHit3D, dynamicClosingSpeed3D, foundDynamic3D, found3D, hit3D, Fixed64.Zero))
            {
                hit3D = dynamicHit3D;
                found3D = true;
                hit3DKind = ContinuousCollisionTargetKind.Dynamic3D;
            }

            bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(startPosition, proposedPosition, mixedHitCount, out PhysicsMixedHit hitMixed);
            ContinuousCollisionTargetKind hitMixedKind = foundMixed
                ? ContinuousCollisionTargetKind.Static2D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamicMixed = TryGetFirstDynamicMixedContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                remainingFrameFraction,
                out PhysicsMixedHit dynamicHitMixed,
                out Fixed64 dynamicClosingSpeedMixed);
            if (ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(dynamicHitMixed, dynamicClosingSpeedMixed, foundDynamicMixed, foundMixed, hitMixed, Fixed64.Zero))
            {
                hitMixed = dynamicHitMixed;
                foundMixed = true;
                hitMixedKind = ContinuousCollisionTargetKind.Dynamic2D;
            }

            if (found3D && (!foundMixed || hit3D.Distance <= hitMixed.Distance))
            {
                normal = hit3D.Normal;
                distance = hit3D.Distance;
                targetKind = hit3DKind;
                target3D = hit3D.Collider;
                target2D = null;
                return true;
            }

            if (foundMixed)
            {
                normal = hitMixed.NormalFor3DSource;
                distance = hitMixed.Distance;
                targetKind = hitMixedKind;
                target3D = null;
                target2D = hitMixed.Collider2D;
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
        }

        normal = Vector3d.Zero;
        distance = Fixed64.Zero;
        targetKind = ContinuousCollisionTargetKind.None;
        target3D = null;
        target2D = null;
        return false;
    }

    private bool TryGetFirstValidContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        int hitCount,
        bool hitsAreShapeExact,
        out Physics3DHit hit)
    {
        Vector3d displacement = proposedPosition - startPosition;
        Vector3d direction = displacement.MagnitudeSquared > Fixed64.Epsilon ? displacement.Normalized : Vector3d.Zero;
        bool found = false;
        Physics3DHit best = default;
        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit candidate = _continuousCollisionHits[i];
            if (!IsValidContinuousCollisionHit(candidate))
                continue;

            Physics3DHit refined;
            if (hitsAreShapeExact)
                refined = ApplyShapeExactContinuousContactSlop(candidate);
            else if (TryRefineShapeExactContinuousCollisionHit(candidate, displacement, direction, out Physics3DHit exactHit, out bool exactSupported))
                refined = exactHit;
            else if (exactSupported)
                continue;
            else
                refined = candidate;

            if (!IsClosingContinuousCollisionHit(displacement, refined.Normal))
                continue;

            if (found && !ContinuousCollisionHitComesBefore(refined, best))
                continue;

            best = refined;
            found = true;
        }

        hit = best;
        return found;
    }

    private int QueryStaticContinuousCollisionHits(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        out bool hitsAreShapeExact)
    {
        Vector3d displacement = proposedPosition - startPosition;
        if (Collider is not LSSphereCollider && IsExactConvexSourceSupported(Collider))
        {
            hitsAreShapeExact = true;
            return Context.Query3D.SweepExactSourceAgainstStaticAll(
                Collider,
                displacement,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);
        }

        hitsAreShapeExact = false;
        return Context.Query3D.SweepSphereAgainstStaticAll(
            startPosition,
            proposedPosition,
            proxyRadius,
            PhysicsLayerMask.All,
            _continuousCollisionHits,
            Collider,
            includeTriggers: false);
    }

    private bool TryGetFirstValidMixedContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        int hitCount,
        out PhysicsMixedHit hit)
    {
        Vector3d displacement = proposedPosition - startPosition;
        for (int i = 0; i < hitCount; i++)
        {
            PhysicsMixedHit candidate = _continuousMixedCollisionHits[i];
            if (!IsValidMixedContinuousCollisionHit(candidate)
                || !IsClosingContinuousCollisionHit(displacement, candidate.NormalFor3DSource))
                continue;

            hit = candidate;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryRefineShapeExactContinuousCollisionHit(
        Physics3DHit candidate,
        Vector3d displacement,
        Vector3d direction,
        out Physics3DHit refined,
        out bool exactSupported)
    {
        refined = default;
        exactSupported = false;
        LSCollider? target = candidate.Collider;
        if (target == null || displacement.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        if (Collider is LSSphereCollider)
            return false;

        if (target is not LSSphereCollider targetSphere)
            return false;

        return TryRefineContinuousCollisionAgainstTargetSphere(targetSphere, displacement, direction, out refined, out exactSupported);
    }

    private bool TryRefineContinuousCollisionAgainstTargetSphere(
        LSSphereCollider targetSphere,
        Vector3d displacement,
        Vector3d direction,
        out Physics3DHit refined,
        out bool exactSupported)
    {
        exactSupported = true;
        refined = default;

        Vector3d reverseStart = targetSphere.Center;
        Vector3d reverseEnd = targetSphere.Center - displacement;
        _shapeExactContinuousSweepWorker.Prepare(reverseStart, reverseEnd, targetSphere.ScaledRadius);
        if (!_shapeExactContinuousSweepWorker.TrySweep(Collider, out Vector3d reverseCenterAtImpact, out Fixed64 distance))
            return false;

        Vector3d sourcePoint = Collider.ClosestPointOnSurface(reverseCenterAtImpact);
        Vector3d normalDelta = sourcePoint - reverseCenterAtImpact;
        Vector3d normal = normalDelta.MagnitudeSquared > Fixed64.Epsilon
            ? normalDelta.Normalized
            : -direction;
        Vector3d point = targetSphere.Center + normal * targetSphere.ScaledRadius;
        refined = new Physics3DHit(targetSphere, point, normal, distance, direction);
        return true;
    }

    private static Physics3DHit ApplyShapeExactContinuousContactSlop(Physics3DHit hit)
    {
        Fixed64 distance = hit.Distance > ShapeExactContinuousContactSlop
            ? hit.Distance - ShapeExactContinuousContactSlop
            : Fixed64.Zero;
        return new Physics3DHit(hit.Collider, hit.Point, hit.Normal, distance, hit.Direction);
    }

    private void PrepareExactConvexSourceSweep(Vector3d displacement)
    {
        switch (Collider)
        {
            case LSMeshCollider mesh:
                _shapeExactContinuousConvexSweepWorker.PrepareConvexMeshSource(mesh, displacement);
                return;
            case LSCompoundCollider compound:
                _shapeExactContinuousConvexSweepWorker.PrepareCompoundSource(compound, displacement);
                return;
            default:
                _shapeExactContinuousConvexSweepWorker.PreparePrimitiveSource(Collider, displacement);
                return;
        }
    }

    private static bool IsExactConvexSourceSupported(LSCollider collider)
    {
        return collider switch
        {
            LSSphereCollider => true,
            LSCapsuleCollider => true,
            LSCuboidCollider => true,
            LSCylinderCollider => true,
            LSConeCollider => true,
            LSMeshCollider { Mode: MeshColliderMode.Convex } => true,
            LSCompoundCollider compound => AreExactConvexCompoundPartsSupported(compound),
            _ => false
        };
    }

    private static bool AreExactConvexCompoundPartsSupported(LSCompoundCollider compound)
    {
        for (int i = 0; i < compound.PartCount; i++)
        {
            if (!IsExactConvexSourceSupported(compound.GetPartCollider(i)))
                return false;
        }

        return true;
    }

    private static bool ContinuousCollisionHitComesBefore(Physics3DHit left, Physics3DHit right)
    {
        int distanceCompare = left.Distance.CompareTo(right.Distance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        return left.Collider!.Id < right.Collider!.Id;
    }

    private bool TryGetFirstDynamicContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out Physics3DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        bool found = false;
        Physics3DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out SolidBody target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector3d targetStart = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector3d targetDisplacement = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Fixed64 targetRadius = ResolveContinuousCollisionProxyRadius(target.Collider);
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normal,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Physics3DHit candidate;
            if (TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    out Physics3DHit exactHit,
                    out Fixed64 exactClosingSpeed,
                    out bool exactSupported))
            {
                candidate = exactHit;
                candidateClosingSpeed = exactClosingSpeed;
            }
            else if (exactSupported)
            {
                continue;
            }
            else
            {
                Fixed64 distance = sourceLength * normalizedTime;
                Vector3d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
                Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
                Vector3d point = ContinuousCollisionMath.ResolveContactPointOnTarget(sourceCenter, targetCenter, normal, targetRadius);
                candidate = new Physics3DHit(target.Collider, point, normal, distance, sourceDirection);
            }

            if (!ContinuousCollisionCandidateOrdering.ShouldReplaceHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool TryGetExactDynamicRelativeContinuousCollisionHit(
        SolidBody target,
        Vector3d sourceStart,
        Vector3d sourceDisplacement,
        Vector3d targetStart,
        Vector3d targetDisplacement,
        Fixed64 sourceLength,
        out Physics3DHit hit,
        out Fixed64 closingSpeed,
        out bool exactSupported)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;
        exactSupported = false;

        Vector3d relativeDisplacement = sourceDisplacement - targetDisplacement;
        Fixed64 relativeLength = relativeDisplacement.Magnitude;
        if (relativeLength <= Fixed64.Epsilon || sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d relativeDirection = relativeDisplacement / relativeLength;
        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        Vector3d originalSourcePosition = Position3d;
        FixedQuaternion originalSourceRotation = Rotation;
        bool originalSourcePositionMutated = _positionMutated;
        bool originalSourceRotationMutated = _rotationMutated;
        Vector3d originalTargetPosition = target.Position3d;
        FixedQuaternion originalTargetRotation = target.Rotation;
        bool originalTargetPositionMutated = target._positionMutated;
        bool originalTargetRotationMutated = target._rotationMutated;

        try
        {
            Position3d = sourceStart;
            target.Position3d = targetStart;
            target.Rotation = target.ContinuousCollisionFrameRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            target.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);

            Physics3DHit relativeHit;
            if (Collider is LSSphereCollider sourceSphere)
            {
                exactSupported = true;
                if (!TrySweepRelativeSourceSphere(sourceSphere, target.Collider, relativeDisplacement, relativeDirection, out relativeHit))
                    return false;
            }
            else if (target.Collider is LSSphereCollider targetSphere)
            {
                if (!TryRefineContinuousCollisionAgainstTargetSphere(targetSphere, relativeDisplacement, relativeDirection, out relativeHit, out exactSupported))
                    return false;
            }
            else if (IsExactConvexSourceSupported(Collider))
            {
                exactSupported = true;
                PrepareExactConvexSourceSweep(relativeDisplacement);
                if (!_shapeExactContinuousConvexSweepWorker.TrySweepPreparedSource(target.Collider, out Physics3DHit convexHit))
                    return false;

                relativeHit = ApplyShapeExactContinuousContactSlop(convexHit);
            }
            else
            {
                return false;
            }

            closingSpeed = -Vector3d.Dot(relativeDisplacement, relativeHit.Normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 normalizedTime = FixedMath.Clamp01(relativeHit.Distance / relativeLength);
            hit = new Physics3DHit(
                target.Collider,
                relativeHit.Point + targetDisplacement * normalizedTime,
                relativeHit.Normal,
                sourceLength * normalizedTime,
                sourceDirection);
            return true;
        }
        finally
        {
            Position3d = originalSourcePosition;
            Rotation = originalSourceRotation;
            target.Position3d = originalTargetPosition;
            target.Rotation = originalTargetRotation;
            target.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            target._positionMutated = originalTargetPositionMutated;
            target._rotationMutated = originalTargetRotationMutated;
            _positionMutated = originalSourcePositionMutated;
            _rotationMutated = originalSourceRotationMutated;
        }
    }

    private bool TrySweepRelativeSourceSphere(
        LSSphereCollider sourceSphere,
        LSCollider target,
        Vector3d relativeDisplacement,
        Vector3d relativeDirection,
        out Physics3DHit hit)
    {
        hit = default;
        _shapeExactContinuousSweepWorker.Prepare(
            sourceSphere.Center,
            sourceSphere.Center + relativeDisplacement,
            sourceSphere.ScaledRadius);
        if (!_shapeExactContinuousSweepWorker.TrySweep(target, out Vector3d sphereCenterAtImpact, out Fixed64 distance))
            return false;

        Vector3d point = ResolveSweptSphereContinuousPoint(target, sphereCenterAtImpact, relativeDirection);
        Vector3d normal = ResolveSweptSphereContinuousNormal(target, point, sphereCenterAtImpact, relativeDirection);
        hit = new Physics3DHit(target, point, normal, distance, relativeDirection);
        return true;
    }

    private static Vector3d ResolveSweptSphereContinuousPoint(
        LSCollider target,
        Vector3d sphereCenterAtImpact,
        Vector3d direction)
    {
        Vector3d centerDelta = sphereCenterAtImpact - target.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return target.Center - direction * target.ScaledRadius;

        return target.ClosestPointOnSurface(sphereCenterAtImpact);
    }

    private static Vector3d ResolveSweptSphereContinuousNormal(
        LSCollider target,
        Vector3d point,
        Vector3d sphereCenterAtImpact,
        Vector3d direction)
    {
        Vector3d fromPointToSphereCenter = sphereCenterAtImpact - point;
        if ((target is LSCuboidCollider || target is LSCylinderCollider || target is LSConeCollider)
            && fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
        {
            return fromPointToSphereCenter.Normalized;
        }

        Vector3d normal = target.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normal.Normalized;
            if (target is LSMeshCollider && Vector3d.Dot(normal, direction) > Fixed64.Zero)
                return -normal;

            return normal;
        }

        if (fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
            return fromPointToSphereCenter.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Zero;
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out PhysicsMixedHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryMixedContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out SolidBody2D target)
                || !IsEligibleDynamicMixed2DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = FixedMath.Max(
                target.ResolveContinuousCollisionProxyRadiusForDynamicTarget(),
                target.Collider.MixedHalfThickness);
            if (targetRadius <= Fixed64.Epsilon)
                continue;

            Vector2d targetStart2D = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector2d targetDisplacement2D = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Vector3d targetStart = new(targetStart2D.X, target.Collider.MixedSlabCenterY, targetStart2D.Y);
            Vector3d targetDisplacement = new(targetDisplacement2D.X, Fixed64.Zero, targetDisplacement2D.Y);
            if (!ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normalForSource,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            Vector3d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
            Vector3d point3D = sourceCenter - normalForSource * proxyRadius;
            Vector3d point2D = ContinuousCollisionMath.ResolveContactPointOnTarget(sourceCenter, targetCenter, normalForSource, targetRadius);
            var candidate = new PhysicsMixedHit(
                null,
                target.Collider,
                point3D,
                point2D,
                -normalForSource,
                PhysicsQueryReducerKind.ConservativeFallback,
                distance,
                sourceDirection);
            if (!ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

}
