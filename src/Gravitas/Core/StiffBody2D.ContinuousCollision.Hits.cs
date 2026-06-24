//=======================================================================
// StiffBody2D.ContinuousCollision.Hits.cs
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
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class StiffBody2D
{
    private bool TryGetFirstContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out Vector2d normal,
        out Fixed64 distance,
        out ContinuousCollisionTargetKind targetKind,
        out LSCollider2D? target2D,
        out LSCollider? target3D)
    {
        Vector2d originalPosition = _position;
        try
        {
            _position = startPosition;
            Collider.RebuildRuntimeShapeOnly();

            int hitCount = Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);
            int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
                ? Context.QueryMixed.SweepCircleAgainstStatic3DAll(
                    startPosition,
                    proposedPosition,
                    proxyRadius,
                    Collider.MixedSlabCenterY,
                    Collider.MixedHalfThickness,
                    PhysicsLayerMask.All,
                    _continuousMixedCollisionHits,
                    Collider,
                    includeTriggers: false,
                    cacheTargetPartitions: true)
                : 0;

            bool found2D = TryGetFirstValidContinuousCollisionHit(startPosition, proposedPosition, hitCount, out Physics2DHit hit2D);
            ContinuousCollisionTargetKind hit2DKind = found2D
                ? ContinuousCollisionTargetKind.Static2D
                : ContinuousCollisionTargetKind.None;
            bool foundDynamic2D = TryGetFirstDynamicContinuousCollisionHit(
                startPosition,
                proposedPosition,
                proxyRadius,
                elapsedFrameFraction,
                remainingFrameFraction,
                out Physics2DHit dynamicHit2D,
                out Fixed64 dynamicClosingSpeed2D);
            if (ContinuousCollisionCandidateOrdering.ShouldReplaceHit(dynamicHit2D, dynamicClosingSpeed2D, foundDynamic2D, found2D, hit2D, Fixed64.Zero))
            {
                hit2D = dynamicHit2D;
                found2D = true;
                hit2DKind = ContinuousCollisionTargetKind.Dynamic2D;
            }

            bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(startPosition, proposedPosition, mixedHitCount, out PhysicsMixedHit hitMixed);
            ContinuousCollisionTargetKind hitMixedKind = foundMixed
                ? ContinuousCollisionTargetKind.Static3D
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
                hitMixedKind = ContinuousCollisionTargetKind.Dynamic3D;
            }

            if (found2D && (!foundMixed || hit2D.Distance <= hitMixed.Distance))
            {
                normal = hit2D.Normal;
                distance = hit2D.Distance;
                targetKind = hit2DKind;
                target2D = hit2D.Collider;
                target3D = null;
                return true;
            }

            if (foundMixed)
            {
                normal = hitMixed.NormalFor2DSource;
                distance = hitMixed.Distance;
                targetKind = hitMixedKind;
                target2D = null;
                target3D = hitMixed.Collider3D;
                return true;
            }
        }
        finally
        {
            _position = originalPosition;
            Collider.RebuildRuntimeShapeOnly();
        }

        normal = Vector2d.Zero;
        distance = Fixed64.Zero;
        targetKind = ContinuousCollisionTargetKind.None;
        target2D = null;
        target3D = null;
        return false;
    }

    private bool TryGetFirstValidContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        int hitCount,
        out Physics2DHit hit)
    {
        Vector2d displacement = proposedPosition - startPosition;
        bool found = false;
        Physics2DHit best = default;
        for (int i = 0; i < hitCount; i++)
        {
            Physics2DHit candidate = _continuousCollisionHits[i];
            if (!IsValidContinuousCollisionHit(candidate))
                continue;

            if (!QueryDetection2D.TrySweepMoverShape(Collider, displacement, candidate.Collider, out Physics2DHit refined)
                || !IsClosingContinuousCollisionHit(displacement, refined.Normal))
                continue;

            if (found && !Physics2DHitSorter.ComesBefore(refined, best))
                continue;

            best = refined;
            found = true;
        }

        hit = best;
        return found;
    }

    private bool TryGetFirstValidMixedContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        int hitCount,
        out PhysicsMixedHit hit)
    {
        Vector2d displacement = proposedPosition - startPosition;
        for (int i = 0; i < hitCount; i++)
        {
            PhysicsMixedHit candidate = _continuousMixedCollisionHits[i];
            if (!IsValidMixedContinuousCollisionHit(candidate)
                || !IsClosingContinuousCollisionHit(displacement, candidate.NormalFor2DSource))
                continue;

            hit = candidate;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryGetFirstDynamicContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        Fixed64 elapsedFrameFraction,
        Fixed64 remainingFrameFraction,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector2d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        bool found = false;
        Physics2DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out StiffBody2D target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector2d targetStart = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector2d targetDisplacement = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeCircles(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out _,
                    out _,
                    out _))
            {
                continue;
            }

            if (!TryGetExactDynamicRelativeContinuousCollisionHit(
                    target,
                    startPosition,
                    sourceDisplacement,
                    targetStart,
                    targetDisplacement,
                    sourceLength,
                    out Physics2DHit candidate,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
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
        StiffBody2D target,
        Vector2d sourceStart,
        Vector2d sourceDisplacement,
        Vector2d targetStart,
        Vector2d targetDisplacement,
        Fixed64 sourceLength,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector2d relativeDisplacement = sourceDisplacement - targetDisplacement;
        Fixed64 relativeLength = relativeDisplacement.Magnitude;
        if (relativeLength <= Fixed64.Epsilon || sourceLength <= Fixed64.Epsilon)
            return false;

        Vector2d originalSourcePosition = _position;
        Fixed64 originalSourceRotation = _rotation;
        Vector2d originalTargetPosition = target._position;
        Fixed64 originalTargetRotation = target._rotation;
        try
        {
            _position = sourceStart;
            target._position = targetStart;
            target._rotation = target.ContinuousCollisionFrameRotation;
            Collider.RebuildRuntimeShapeOnly();
            target.Collider.RebuildRuntimeShapeOnly();

            if (!QueryDetection2D.TrySweepMoverShape(Collider, relativeDisplacement, target.Collider, out Physics2DHit relativeHit))
                return false;

            closingSpeed = -Vector2d.Dot(relativeDisplacement, relativeHit.Normal);
            if (closingSpeed <= Fixed64.Epsilon)
                return false;

            Fixed64 normalizedTime = FixedMath.Clamp01(relativeHit.Distance / relativeLength);
            Vector2d point = relativeHit.Point + targetDisplacement * normalizedTime;
            hit = new Physics2DHit(
                target.Collider,
                point,
                relativeHit.Normal,
                sourceLength * normalizedTime);
            return true;
        }
        finally
        {
            _position = originalSourcePosition;
            _rotation = originalSourceRotation;
            target._position = originalTargetPosition;
            target._rotation = originalTargetRotation;
            target.Collider.RebuildRuntimeShapeOnly();
            Collider.RebuildRuntimeShapeOnly();
        }
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
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

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement2D.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceStart = new(startPosition.X, Collider.MixedSlabCenterY, startPosition.Y);
        Vector3d sourceDisplacement = new(sourceDisplacement2D.X, Fixed64.Zero, sourceDisplacement2D.Y);
        Fixed64 sourceRadius = FixedMath.Max(proxyRadius, Collider.MixedHalfThickness);
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(sourceStart, sourceDisplacement, sourceRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out StiffBody target)
                || !IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Vector3d targetStart = target.ContinuousCollisionFrameStart
                + target.ContinuousCollisionFrameDisplacement * elapsedFrameFraction;
            Vector3d targetDisplacement = target.ContinuousCollisionFrameDisplacement * remainingFrameFraction;
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
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
            Vector3d sourceCenter = sourceStart + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
            Vector3d point2D = sourceCenter - normalForSource * sourceRadius;
            Vector3d point3D = ResolveDynamicContactPoint(sourceCenter, targetCenter, normalForSource, targetRadius);
            var candidate = new PhysicsMixedHit(
                target.Collider,
                null,
                point3D,
                point2D,
                normalForSource,
                PhysicsQueryReducerKind.ConservativeFallback,
                distance,
                sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon ? sourceDisplacement.Normalized : Vector3d.Zero);
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

    private bool IsEligibleDynamicContinuousCollisionTarget(StiffBody2D target)
    {
        if (ReferenceEquals(target, this)
            || !target.Active
            || target.Immovable
            || target.IsKinematic
            || target.Collider.IsTrigger
            || !Context.Physics2D.RequireCollisionPair(Collider, target.Collider))
        {
            return false;
        }

        return true;
    }

    private bool IsEligibleDynamicMixed3DTarget(StiffBody target)
    {
        return target.Active
            && !target.Immovable
            && !target.IsKinematic
            && !target.Collider.IsTrigger
            && Context.MixedCollisions.RequireCollisionPair(target.Collider, Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveDynamicContactPoint(
        Vector2d sourceCenter,
        Vector2d targetCenter,
        Vector2d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector2d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveDynamicContactPoint(
        Vector3d sourceCenter,
        Vector3d targetCenter,
        Vector3d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector3d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
    }

}
