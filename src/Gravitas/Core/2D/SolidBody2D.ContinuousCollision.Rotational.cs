//=======================================================================
// SolidBody2D.ContinuousCollision.Rotational.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;

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

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * proxyRadius;
        if (proxyRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= proxyRadius))
        {
            return false;
        }

        Vector2d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query2D.OverlapCircleAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDelta);
        if (stepCount <= 0)
            return false;

        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
        try
        {
            for (int step = 1; step <= stepCount; step++)
            {
                Fixed64 lowerTime = (Fixed64)(step - 1) / (Fixed64)stepCount;
                Fixed64 sampleTime = (Fixed64)step / (Fixed64)stepCount;
                bool foundSampleHit = false;
                Fixed64 bestSafeTime = Fixed64.Zero;
                int bestTargetId = int.MaxValue;
                Contact2D bestContact = default;

                for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
                {
                    SampleRotationalContinuousPose(startPosition, displacement, startRotation, angularDelta, sampleTime);
                    LSCollider2D? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Contact2D contact))
                        continue;

                    LSCollider2D targetCollider = target!;
                    Fixed64 safeTime = RefineRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        angularDelta,
                        lowerTime,
                        sampleTime,
                        contact,
                        out Contact2D refinedContact);
                    if (!ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                            safeTime,
                            targetCollider.Id,
                            foundSampleHit,
                            bestSafeTime,
                            bestTargetId))
                    {
                        continue;
                    }

                    foundSampleHit = true;
                    bestSafeTime = safeTime;
                    bestTargetId = targetCollider.Id;
                    bestContact = refinedContact;
                }

                if (!foundSampleHit)
                    continue;

                proposedPosition = startPosition + displacement * bestSafeTime;
                proposedRotation = startRotation + angularDelta * bestSafeTime;
                StopRotationalContinuousCollision(bestContact.Normal);
                return true;
            }
        }
        finally
        {
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
        }

        return false;
    }

    private bool TryResolveKinematicRotationalContinuousCollision(
        Vector2d startPosition,
        ref Vector2d proposedPosition,
        Fixed64 startRotation,
        ref Fixed64 proposedRotation)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDelta = proposedRotation - startRotation;
        Fixed64 angularDistance = angularDelta.Abs();
        if (angularDistance <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * proxyRadius;
        if (proxyRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= proxyRadius))
        {
            return false;
        }

        Vector2d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query2D.OverlapCircleAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query2D.SweepCircleAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDelta);
        if (stepCount <= 0)
            return false;

        Vector2d originalPosition = _position;
        Fixed64 originalRotation = _rotation;
        try
        {
            for (int step = 1; step <= stepCount; step++)
            {
                Fixed64 lowerTime = (Fixed64)(step - 1) / (Fixed64)stepCount;
                Fixed64 sampleTime = (Fixed64)step / (Fixed64)stepCount;
                bool foundSampleHit = false;
                Fixed64 bestSafeTime = Fixed64.Zero;
                int bestTargetId = int.MaxValue;

                for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
                {
                    SampleRotationalContinuousPose(startPosition, displacement, startRotation, angularDelta, sampleTime);
                    LSCollider2D? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Contact2D contact))
                        continue;

                    LSCollider2D targetCollider = target!;
                    Fixed64 safeTime = RefineRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        angularDelta,
                        lowerTime,
                        sampleTime,
                        contact,
                        out _);
                    if (!ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                            safeTime,
                            targetCollider.Id,
                            foundSampleHit,
                            bestSafeTime,
                            bestTargetId))
                    {
                        continue;
                    }

                    foundSampleHit = true;
                    bestSafeTime = safeTime;
                    bestTargetId = targetCollider.Id;
                }

                if (!foundSampleHit)
                    continue;

                proposedPosition = startPosition + displacement * bestSafeTime;
                proposedRotation = startRotation + angularDelta * bestSafeTime;
                LastContinuousCollisionToiIterationCount++;
                return true;
            }
        }
        finally
        {
            _position = originalPosition;
            _rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly();
        }

        return false;
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

    private bool TrySampleRotationalContinuousCollision(LSCollider2D? target, out Contact2D contact)
    {
        if (!IsValidContinuousCollisionTarget(target))
        {
            contact = default;
            return false;
        }

        return CollisionDetection2D.TryCollide(Collider, target!, out contact);
    }

    private Fixed64 RefineRotationalContinuousCollisionSafeTime(
        LSCollider2D target,
        Vector2d startPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 lowerTime,
        Fixed64 upperTime,
        Contact2D upperContact,
        out Contact2D contact)
    {
        Fixed64 safeTime = lowerTime;
        Fixed64 hitTime = upperTime;
        contact = upperContact;

        for (int iteration = 0; iteration < ContinuousCollisionMath.RotationalToiRefinementIterations; iteration++)
        {
            Fixed64 sampleTime = (safeTime + hitTime) * Fixed64.Half;
            SampleRotationalContinuousPose(startPosition, displacement, startRotation, angularDelta, sampleTime);
            if (TrySampleRotationalContinuousCollision(target, out Contact2D sampleContact))
            {
                hitTime = sampleTime;
                contact = sampleContact;
            }
            else
            {
                safeTime = sampleTime;
            }
        }

        return safeTime;
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
