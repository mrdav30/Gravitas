//=======================================================================
// SolidBody.ContinuousCollision.Rotational.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;

namespace Gravitas;

public partial class SolidBody
{
    private bool TryResolveRotationalContinuousCollision(
        Vector3d startPosition,
        ref Vector3d proposedPosition,
        FixedQuaternion startRotation,
        ref FixedQuaternion proposedRotation)
    {
        if (!CanRotate || !ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Fixed64 angularDistance = _angularSpeed * Context.DeltaTime;
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

        Vector3d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query3D.OverlapSphereAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDistance);
        if (stepCount <= 0)
            return false;

        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
        try
        {
            for (int step = 1; step <= stepCount; step++)
            {
                Fixed64 lowerTime = (Fixed64)(step - 1) / (Fixed64)stepCount;
                Fixed64 sampleTime = (Fixed64)step / (Fixed64)stepCount;
                bool foundSampleHit = false;
                Fixed64 bestSafeTime = Fixed64.Zero;
                int bestTargetId = int.MaxValue;
                Vector3d bestContactNormal = Vector3d.Zero;

                for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
                {
                    SampleDynamicRotationalContinuousPose(startPosition, displacement, startRotation, sampleTime);
                    LSCollider? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Vector3d contactNormal))
                        continue;

                    LSCollider targetCollider = target!;
                    Fixed64 safeTime = RefineDynamicRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        lowerTime,
                        sampleTime,
                        contactNormal,
                        out Vector3d refinedNormal);
                    if (!ShouldReplaceRotationalContinuousCollisionHit(
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
                    bestContactNormal = refinedNormal;
                }

                if (!foundSampleHit)
                    continue;

                proposedPosition = startPosition + displacement * bestSafeTime;
                proposedRotation = IntegrateAngularRotation(startRotation, Context.DeltaTime * bestSafeTime);
                StopRotationalContinuousCollision(bestContactNormal);
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
        }

        return false;
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

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        Fixed64 angularArcLength = angularDistance * proxyRadius;
        if (proxyRadius <= Fixed64.Epsilon
            || angularArcLength <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && angularArcLength <= proxyRadius))
        {
            return false;
        }

        Vector3d displacement = proposedPosition - startPosition;
        int hitCount = displacement.MagnitudeSquared <= Fixed64.Epsilon
            ? Context.Query3D.OverlapSphereAgainstStaticAll(
                startPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false)
            : Context.Query3D.SweepSphereAgainstStaticAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousCollisionHits,
                Collider,
                includeTriggers: false);

        if (hitCount == 0)
            return false;

        int stepCount = ContinuousCollisionMath.ResolveRotationalSubstepCount(angularDistance);
        if (stepCount <= 0)
            return false;

        FixedQuaternion targetRotation = proposedRotation;
        Vector3d originalPosition = Position3d;
        FixedQuaternion originalRotation = Rotation;
        bool originalPositionMutated = _positionMutated;
        bool originalRotationMutated = _rotationMutated;
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
                    SampleKinematicRotationalContinuousPose(startPosition, displacement, startRotation, targetRotation, sampleTime);
                    LSCollider? target = _continuousCollisionHits[hitIndex].Collider;
                    if (!TrySampleRotationalContinuousCollision(target, out Vector3d contactNormal))
                        continue;

                    LSCollider targetCollider = target!;
                    Fixed64 safeTime = RefineKinematicRotationalContinuousCollisionSafeTime(
                        targetCollider,
                        startPosition,
                        displacement,
                        startRotation,
                        targetRotation,
                        lowerTime,
                        sampleTime,
                        contactNormal,
                        out _);
                    if (!ShouldReplaceRotationalContinuousCollisionHit(
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
                proposedRotation = FixedQuaternion.Slerp(startRotation, targetRotation, bestSafeTime).Normalized;
                LastContinuousCollisionToiIterationCount++;
                return true;
            }
        }
        finally
        {
            Position3d = originalPosition;
            Rotation = originalRotation;
            Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
            _positionMutated = originalPositionMutated;
            _rotationMutated = originalRotationMutated;
        }

        return false;
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

    private Fixed64 RefineDynamicRotationalContinuousCollisionSafeTime(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        Fixed64 lowerTime,
        Fixed64 upperTime,
        Vector3d upperContactNormal,
        out Vector3d contactNormal)
    {
        Fixed64 safeTime = lowerTime;
        Fixed64 hitTime = upperTime;
        contactNormal = upperContactNormal;

        for (int iteration = 0; iteration < ContinuousCollisionMath.RotationalToiRefinementIterations; iteration++)
        {
            Fixed64 sampleTime = (safeTime + hitTime) * Fixed64.Half;
            SampleDynamicRotationalContinuousPose(startPosition, displacement, startRotation, sampleTime);
            if (TrySampleRotationalContinuousCollision(target, out Vector3d sampleContactNormal))
            {
                hitTime = sampleTime;
                contactNormal = sampleContactNormal;
            }
            else
            {
                safeTime = sampleTime;
            }
        }

        return safeTime;
    }

    private Fixed64 RefineKinematicRotationalContinuousCollisionSafeTime(
        LSCollider target,
        Vector3d startPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        FixedQuaternion targetRotation,
        Fixed64 lowerTime,
        Fixed64 upperTime,
        Vector3d upperContactNormal,
        out Vector3d contactNormal)
    {
        Fixed64 safeTime = lowerTime;
        Fixed64 hitTime = upperTime;
        contactNormal = upperContactNormal;

        for (int iteration = 0; iteration < ContinuousCollisionMath.RotationalToiRefinementIterations; iteration++)
        {
            Fixed64 sampleTime = (safeTime + hitTime) * Fixed64.Half;
            SampleKinematicRotationalContinuousPose(startPosition, displacement, startRotation, targetRotation, sampleTime);
            if (TrySampleRotationalContinuousCollision(target, out Vector3d sampleContactNormal))
            {
                hitTime = sampleTime;
                contactNormal = sampleContactNormal;
            }
            else
            {
                safeTime = sampleTime;
            }
        }

        return safeTime;
    }

    private static bool ShouldReplaceRotationalContinuousCollisionHit(
        Fixed64 candidateSafeTime,
        int candidateTargetId,
        bool hasCurrent,
        Fixed64 currentSafeTime,
        int currentTargetId)
    {
        if (!hasCurrent)
            return true;

        int timeCompare = candidateSafeTime.CompareTo(currentSafeTime);
        if (timeCompare != 0)
            return timeCompare < 0;

        return candidateTargetId < currentTargetId;
    }

    private static Fixed64 ResolveKinematicAngularDistanceRadians(FixedQuaternion startRotation, FixedQuaternion proposedRotation)
    {
        Fixed64 angleDegrees = FixedQuaternion.Angle(startRotation, proposedRotation) * (Fixed64)2;
        return FixedMath.DegToRad(angleDegrees.Abs());
    }

    private bool TrySampleRotationalContinuousCollision(LSCollider? target, out Vector3d contactNormal)
    {
        contactNormal = Vector3d.Zero;
        if (!IsValidContinuousCollisionTarget(target))
            return false;

        OrderRotationalContinuousCollisionPair(target!, out LSCollider colliderA, out LSCollider colliderB, out bool sourceIsA);
        CollisionType collisionType = ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape);
        if (collisionType == CollisionType.None)
            return false;

        _rotationalContinuousCollisionManifold.BeginUpdate(Context.FrameCount);
        var workItem = new CollisionWorkItem(Context, colliderA, colliderB, collisionType, _rotationalContinuousCollisionManifold);
        if (!CollisionDetection.DoCollisionCheck(workItem) || !_rotationalContinuousCollisionManifold.HasContact)
            return false;

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
        FixedQuaternion angularVelocityQuaternion = new(_angularVelocity.X, _angularVelocity.Y, _angularVelocity.Z, Fixed64.Zero);
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
