//=======================================================================
// SolidBody2D.ContinuousCollision.Rotational.Response.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Materials;

namespace Gravitas;

public partial class SolidBody2D
{
    internal bool TryApplyRotationalContinuousCollisionResponse(
        SolidBody2D target,
        Contact2D contact,
        Fixed64 contactTime,
        Vector2d startPosition,
        Fixed64 startRotation,
        Vector2d displacement,
        Fixed64 angularDelta,
        Fixed64 elapsedTime,
        Fixed64 remainingTime)
    {
        Fixed64 frameFraction = ResolveRotationalFrameFraction(
            elapsedTime,
            remainingTime,
            contactTime);
        Vector2d sourcePositionAtImpact = startPosition + displacement * contactTime;
        Fixed64 sourceRotationAtImpact = CanonicalizeRotation(
            startRotation + angularDelta * contactTime);
        Vector2d targetPositionAtImpact = target.SampleContinuousCollisionPosition(frameFraction);
        Fixed64 targetRotationAtImpact = target.SampleContinuousCollisionRotation(frameFraction);
        Vector2d sourceLinearVelocity = IsKinematic
            ? displacement / remainingTime
            : _linearVelocity;
        Fixed64 sourceAngularVelocity = IsKinematic
            ? angularDelta / remainingTime
            : _angularVelocity;
        Vector2d targetLinearVelocity = target.SampleContinuousCollisionLinearVelocity(frameFraction);
        Fixed64 targetAngularVelocity = target.SampleContinuousCollisionAngularVelocity(frameFraction);
        SolidBody2D? sourceResponseBody = IsKinematic ? null : this;
        SolidBody2D? targetResponseBody = target.IsKinematic ? null : target;
        Fixed64 restitution = PhysicsMaterial.CombineRestitution(
            Collider.Material,
            target.Collider.Material);
        bool sourceCenterResolved = Vector2d.TryAdd(
            sourcePositionAtImpact,
            ClampNearZero(Vector2d.Rotate(_localCenterOfMassOffset, sourceRotationAtImpact)),
            out Vector2d sourceCenterOfMass);
        bool targetCenterResolved = Vector2d.TryAdd(
            targetPositionAtImpact,
            ClampNearZero(Vector2d.Rotate(target._localCenterOfMassOffset, targetRotationAtImpact)),
            out Vector2d targetCenterOfMass);
        bool sourceContactArmResolved = Vector2d.TrySubtract(
            contact.PointA,
            sourceCenterOfMass,
            out Vector2d relativeContactPointA);
        bool targetContactArmResolved = Vector2d.TrySubtract(
            contact.PointB,
            targetCenterOfMass,
            out Vector2d relativeContactPointB);
        bool responseResolved = ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                sourceResponseBody,
                sourceLinearVelocity,
                sourceAngularVelocity,
                relativeContactPointA,
                targetResponseBody,
                targetLinearVelocity,
                targetAngularVelocity,
                relativeContactPointB,
                contact.Normal,
                restitution,
                Context.Settings.RestitutionVelocityThreshold,
                out ContactNormalVelocityDeltaResult2D response);
        if (!(sourceCenterResolved
                & targetCenterResolved
                & sourceContactArmResolved
                & targetContactArmResolved
                & responseResolved)
            || response.NormalVelocity >= -Fixed64.Epsilon)
        {
            return false;
        }

        Vector2d targetPostLinearVelocity = targetLinearVelocity;
        Fixed64 targetPostAngularVelocity = targetAngularVelocity;
        Vector2d targetResolvedPosition = default;
        Fixed64 impactElapsedTime = elapsedTime + remainingTime * contactTime;
        Fixed64 remainingAfterImpact = remainingTime * (Fixed64.One - contactTime);
        if (!IsKinematic)
        {
            bool sourceResponseAdmissible = CanApplyCollisionVelocityDeltas(
                    response.LinearVelocityDeltaA,
                    response.AngularVelocityDeltaA)
                & CanAppendContinuousCollisionFrameSegment(impactElapsedTime)
                & (target.IsKinematic
                    ? Context.Physics2D.CanAdmitContinuousCollisionCandidateRefresh(this)
                    : Context.Physics2D.CanAdmitContinuousCollisionCandidateRefresh(
                        this,
                        target));
            if (!sourceResponseAdmissible)
                return false;
        }

        if (!target.IsKinematic)
        {
            bool targetLinearVelocityResolved = Vector2d.TryAdd(
                targetLinearVelocity,
                response.LinearVelocityDeltaB,
                out targetPostLinearVelocity);
            bool targetAngularVelocityResolved = Fixed64.TryAdd(
                targetAngularVelocity,
                response.AngularVelocityDeltaB,
                out targetPostAngularVelocity);
            bool targetHandoffAdmissible = target.CanApplyContinuousCollisionHandoffState(
                targetPositionAtImpact,
                remainingAfterImpact,
                out targetResolvedPosition);
            if (!(targetLinearVelocityResolved
                    & targetAngularVelocityResolved
                    & targetHandoffAdmissible))
            {
                return false;
            }
        }

        if (!IsKinematic)
        {
            // The admission check above is immediately followed by this
            // reservation in the single-threaded arbiter, so failure is not a
            // runtime state. Reserve the already-proven body set once.
            _ = target.IsKinematic
                ? Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this)
                : Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this, target);
        }

        if (!target.IsKinematic)
        {
            target.ApplyContinuousCollisionHandoffStateReserved(
                targetResolvedPosition,
                targetRotationAtImpact,
                targetPostLinearVelocity,
                targetPostAngularVelocity,
                remainingAfterImpact,
                ignoredCollider2D: Collider);
        }

        if (!IsKinematic)
        {
            ApplyCollisionLinearVelocityDelta(response.LinearVelocityDeltaA);
            ApplyCollisionAngularVelocityDelta(response.AngularVelocityDeltaA);
        }

        return true;
    }

    [System.Runtime.CompilerServices.MethodImpl(
        System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveRotationalFrameFraction(
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        Fixed64 localTime) =>
        FixedMath.Clamp01((elapsedTime + remainingTime * localTime) / Context.DeltaTime);

    private void StopRotationalContinuousCollision(Vector2d contactNormal)
    {
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        RefreshAngularSpeed();
        RemoveClosingContinuousCollisionVelocity(contactNormal);
    }
}
