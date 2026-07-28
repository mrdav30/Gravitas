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
        SolidBody2D? sourceResponseBody = HasSolverMobility ? this : null;
        SolidBody2D? targetResponseBody = target.HasSolverMobility ? target : null;
        Fixed64 restitution = PhysicsMaterial.CombineRestitution(
            Collider.Material,
            target.Collider.Material);
        var sourceCenter = new ContactAnchor2D(
            sourcePositionAtImpact,
            sourceRotationAtImpact,
            _localCenterOfMassOffset);
        var targetCenter = new ContactAnchor2D(
            targetPositionAtImpact,
            targetRotationAtImpact,
            target._localCenterOfMassOffset);
        ContactLever2D sourceContactArm =
            ContactLever2D.Create(contact.AnchorA, sourceCenter);
        ContactLever2D targetContactArm =
            ContactLever2D.Create(contact.AnchorB, targetCenter);
        ContactNormalVelocityDeltaResult2D response = default;
        bool responseResolved = !sourceContactArm.IsExact
            && !targetContactArm.IsExact
            && ExactContactLever2D.CanUseCompactResponse(
                sourceResponseBody,
                sourceLinearVelocity,
                sourceAngularVelocity,
                sourceContactArm.Vector,
                targetResponseBody,
                targetLinearVelocity,
                targetAngularVelocity,
                targetContactArm.Vector,
                contact.Normal)
            && ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                sourceResponseBody,
                sourceLinearVelocity,
                sourceAngularVelocity,
                sourceContactArm.Vector,
                targetResponseBody,
                targetLinearVelocity,
                targetAngularVelocity,
                targetContactArm.Vector,
                contact.Normal,
                restitution,
                Context.Settings.RestitutionVelocityThreshold,
                out response);
        if (!responseResolved)
        {
            responseResolved =
                ContactNormalImpulse2D.TryCalculateVelocityDeltasExact(
                    sourceResponseBody,
                    sourceLinearVelocity,
                    sourceAngularVelocity,
                    contact.AnchorA.GetXZLeverFrom(sourceCenter),
                    targetResponseBody,
                    targetLinearVelocity,
                    targetAngularVelocity,
                    contact.AnchorB.GetXZLeverFrom(targetCenter),
                    contact.Normal,
                    restitution,
                    Context.Settings.RestitutionVelocityThreshold,
                    out response);
        }
        if (!responseResolved
            || !response.IsClosing
            || (response.HasRepresentableNormalVelocity
                && response.NormalVelocity >= -Fixed64.Epsilon))
        {
            return false;
        }

        Vector2d targetPostLinearVelocity = targetLinearVelocity;
        Fixed64 targetPostAngularVelocity = targetAngularVelocity;
        Vector2d targetResolvedPosition = default;
        Fixed64 impactElapsedTime = elapsedTime + remainingTime * contactTime;
        Fixed64 remainingAfterImpact = remainingTime * (Fixed64.One - contactTime);
        if (sourceResponseBody != null)
        {
            bool sourceResponseAdmissible = CanApplyCollisionVelocityDeltas(
                    response.LinearVelocityDeltaA,
                    response.AngularVelocityDeltaA)
                & CanAppendContinuousCollisionFrameSegment(impactElapsedTime)
                & (targetResponseBody == null
                    ? Context.Physics2D.CanAdmitContinuousCollisionCandidateRefresh(this)
                    : Context.Physics2D.CanAdmitContinuousCollisionCandidateRefresh(
                        this,
                        target));
            if (!sourceResponseAdmissible)
                return false;
        }

        if (targetResponseBody != null)
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
                targetRotationAtImpact,
                remainingAfterImpact,
                out targetResolvedPosition);
            if (!(targetLinearVelocityResolved
                    & targetAngularVelocityResolved
                    & targetHandoffAdmissible))
            {
                return false;
            }
        }

        if (sourceResponseBody != null)
        {
            // The admission check above is immediately followed by this
            // reservation in the single-threaded arbiter, so failure is not a
            // runtime state. Reserve the already-proven body set once.
            _ = targetResponseBody == null
                ? Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this)
                : Context.Physics2D.TryReserveContinuousCollisionCandidateRefresh(this, target);
        }

        if (targetResponseBody != null)
        {
            target.ApplyContinuousCollisionHandoffStateReserved(
                targetResolvedPosition,
                targetRotationAtImpact,
                targetPostLinearVelocity,
                targetPostAngularVelocity,
                remainingAfterImpact,
                ignoredCollider2D: Collider);
        }

        if (sourceResponseBody != null)
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
