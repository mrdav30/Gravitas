//=======================================================================
// SolidBody.ContinuousCollision.Rotational.Response.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Materials;

namespace Gravitas;

public partial class SolidBody
{
    internal bool TryApplyRotationalContinuousCollisionResponse(
        SolidBody target,
        ManifoldContact contact,
        Fixed64 localContactTime,
        Vector3d sourceSegmentStart,
        Vector3d sourceDisplacement,
        FixedQuaternion sourceSegmentStartRotation,
        Fixed64 elapsedTime,
        Fixed64 remainingTime,
        bool sourceIsKinematic)
    {
        Fixed64 frameFraction = ResolveRotationalFrameFraction(
            elapsedTime,
            remainingTime,
            localContactTime);
        Fixed64 consumedTime = remainingTime * localContactTime;
        Fixed64 impactElapsedTime = elapsedTime + consumedTime;
        Fixed64 remainingAfterImpact = remainingTime - consumedTime;
        Vector3d sourcePosition = sourceSegmentStart
            + sourceDisplacement * localContactTime;
        FixedQuaternion sourceRotation = sourceIsKinematic
            ? FixedQuaternion.Slerp(
                sourceSegmentStartRotation,
                ContinuousCollisionFrameTargetRotation,
                localContactTime).Normalized
            : IntegrateAngularRotation(
                sourceSegmentStartRotation,
                _angularVelocity,
                consumedTime);
        Vector3d sourceLinearVelocity = sourceIsKinematic
            ? SampleContinuousCollisionLinearVelocity(frameFraction)
            : _linearVelocity;
        Vector3d sourceAngularVelocity = sourceIsKinematic
            ? SampleContinuousCollisionAngularVelocity(frameFraction)
            : _angularVelocity;
        Vector3d targetPosition = target.SampleContinuousCollisionPosition(frameFraction);
        FixedQuaternion targetRotation = target.SampleContinuousCollisionRotation(frameFraction);
        Vector3d targetLinearVelocity = target.SampleContinuousCollisionLinearVelocity(frameFraction);
        Vector3d targetAngularVelocity = target.SampleContinuousCollisionAngularVelocity(frameFraction);

        OrderRotationalContinuousCollisionPair(
            target.Collider,
            out _,
            out _,
            out bool sourceIsA);
        SolidBody bodyA = sourceIsA ? this : target;
        SolidBody bodyB = sourceIsA ? target : this;
        Vector3d positionA = sourceIsA ? sourcePosition : targetPosition;
        Vector3d positionB = sourceIsA ? targetPosition : sourcePosition;
        FixedQuaternion rotationA = sourceIsA ? sourceRotation : targetRotation;
        FixedQuaternion rotationB = sourceIsA ? targetRotation : sourceRotation;
        Vector3d linearVelocityA = sourceIsA ? sourceLinearVelocity : targetLinearVelocity;
        Vector3d linearVelocityB = sourceIsA ? targetLinearVelocity : sourceLinearVelocity;
        Vector3d angularVelocityA = sourceIsA ? sourceAngularVelocity : targetAngularVelocity;
        Vector3d angularVelocityB = sourceIsA ? targetAngularVelocity : sourceAngularVelocity;
        bool armAResolved = contact.AnchorA.TryGetOffsetFrom(
            new ContactAnchor(
                positionA,
                rotationA,
                bodyA._localCenterOfMassOffset),
            out Vector3d contactArmA);
        bool armBResolved = contact.AnchorB.TryGetOffsetFrom(
            new ContactAnchor(
                positionB,
                rotationB,
                bodyB._localCenterOfMassOffset),
            out Vector3d contactArmB);
        if (!(armAResolved & armBResolved))
        {
            return false;
        }

        Fixed64 restitution = PhysicsMaterial.CombineRestitution(
            Collider.Material,
            target.Collider.Material);
        Fixed3x3 originalSourceInverseInertia = _inverseInertiaTensor;
        Fixed3x3 originalTargetInverseInertia = target._inverseInertiaTensor;
        ContactNormalVelocityDeltaResult3D result;
        bool calculated;
        try
        {
            _inverseInertiaTensor = ResolveContinuousCollisionInverseInertia(sourceRotation);
            target._inverseInertiaTensor = target.ResolveContinuousCollisionInverseInertia(targetRotation);
            calculated = ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                bodyA,
                linearVelocityA,
                angularVelocityA,
                contactArmA,
                bodyB,
                linearVelocityB,
                angularVelocityB,
                contactArmB,
                contact.Normal,
                restitution,
                Context.Settings.RestitutionVelocityThreshold,
                out result);
        }
        finally
        {
            _inverseInertiaTensor = originalSourceInverseInertia;
            target._inverseInertiaTensor = originalTargetInverseInertia;
        }

        if (!calculated || result.NormalVelocity >= -Fixed64.Epsilon)
        {
            return false;
        }

        Vector3d sourceLinearDelta = sourceIsA
            ? result.LinearVelocityDeltaA
            : result.LinearVelocityDeltaB;
        Vector3d sourceAngularDelta = sourceIsA
            ? result.AngularVelocityDeltaA
            : result.AngularVelocityDeltaB;
        Vector3d targetLinearDelta = sourceIsA
            ? result.LinearVelocityDeltaB
            : result.LinearVelocityDeltaA;
        Vector3d targetAngularDelta = sourceIsA
            ? result.AngularVelocityDeltaB
            : result.AngularVelocityDeltaA;
        bool sourceNeedsRefresh = HasSolverMobility;
        bool targetNeedsRefresh = target.HasSolverMobility;
        Vector3d postSourceLinearVelocity = sourceLinearVelocity;
        Vector3d postSourceAngularVelocity = sourceAngularVelocity;
        bool sourceStateResolved = !sourceNeedsRefresh
            || (Vector3d.TryAdd(
                    sourceLinearVelocity,
                    ProjectLinearMotion(sourceLinearDelta),
                    out postSourceLinearVelocity)
                & Vector3d.TryAdd(
                    sourceAngularVelocity,
                    ProjectAngularMotion(sourceAngularDelta),
                    out postSourceAngularVelocity)
                & CanAppendContinuousCollisionSegment(impactElapsedTime));
        if (!sourceStateResolved)
        {
            return false;
        }

        Vector3d resolvedTargetPosition = default;
        Vector3d resolvedTargetLinearVelocity = targetLinearVelocity;
        Vector3d resolvedTargetAngularVelocity = targetAngularVelocity;
        bool targetStateResolved = !targetNeedsRefresh
            || (Vector3d.TryAdd(
                    targetLinearVelocity,
                    target.ProjectLinearMotion(targetLinearDelta),
                    out resolvedTargetLinearVelocity)
                & Vector3d.TryAdd(
                    targetAngularVelocity,
                    target.ProjectAngularMotion(targetAngularDelta),
                    out resolvedTargetAngularVelocity)
                & target.CanApplyContinuousCollisionHandoff(
                    targetPosition,
                    targetRotation,
                    remainingAfterImpact,
                    out resolvedTargetPosition));
        if (!targetStateResolved)
        {
            return false;
        }

        if (sourceNeedsRefresh && targetNeedsRefresh)
            _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this, target);
        else if (sourceNeedsRefresh)
            _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(this);
        else
            _ = Context.Physics.TryReserveContinuousCollisionCandidateRefresh(target);

        if (sourceNeedsRefresh)
        {
            ApplyCollisionVelocityState(
                postSourceLinearVelocity,
                postSourceAngularVelocity);
            AppendContinuousCollisionSegment(
                sourcePosition,
                sourceRotation,
                _linearVelocity,
                _angularVelocity,
                impactElapsedTime);
            Context.Physics.RefreshContinuousCollisionCandidate(this);
        }

        if (targetNeedsRefresh)
        {
            target.ApplyContinuousCollisionHandoffReserved(
                resolvedTargetPosition,
                targetRotation,
                resolvedTargetLinearVelocity,
                resolvedTargetAngularVelocity,
                remainingAfterImpact,
                ignoredCollider3D: Collider);
        }

        return true;
    }

    private Fixed3x3 ResolveContinuousCollisionInverseInertia(FixedQuaternion rotation)
    {
        if (_inverseLocalInertiaTensor == Fixed3x3.Zero)
            return Fixed3x3.Zero;

        Fixed3x3 orientation = rotation.ToMatrix3x3();
        return orientation
            * _inverseLocalInertiaTensor
            * rotation.Conjugate().ToMatrix3x3();
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
