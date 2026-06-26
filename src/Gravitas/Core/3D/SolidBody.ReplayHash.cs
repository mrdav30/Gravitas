//=======================================================================
// SolidBody.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

public partial class SolidBody
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("body.3d", 1);
        writer.WriteInt32(_dynamicId);
        writer.WriteBool(Debug);
        writer.WriteBool(Active);
        writer.WriteBool(_immovable);
        writer.WriteBool(_isKinematic);
        writer.WriteVector2d(_position2dUnmarked);
        writer.WriteFixed64(_heightPosUnmarked);
        writer.WriteVector3d(_spawnedPosition);
        writer.WriteVector3d(_lastPosition);
        writer.WriteFixed64(GroundOriginOffset);
        writer.WriteFixed64(GroundedDistanceRay);
        writer.WriteFixed64(GroundDownDistanceOnAir);
        writer.WriteEnum(GroundingMode);
        writer.WriteEnum(GroundProbeMode);
        writer.WriteFixed64(GroundProbeRadius);
        writer.WriteBool(_skipGroundingCheck);
        writer.WriteInt32(_lastGroundCheckFrame);
        writer.WriteFixed64(StepOffset);
        writer.WriteVector3d(_groundNormal);
        writer.WriteVector3d(_hitPlatformPosition);
        writer.WriteVector3d(_hitPoint);
        writer.WriteBool(_isGrounded);
        writer.WriteBool(_wasGrounded);
        writer.WriteVector3d(_lastGroundedPosition);
        writer.WriteQuaternion(_rotation);
        writer.WriteBool(PreventAngularForces);
        writer.WriteVector3d(_linearVelocity);
        writer.WriteVector3d(_linearDirection);
        writer.WriteVector3d(_angularVelocity);
        writer.WriteVector3d(_angularDirection);
        writer.WriteVector3d(_deltaTorque);
        writer.WriteVector3d(_localCenterOfMassOffset);
        writer.WriteBool(_centerOfMassOffsetExplicit);
        writer.WriteFixed64(RestitutionCoefficient);
        writer.WriteFixed64(_gravityScale);
        writer.WriteBool(_isSleeping);
        writer.WriteInt32(_sleepFrameCount);
        writer.WriteBool(_sleepEnabled);
        writer.WriteInt32(_sleepFrameThreshold);
        writer.WriteFixed64(_sleepLinearSpeedThreshold);
        writer.WriteFixed64(_sleepAngularSpeedThreshold);
        writer.WriteEnum(_continuousCollisionMode);
        writer.WriteFixed64(_linearSpeed);
        writer.WriteVector3d(_linearAccelerationStore);
        writer.WriteVector3d(_deltaAcceleration);
        writer.WriteVector3d(_linearAcceleration);
        writer.WriteFixed64(_angularSpeed);
        writer.WriteVector3d(_angularAccelerationStore);
        writer.WriteVector3d(_angularAcceleration);
        writer.WriteVector3d(_impulseStore);
        writer.WriteVector2d(_positionCorrection);
        writer.WriteVector3d(_timeScaledAcceleration);
        writer.WriteVector3d(_timeScaledDeceleration);
        writer.WriteBool(_decelerating);
        writer.WriteBool(_isVelocityConstant);
        writer.WriteFixed64(LinearDragCoefficient);
        writer.WriteFixed64(AngularDragCoefficient);
        writer.WriteFixed64(_frictionCoefficient);
        writer.WriteVector3d(_normalForce);
        writer.WriteFixed64(Mass);

        writer.WriteSection("body.3d.ccd-authoritative", 1);
        writer.WriteBool(_continuousCollisionHandoffPending);
        if (_continuousCollisionHandoffPending)
        {
            writer.WriteInt32(_continuousCollisionHandoffToken);
            writer.WriteFixed64(_continuousCollisionHandoffRemainingTime);
            writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider3D?.Id ?? -1);
            writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider2D?.Id ?? -1);
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("body.3d.solver-caches", 1);
        writer.WriteInt32(_continuousCollisionFrameToken);
        writer.WriteVector3d(_continuousCollisionFrameStart);
        writer.WriteVector3d(_continuousCollisionFrameDisplacement);
        writer.WriteQuaternion(_continuousCollisionFrameRotation);
        writer.WriteInt32(_continuousCollisionHandoffToken);
        writer.WriteFixed64(_continuousCollisionHandoffRemainingTime);
        writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider3D?.Id ?? -1);
        writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider2D?.Id ?? -1);
        writer.WriteInt32(LastContinuousCollisionToiIterationCount);
        writer.WriteBool(LastContinuousCollisionToiIterationLimitReached);
        writer.WriteFixed3x3(_inertiaTensor);
        writer.WriteFixed3x3(_worldInertiaTensor);
        writer.WriteFixed3x3(_inverseLocalInertiaTensor);
        writer.WriteFixed3x3(_inverseInertiaTensor);
        writer.WriteInt32(_continuousCollisionHits.Count);
        writer.WriteInt32(_continuousMixedCollisionHits.Count);
        writer.WriteInt32(_rotationalContinuousCollisionManifold.Count);
    }
}
