//=======================================================================
// SolidBody2D.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

public sealed partial class SolidBody2D
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("body.2d", 1);
        writer.WriteInt32(DynamicId);
        writer.WriteBool(Active);
        writer.WriteBool(_immovable);
        writer.WriteBool(_isKinematic);
        writer.WriteBool(_isDynamic);
        writer.WriteVector2d(_position);
        writer.WriteFixed64(_rotation);
        writer.WriteBool(PreventAngularForces);
        writer.WriteVector2d(_localCenterOfMassOffset);
        writer.WriteBool(_centerOfMassOffsetExplicit);
        writer.WriteVector2d(_linearVelocity);
        writer.WriteVector2d(_linearAccelerationStore);
        writer.WriteVector2d(_deltaAcceleration);
        writer.WriteFixed64(_linearSpeed);
        writer.WriteFixed64(_angularVelocity);
        writer.WriteFixed64(_angularAccelerationStore);
        writer.WriteFixed64(_deltaAngularAcceleration);
        writer.WriteFixed64(_angularSpeed);
        writer.WriteBool(_isSleeping);
        writer.WriteInt32(_sleepFrameCount);
        writer.WriteFixed64(_mass);
        writer.WriteFixed64(RestitutionCoefficient);
        writer.WriteFixed64(FrictionCoefficient);
        writer.WriteVector2d(Gravity);
        writer.WriteFixed64(_gravityScale);
        writer.WriteBool(SleepEnabled);
        writer.WriteInt32(SleepFrameThreshold);
        writer.WriteFixed64(SleepLinearSpeedThreshold);
        writer.WriteFixed64(_sleepAngularSpeedThreshold);
        writer.WriteEnum(_continuousCollisionMode);

        writer.WriteSection("body.2d.ccd-authoritative", 1);
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

        writer.WriteSection("body.2d.solver-caches", 1);
        writer.WriteInt32(_continuousCollisionFrameToken);
        writer.WriteVector2d(_continuousCollisionFrameStart);
        writer.WriteVector2d(_continuousCollisionFrameDisplacement);
        writer.WriteFixed64(_continuousCollisionFrameRotation);
        writer.WriteInt32(_continuousCollisionHandoffToken);
        writer.WriteFixed64(_continuousCollisionHandoffRemainingTime);
        writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider3D?.Id ?? -1);
        writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider2D?.Id ?? -1);
        writer.WriteInt32(LastContinuousCollisionToiIterationCount);
        writer.WriteBool(LastContinuousCollisionToiIterationLimitReached);
        writer.WriteFixed64(_momentOfInertia);
        writer.WriteFixed64(_inverseMomentOfInertia);
        writer.WriteInt32(_continuousCollisionHits.Count);
        writer.WriteInt32(_continuousMixedCollisionHits.Count);
    }
}
