//=======================================================================
// SolidBody2D.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp.Chronicler;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("body.2d", 2);
        writer.WriteInt32(DynamicId);
        writer.WriteBool(Active);
        writer.WriteEnum(_freezeAxes);
        writer.WriteBool(_isKinematic);
        writer.WriteBool(_isDynamic);
        writer.WriteVector2d(_position);
        writer.WriteFixed64(_rotation);
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
        writer.WriteVector2d(Gravity);
        writer.WriteFixed64(_gravityScale);
        writer.WriteBool(SleepEnabled);
        writer.WriteInt32(SleepFrameThreshold);
        writer.WriteFixed64(SleepLinearSpeedThreshold);
        writer.WriteFixed64(_sleepAngularSpeedThreshold);
        writer.WriteEnum(_groundingMode);
        writer.WriteEnum(_groundProbeMode);
        writer.WriteBool(_useGravityDerivedGroundUpDirection);
        writer.WriteVector2d(_groundUpDirection);
        writer.WriteFixed64(_groundProbeRadius);
        writer.WriteFixed64(GroundedDistanceRay);
        writer.WriteFixed64(GroundDownDistanceOnAir);
        writer.WriteFixed64(GroundMinNormalDot);
        writer.WriteBool(_isGrounded);
        writer.WriteBool(_wasGrounded);
        writer.WriteVector2d(_groundNormal);
        writer.WriteVector2d(_groundPoint);
        writer.WriteVector2d(_lastGroundedPosition);
        writer.WriteEnum(_continuousCollisionMode);

        writer.WriteSection("body.2d.ccd-authoritative", 2);
        writer.WriteBool(_continuousCollisionHandoffPending);
        if (_continuousCollisionHandoffPending)
        {
            writer.WriteInt32(_continuousCollisionHandoffToken);
            writer.WriteFixed64(_continuousCollisionHandoffRemainingTime);
            writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider3D?.ReplayOrdinal ?? -1);
            writer.WriteInt32(_continuousCollisionHandoffIgnoredCollider2D?.ReplayOrdinal ?? -1);
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("body.2d.solver-caches", 2);
        writer.WriteInt32(_continuousCollisionFrameToken);
        writer.WriteVector2d(_continuousCollisionFrameStart);
        writer.WriteVector2d(_continuousCollisionFrameDisplacement);
        writer.WriteFixed64(_continuousCollisionFrameRotation);
        writer.WriteInt32(_continuousCollisionHandoffToken);
        writer.WriteFixed64(_continuousCollisionHandoffRemainingTime);
        writer.WriteInt32(LastContinuousCollisionToiIterationCount);
        writer.WriteBool(LastContinuousCollisionToiIterationLimitReached);
        writer.WriteFixed64(_momentOfInertia);
        writer.WriteFixed64(_inverseMomentOfInertia);
        writer.WriteInt32(_continuousCollisionHits.Count);
        writer.WriteInt32(_continuousMixedCollisionHits.Count);
    }
}
