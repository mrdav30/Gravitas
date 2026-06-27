//=======================================================================
// SolidBody.Serialization.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;

namespace Gravitas;

public partial class SolidBody
{
    public void RecordData(IChronicler chronicler)
    {
        GroundingMode groundingMode = GroundingMode;
        GroundProbeMode groundProbeMode = GroundProbeMode;
        Fixed64 groundProbeRadius = GroundProbeRadius;
        BodyFreezeAxes3D freezeAxes = FreezeAxes;
        bool isKinematic = IsKinematic;
        Fixed64 gravityScale = GravityScale;

        RecordValues.Look(chronicler, ref Debug, "Debug");
        RecordValues.Look(chronicler, ref Active, "Active");
        RecordValues.Look(chronicler, ref freezeAxes, "FreezeAxes", BodyFreezeAxes3D.None);
        RecordValues.Look(chronicler, ref isKinematic, "IsKinematic", false);
        RecordValues.Look(chronicler, ref _position2dUnmarked, "Position2d");
        RecordValues.Look(chronicler, ref _heightPosUnmarked, "HeightPos");
        RecordValues.Look(chronicler, ref _spawnedPosition, "SpawnedPosition");
        RecordValues.Look(chronicler, ref _lastPosition, "LastPosition");
        RecordValues.Look(chronicler, ref GroundOriginOffset, "GroundOriginOffset");
        RecordValues.Look(chronicler, ref GroundedDistanceRay, "GroundedDistanceRay");
        RecordValues.Look(chronicler, ref GroundDownDistanceOnAir, "GroundDownDistanceOnAir");
        RecordValues.Look(chronicler, ref groundingMode, "GroundingMode", GroundingMode.Automatic);
        RecordValues.Look(chronicler, ref groundProbeMode, "GroundProbeMode", GroundProbeMode.Auto);
        RecordValues.Look(chronicler, ref groundProbeRadius, "GroundProbeRadius");
        RecordValues.Look(chronicler, ref _skipGroundingCheck, "SkipGroundingCheck", false);
        RecordValues.Look(chronicler, ref _lastGroundCheckFrame, "LastGroundCheckFrame");
        RecordValues.Look(chronicler, ref StepOffset, "StepOffset");
        RecordValues.Look(chronicler, ref _groundNormal, "GroundNormal");
        RecordValues.Look(chronicler, ref _hitPlatformPosition, "HitPlatformPosition");
        RecordValues.Look(chronicler, ref _hitPoint, "HitPoint");
        RecordValues.Look(chronicler, ref _isGrounded, "IsGrounded");
        RecordValues.Look(chronicler, ref _wasGrounded, "WasGrounded");
        RecordValues.Look(chronicler, ref _lastGroundedPosition, "LastGroundedPosition");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref _linearVelocity, "LinearVelocity");
        RecordValues.Look(chronicler, ref _linearDirection, "LinearDirection");
        RecordValues.Look(chronicler, ref _angularVelocity, "AngularVelocity");
        RecordValues.Look(chronicler, ref _angularDirection, "AngularDirection");
        RecordValues.Look(chronicler, ref _deltaTorque, "DeltaTorque");
        RecordValues.Look(chronicler, ref _localCenterOfMassOffset, "LocalCenterOfMassOffset");
        RecordValues.Look(chronicler, ref _centerOfMassOffsetExplicit, "CenterOfMassOffsetExplicit", false);
        RecordValues.Look(chronicler, ref gravityScale, "GravityScale", Fixed64.One);
        RecordValues.Look(chronicler, ref _isSleeping, "IsSleeping");
        RecordValues.Look(chronicler, ref _sleepFrameCount, "SleepFrameCount");
        RecordValues.Look(chronicler, ref _sleepEnabled, "SleepEnabled", true);
        RecordValues.Look(chronicler, ref _sleepFrameThreshold, "SleepFrameThreshold", 16);
        RecordValues.Look(chronicler, ref _sleepLinearSpeedThreshold, "SleepLinearSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref _sleepAngularSpeedThreshold, "SleepAngularSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref _continuousCollisionMode, "ContinuousCollisionMode", ContinuousCollisionMode.Inherit);
        RecordValues.Look(chronicler, ref _linearSpeed, "LinearSpeed");
        RecordValues.Look(chronicler, ref _linearAccelerationStore, "LinearAccelerationStore");
        RecordValues.Look(chronicler, ref _deltaAcceleration, "DeltaAcceleration");
        RecordValues.Look(chronicler, ref _linearAcceleration, "LinearAcceleration");
        RecordValues.Look(chronicler, ref _angularSpeed, "AngularSpeed");
        RecordValues.Look(chronicler, ref _angularAccelerationStore, "AngularAccelerationStore");
        RecordValues.Look(chronicler, ref _angularAcceleration, "AngularAcceleration");
        RecordValues.Look(chronicler, ref _impulseStore, "ImpulseStore");
        RecordValues.Look(chronicler, ref _positionCorrection, "PositionCorrection");
        RecordValues.Look(chronicler, ref _timeScaledAcceleration, "TimeScaledAcceleration");
        RecordValues.Look(chronicler, ref _timeScaledDeceleration, "TimeScaledDeceleration");
        RecordValues.Look(chronicler, ref _decelerating, "Decelerating");
        RecordValues.Look(chronicler, ref _isVelocityConstant, "IsVelocityConstant");
        RecordValues.Look(chronicler, ref LinearDragCoefficient, "LinearDragCoefficient");
        RecordValues.Look(chronicler, ref AngularDragCoefficient, "AngularDragCoefficient");
        RecordValues.Look(chronicler, ref _normalForce, "NormalForce");
        RecordValues.Look(chronicler, ref Mass, "Mass");

        if (chronicler.Mode == SerializationMode.Loading)
        {
            _freezeAxes = freezeAxes;
            _isKinematic = isKinematic;
            GroundingMode = groundingMode;
            GroundProbeMode = groundProbeMode;
            GroundProbeRadius = groundProbeRadius;
            GravityScale = gravityScale;
            _hitPlatform = null;
        }

        Collider?.RecordData(chronicler);

        if (chronicler.Mode == SerializationMode.Loading)
            ApplyLoadedState();
    }

    private void ApplyLoadedState()
    {
        _positionTransform.Position = Position3d;
        _rotationTransform.Rotation = Rotation;

        _positionMutated = false;
        _positionChangedBuffer = false;
        _rotationMutated = false;
        _rotationChangedBuffer = false;
        _settingVisualsCounter = 0;
        _rotationSpeed = DefaultRotationSpeed;
        _rotationInterpoleSpeed = Fixed64.Zero;
        _visualPosition = Position3d;
        _lastVisualPosition = Position3d;
        _visualRotation = Rotation;
        _lastVisualRotation = Rotation;

        RefreshMassPropertiesFromColliderShape();
        ApplyFreezeConstraintsToMotion();

        Collider?.Simulate();
    }

}
