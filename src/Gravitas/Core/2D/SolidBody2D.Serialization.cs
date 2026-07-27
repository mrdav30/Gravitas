//=======================================================================
// SolidBody2D.Serialization.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    public void RecordData(IChronicler chronicler)
    {
        bool active = Active;
        BodyFreezeAxes2D freezeAxes = FreezeAxes;
        BodyMotionType motionType = MotionType;
        Fixed64 mass = Mass;
        Vector2d gravity = Gravity;
        Fixed64 gravityScale = GravityScale;
        bool sleepEnabled = SleepEnabled;
        int sleepFrameThreshold = SleepFrameThreshold;
        Fixed64 sleepLinearSpeedThreshold = SleepLinearSpeedThreshold;
        Fixed64 sleepAngularSpeedThreshold = SleepAngularSpeedThreshold;
        GroundingMode groundingMode = GroundingMode;
        GroundProbeMode2D groundProbeMode = GroundProbeMode;
        bool useGravityDerivedGroundUpDirection = UseGravityDerivedGroundUpDirection;
        Vector2d groundUpDirection = GroundUpDirection;
        Fixed64 groundProbeRadius = GroundProbeRadius;
        Fixed64 groundedDistanceRay = GroundedDistanceRay;
        Fixed64 groundDownDistanceOnAir = GroundDownDistanceOnAir;
        Fixed64 groundMinNormalDot = GroundMinNormalDot;
        ContinuousCollisionMode continuousCollisionMode = ContinuousCollisionMode;

        RecordValues.Look(chronicler, ref active, "Active", false);
        RecordValues.Look(chronicler, ref freezeAxes, "FreezeAxes", BodyFreezeAxes2D.None);
        RecordValues.Look(chronicler, ref motionType, "MotionType", BodyMotionType.Dynamic);
        if (chronicler.Mode == SerializationMode.Loading)
        {
            PreflightLoadedMotionType(motionType);
            SwiftThrowHelper.ThrowIfArgument(
                (freezeAxes & ~BodyFreezeAxes2D.All) != BodyFreezeAxes2D.None,
                nameof(freezeAxes),
                "Unsupported 2D freeze axis bits.");
        }

        RecordValues.Look(chronicler, ref _position, "Position");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref _localCenterOfMassOffset, "LocalCenterOfMassOffset");
        RecordValues.Look(chronicler, ref _centerOfMassOffsetExplicit, "CenterOfMassOffsetExplicit", false);
        RecordValues.Look(chronicler, ref _linearVelocity, "LinearVelocity");
        RecordValues.Look(chronicler, ref _linearAccelerationStore, "LinearAccelerationStore");
        RecordValues.Look(chronicler, ref _deltaAcceleration, "DeltaAcceleration");
        RecordValues.Look(chronicler, ref _linearSpeed, "LinearSpeed");
        RecordValues.Look(chronicler, ref _angularVelocity, "AngularVelocity");
        RecordValues.Look(chronicler, ref _angularAccelerationStore, "AngularAccelerationStore");
        RecordValues.Look(chronicler, ref _deltaAngularAcceleration, "DeltaAngularAcceleration");
        RecordValues.Look(chronicler, ref _angularSpeed, "AngularSpeed");
        RecordValues.Look(chronicler, ref _isSleeping, "IsSleeping");
        RecordValues.Look(chronicler, ref _sleepFrameCount, "SleepFrameCount");
        RecordValues.Look(chronicler, ref mass, "Mass");
        RecordValues.Look(chronicler, ref gravity, "Gravity", Vector2d.Zero);
        RecordValues.Look(chronicler, ref gravityScale, "GravityScale", Fixed64.One);
        RecordValues.Look(chronicler, ref sleepEnabled, "SleepEnabled", true);
        RecordValues.Look(chronicler, ref sleepFrameThreshold, "SleepFrameThreshold", 16);
        RecordValues.Look(chronicler, ref sleepLinearSpeedThreshold, "SleepLinearSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref sleepAngularSpeedThreshold, "SleepAngularSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref groundingMode, "GroundingMode", GroundingMode.Automatic);
        RecordValues.Look(chronicler, ref groundProbeMode, "GroundProbeMode", GroundProbeMode2D.Auto);
        RecordValues.Look(chronicler, ref useGravityDerivedGroundUpDirection, "UseGravityDerivedGroundUpDirection", true);
        RecordValues.Look(chronicler, ref groundUpDirection, "GroundUpDirection", Vector2d.Forward);
        RecordValues.Look(chronicler, ref groundProbeRadius, "GroundProbeRadius");
        RecordValues.Look(chronicler, ref groundedDistanceRay, "GroundedDistanceRay", Fixed64.Half);
        RecordValues.Look(chronicler, ref groundDownDistanceOnAir, "GroundDownDistanceOnAir", Fixed64.Half);
        RecordValues.Look(chronicler, ref groundMinNormalDot, "GroundMinNormalDot", Fixed64.Half);
        RecordValues.Look(chronicler, ref _isGrounded, "IsGrounded");
        RecordValues.Look(chronicler, ref _wasGrounded, "WasGrounded");
        RecordValues.Look(chronicler, ref _groundNormal, "GroundNormal");
        RecordValues.Look(chronicler, ref _hasGroundPoint, "HasGroundPoint");
        RecordValues.Look(chronicler, ref _groundPoint, "GroundPoint");
        RecordValues.Look(chronicler, ref _lastGroundedPosition, "LastGroundedPosition");
        RecordValues.Look(chronicler, ref continuousCollisionMode, "ContinuousCollisionMode", ContinuousCollisionMode.Inherit);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            ApplyLoadedMotionType(motionType);
            DiscardContinuousCollisionHandoff();
            InvalidateContinuousCollisionFrame();
            ContinuousCollisionMode = continuousCollisionMode;
            _rotation = CanonicalizeRotation(_rotation);
            _freezeAxes = freezeAxes;
            Mass = mass;
            Gravity = gravity;
            GravityScale = gravityScale;
            SleepEnabled = sleepEnabled;
            SleepFrameThreshold = sleepFrameThreshold;
            SleepLinearSpeedThreshold = sleepLinearSpeedThreshold;
            SleepAngularSpeedThreshold = sleepAngularSpeedThreshold;
            _groundingMode = groundingMode;
            _groundProbeMode = groundProbeMode;
            _useGravityDerivedGroundUpDirection = useGravityDerivedGroundUpDirection;
            _groundUpDirection = groundUpDirection.MagnitudeSquared > Fixed64.Epsilon
                ? groundUpDirection.Normalized
                : DefaultGroundUpDirection;
            _groundProbeRadius = groundProbeRadius < Fixed64.Zero ? Fixed64.Zero : groundProbeRadius;
            GroundedDistanceRay = groundedDistanceRay;
            GroundDownDistanceOnAir = groundDownDistanceOnAir;
            GroundMinNormalDot = groundMinNormalDot;
            _groundNormal = _groundNormal.MagnitudeSquared > Fixed64.Epsilon
                ? _groundNormal.Normalized
                : Vector2d.Zero;
            _groundedTransitionCapturedForStep = false;
            _groundCollider = null;
            _groundColliderBroadPhaseVersion = 0;
            ClearGroundContactCandidate();
        }

        Collider.RecordData(chronicler);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            RefreshMassPropertiesFromColliderShape();
            ApplyLoadedFreezeConstraintsToMotion();
            ApplyLoadedState();
            if (!active || Collider.Id < 0)
                Deactivate();
            _groundingStateVersion++;
        }
    }

    private void ApplyLoadedState()
    {
        SetHostWorldPose(Agent.Transform, _position, _rotation);
        Collider.Rebuild();
    }

    private void ApplyLoadedFreezeConstraintsToMotion()
    {
        _linearVelocity = ProjectLinearMotion(_linearVelocity);
        _linearAccelerationStore = ProjectLinearMotion(_linearAccelerationStore);
        _deltaAcceleration = ProjectLinearMotion(_deltaAcceleration);
        RefreshLinearSpeed();

        if (IsRotationFullyFrozen)
        {
            _angularVelocity = Fixed64.Zero;
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
            _angularSpeed = Fixed64.Zero;
            return;
        }

        RefreshAngularSpeed();
    }
}
