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
        bool immovable = Immovable;
        bool isKinematic = IsKinematic;
        Fixed64 mass = Mass;
        Fixed64 restitutionCoefficient = RestitutionCoefficient;
        Fixed64 frictionCoefficient = FrictionCoefficient;
        Vector2d gravity = Gravity;
        bool sleepEnabled = SleepEnabled;
        int sleepFrameThreshold = SleepFrameThreshold;
        Fixed64 sleepLinearSpeedThreshold = SleepLinearSpeedThreshold;
        Fixed64 sleepAngularSpeedThreshold = SleepAngularSpeedThreshold;

        RecordValues.Look(chronicler, ref active, "Active", false);
        RecordValues.Look(chronicler, ref immovable, "Immovable", false);
        RecordValues.Look(chronicler, ref isKinematic, "IsKinematic", false);
        RecordValues.Look(chronicler, ref _position, "Position");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref PreventAngularForces, "PreventAngularForces", false);
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
        RecordValues.Look(chronicler, ref restitutionCoefficient, "RestitutionCoefficient", Fixed64.Half);
        RecordValues.Look(chronicler, ref frictionCoefficient, "FrictionCoefficient", Fixed64.One);
        RecordValues.Look(chronicler, ref gravity, "Gravity", Vector2d.Zero);
        RecordValues.Look(chronicler, ref sleepEnabled, "SleepEnabled", true);
        RecordValues.Look(chronicler, ref sleepFrameThreshold, "SleepFrameThreshold", 16);
        RecordValues.Look(chronicler, ref sleepLinearSpeedThreshold, "SleepLinearSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref sleepAngularSpeedThreshold, "SleepAngularSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref _continuousCollisionMode, "ContinuousCollisionMode", ContinuousCollisionMode.Inherit);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            Active = active;
            _immovable = immovable;
            _isKinematic = isKinematic;
            Mass = mass;
            RestitutionCoefficient = restitutionCoefficient;
            FrictionCoefficient = frictionCoefficient;
            Gravity = gravity;
            SleepEnabled = sleepEnabled;
            SleepFrameThreshold = sleepFrameThreshold;
            SleepLinearSpeedThreshold = sleepLinearSpeedThreshold;
            SleepAngularSpeedThreshold = sleepAngularSpeedThreshold;
        }

        Collider.RecordData(chronicler);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            RefreshMassPropertiesFromColliderShape();
            ApplyLoadedState();
        }
    }

    private void ApplyLoadedState()
    {
        FixedTransform transform = Agent.Transform;
        Vector3d currentPosition = transform.Position;
        transform.Position = new Vector3d(_position.X, currentPosition.Y, _position.Y);
        transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            FixedMath.RadToDeg(_rotation),
            Fixed64.Zero);
        Collider.Rebuild();
    }
}
