//=======================================================================
// StiffBody2D.Motion.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class StiffBody2D
{
    public void AddForce(Vector2d force)
    {
        if (force != Vector2d.Zero)
            Wake();

        _deltaAcceleration += force * InverseMass;
    }

    public void AddTorque(Fixed64 torque)
    {
        if (torque == Fixed64.Zero || !CanRotate)
            return;

        Wake();
        _deltaAngularAcceleration += torque * EffectiveInverseMomentOfInertia;
    }

    public void AddAngularImpulse(Fixed64 impulse)
    {
        if (impulse == Fixed64.Zero || !CanRotate)
            return;

        Wake();
        _angularVelocity += impulse * EffectiveInverseMomentOfInertia;
        RefreshAngularSpeed();
    }

    public void SetPosition(Vector2d position)
    {
        if (_position != position)
            Wake();

        _position = position;
        Collider.Rebuild();
    }

    public void SetRotation(Fixed64 rotation)
    {
        if (_rotation != rotation)
            Wake();

        _rotation = rotation;
        Collider.Rebuild();
    }

    private void RefreshPartitionMobility()
    {
        if (!Active)
            return;

        Context.Collisions2D.RefreshColliderPartition(Collider);
        if (Context.Settings.RuntimeMode.RunsMixedContacts())
            Context.MixedCollisions.Refresh2DColliderPartition(Collider);
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector2d velocityDelta)
    {
        if (!CanTranslate || velocityDelta == Vector2d.Zero)
            return;

        WakeFromCollision();
        _linearVelocity += velocityDelta;
        RefreshLinearSpeed();
    }

    internal void ApplyCollisionAngularVelocityDelta(Fixed64 velocityDelta)
    {
        if (!CanRotate || velocityDelta == Fixed64.Zero)
            return;

        WakeFromCollision();
        _angularVelocity += velocityDelta;
        RefreshAngularSpeed();
    }

    internal void ApplyCollisionPositionCorrection(Vector2d positionCorrection)
    {
        if (!CanTranslate || positionCorrection == Vector2d.Zero)
            return;

        _position += positionCorrection;
        Collider.Rebuild();
    }

    private bool CanSleep => SleepEnabled && CanTranslate;

    private void UpdateSleepState()
    {
        if (!CanSleep)
        {
            _sleepFrameCount = 0;
            return;
        }

        if (_linearSpeed > SleepLinearSpeedThreshold || _angularSpeed > _sleepAngularSpeedThreshold)
        {
            _sleepFrameCount = 0;
            return;
        }

        if (_sleepFrameCount < SleepFrameThreshold)
            _sleepFrameCount++;

        if (_sleepFrameCount >= SleepFrameThreshold)
            Sleep();
    }

    private void RefreshLinearSpeed()
    {
        _linearSpeed = _linearVelocity.Magnitude;
        if (_linearSpeed <= Fixed64.Epsilon)
        {
            _linearVelocity = Vector2d.Zero;
            _linearSpeed = Fixed64.Zero;
        }
    }

    private void RefreshAngularSpeed()
    {
        _angularSpeed = _angularVelocity.Abs();
        if (_angularSpeed <= Fixed64.Epsilon)
        {
            _angularVelocity = Fixed64.Zero;
            _angularSpeed = Fixed64.Zero;
        }
    }

    internal void RefreshMassPropertiesFromColliderShape()
    {
        if (!_centerOfMassOffsetExplicit)
            _localCenterOfMassOffset = Collider.CalculateLocalCenterOfMassOffset();

        if (_mass <= Fixed64.Zero)
        {
            _momentOfInertia = Fixed64.Zero;
            _inverseMomentOfInertia = Fixed64.Zero;
            return;
        }

        _momentOfInertia = Collider.CalculateMomentOfInertia(_mass, _localCenterOfMassOffset);
        _inverseMomentOfInertia = _momentOfInertia > Fixed64.Zero
            ? Fixed64.One / _momentOfInertia
            : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ClampNearZero(Vector2d value)
    {
        Fixed64 x = value.X.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.X;
        Fixed64 y = value.Y.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.Y;
        return new Vector2d(x, y);
    }
}
