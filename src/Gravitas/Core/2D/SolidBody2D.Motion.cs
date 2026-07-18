//=======================================================================
// SolidBody2D.Motion.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    /// <summary>
    /// Queues a planar force in mass-distance-per-time-squared units for
    /// integration during the next fixed step.
    /// </summary>
    /// <param name="force">The X/Z-plane force to apply.</param>
    public void AddForce(Vector2d force)
    {
        Vector2d accelerationDelta = ProjectLinearMotion(force * EffectiveInverseMass);
        if (accelerationDelta == Vector2d.Zero)
            return;

        Wake();
        _deltaAcceleration += accelerationDelta;
    }

    /// <summary>
    /// Queues a yaw torque in mass-distance-squared-per-time-squared units for
    /// integration during the next fixed step.
    /// </summary>
    /// <param name="torque">The signed yaw torque applied to the body.</param>
    public void AddTorque(Fixed64 torque)
    {
        Fixed64 accelerationDelta = torque * EffectiveInverseMomentOfInertia;
        if (accelerationDelta == Fixed64.Zero)
            return;

        Wake();
        _deltaAngularAcceleration += accelerationDelta;
    }

    /// <summary>
    /// Applies an X/Z-plane linear impulse immediately as a velocity change.
    /// The impulse is expressed in mass-distance-per-time units and does not
    /// advance the fixed-step simulation or apply a time-step factor.
    /// </summary>
    /// <param name="impulse">The planar linear impulse to apply.</param>
    public void AddLinearImpulse(Vector2d impulse)
    {
        Vector2d velocityDelta = ProjectLinearMotion(impulse * EffectiveInverseMass);
        if (velocityDelta == Vector2d.Zero)
            return;

        Wake();
        _linearVelocity += velocityDelta;
        RefreshLinearSpeed();
    }

    /// <summary>
    /// Applies an immediate yaw-velocity change derived from the supplied
    /// angular impulse and inverse moment of inertia. The impulse is expressed
    /// in mass-distance-squared-per-time units and does not apply a time-step
    /// factor.
    /// </summary>
    /// <param name="impulse">The signed yaw impulse applied to the body.</param>
    public void AddAngularImpulse(Fixed64 impulse)
    {
        Fixed64 velocityDelta = impulse * EffectiveInverseMomentOfInertia;
        if (velocityDelta == Fixed64.Zero)
            return;

        Wake();
        _angularVelocity += velocityDelta;
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
        Fixed64 canonicalRotation = CanonicalizeRotation(rotation);
        if (_rotation != canonicalRotation)
            Wake();

        _rotation = canonicalRotation;
        Collider.Rebuild();
    }

    private void RefreshPartitionMobility()
    {
        if (!Active)
            return;

        if (Collider.IsPartitioned)
            Collider.Simulate();

        Context.Collisions2D.RefreshColliderPartition(Collider);
        if (Context.Settings.RuntimeMode.RunsMixedContacts())
            Context.MixedCollisions.Refresh2DColliderPartition(Collider);
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector2d velocityDelta)
    {
        velocityDelta = ProjectLinearMotion(velocityDelta);
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
        positionCorrection = ProjectLinearMotion(positionCorrection);
        if (!CanTranslate || positionCorrection == Vector2d.Zero)
            return;

        _position += positionCorrection;
        Collider.Rebuild();
    }

    private bool CanSleep => SleepEnabled && CanTranslate;

    private void ApplyFreezeConstraintsToMotion()
    {
        _linearVelocity = ProjectLinearMotion(_linearVelocity);
        _linearAccelerationStore = ProjectLinearMotion(_linearAccelerationStore);
        _deltaAcceleration = ProjectLinearMotion(_deltaAcceleration);
        RefreshLinearSpeed();

        if (!CanRotate)
        {
            _angularVelocity = Fixed64.Zero;
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
            _angularSpeed = Fixed64.Zero;
        }
    }

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
