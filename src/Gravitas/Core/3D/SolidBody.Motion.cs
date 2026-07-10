//=======================================================================
// SolidBody.Motion.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    private bool CanUseAngularInertia => !IsPositionFullyFrozen && !IsKinematic && !IsRotationFullyFrozen;

    private void UpdateSleepState()
    {
        if (!CanSleep)
        {
            _sleepFrameCount = 0;
            return;
        }

        if (_linearSpeed > SleepLinearSpeedThreshold || _angularSpeed > SleepAngularSpeedThreshold)
        {
            _sleepFrameCount = 0;
            return;
        }

        if (_sleepFrameCount < _sleepFrameThreshold)
            _sleepFrameCount++;

        if (_sleepFrameCount >= _sleepFrameThreshold)
            Sleep();
    }

    private void ClearMotionForSleep()
    {
        _linearVelocity = Vector3d.Zero;
        _linearDirection = Vector3d.Zero;
        _linearAccelerationStore = Vector3d.Zero;
        _deltaAcceleration = Vector3d.Zero;
        _linearAcceleration = Vector3d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularVelocity = Vector3d.Zero;
        _angularDirection = Vector3d.Zero;
        _deltaTorque = Vector3d.Zero;
        _angularAccelerationStore = Vector3d.Zero;
        _angularAcceleration = Vector3d.Zero;
        _angularSpeed = Fixed64.Zero;
        _impulseStore = Vector3d.Zero;
        _positionCorrection = Vector2d.Zero;
        _timeScaledAcceleration = Vector3d.Zero;
        _timeScaledDeceleration = Vector3d.Zero;
    }

    private void ApplyFreezeConstraintsToMotion()
    {
        Vector3d lastLinearVelocity = _linearVelocity;
        _linearVelocity = ProjectLinearMotion(_linearVelocity);
        _linearAccelerationStore = ProjectLinearMotion(_linearAccelerationStore);
        _deltaAcceleration = ProjectLinearMotion(_deltaAcceleration);
        _linearAcceleration = ProjectLinearMotion(_linearAcceleration);
        _impulseStore = ProjectLinearMotion(_impulseStore);
        _timeScaledAcceleration = ProjectLinearMotion(_timeScaledAcceleration);
        _timeScaledDeceleration = ProjectLinearMotion(_timeScaledDeceleration);
        _positionCorrection = ProjectLinearMotion(_positionCorrection.ToVector3d(Fixed64.Zero)).ToVector2d();
        RefreshLinearMotionState(lastLinearVelocity);

        Vector3d lastAngularVelocity = _angularVelocity;
        _angularVelocity = ProjectAngularMotion(_angularVelocity);
        _deltaTorque = ProjectAngularMotion(_deltaTorque);
        _angularAccelerationStore = ProjectAngularMotion(_angularAccelerationStore);
        _angularAcceleration = ProjectAngularMotion(_angularAcceleration);
        RefreshAngularMotionState(lastAngularVelocity);
    }

    private void RefreshPartitionAwakeState()
    {
        if (Collider is { IsPartitioned: true })
            Context.Collisions.RefreshPartitionAwakeState(Collider);
    }

    private void RefreshPartitionMobility()
    {
        if (!Active || !Collider.TryGetBoundContext(out GravitasWorldContext? context))
            return;

        if (Collider.IsPartitioned)
            Collider.Simulate();

        if (context!.Settings.RuntimeMode.RunsMixedContacts())
            context.MixedCollisions.Refresh3DColliderPartition(Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void UpdateTimeScaledAcceleration()
    {
        PhysicsEnvironment environment = Context.Environment;
        _timeScaledAcceleration = _linearSpeed > Fixed64.Zero
            ? _linearAcceleration * _linearSpeed / Context.FrameRate
            : Vector3d.Zero;
        _timeScaledDeceleration = _timeScaledAcceleration != Vector3d.Zero
            ? _timeScaledAcceleration * environment.DecelerationMultiplier
            : Vector3d.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddTorque(Vector3d torque)
    {
        Vector3d accelerationDelta = ProjectAngularMotion(torque * _inverseInertiaTensor);
        if (accelerationDelta == Vector3d.Zero)
            return;

        Wake();
        _deltaTorque += accelerationDelta;
        Context.Diagnostics.EmitTorqueDelta(this, torque);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddForce(Vector3d force)
    {
        Vector3d accelerationDelta = ProjectLinearMotion(force * InverseMass);
        if (accelerationDelta == Vector3d.Zero)
            return;

        Wake();
        _deltaAcceleration += accelerationDelta;
        Context.Diagnostics.EmitForceDelta(this, force, accelerationDelta);
    }

    private Vector3d _impulseStore = Vector3d.Zero;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddLinearImpulse(Vector3d impulse)
    {
        Vector3d velocityDelta = ProjectLinearMotion((impulse * InverseMass) * Context.DeltaTime);
        if (velocityDelta == Vector3d.Zero)
            return;

        Wake();
        _impulseStore += velocityDelta;
        // testing immediate reaction for collisions...
        UpdateLinearVelocity();
        NonKinematicUpdate();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddAngularImpulse(Vector3d impulse)
    {
        Vector3d velocityDelta = ProjectAngularMotion((impulse * _inverseInertiaTensor) * Context.DeltaTime);
        if (velocityDelta == Vector3d.Zero)
            return;

        Wake();
        _angularVelocity += velocityDelta;
        RefreshAngularMotionState(_angularVelocity - velocityDelta);
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector3d velocityDelta)
    {
        velocityDelta = ProjectLinearMotion(velocityDelta);
        if (!CanTranslate || IsKinematic || velocityDelta == Vector3d.Zero)
            return;

        WakeFromCollision();
        Vector3d lastVelocity = _linearVelocity;
        _linearVelocity += velocityDelta;
        RefreshLinearMotionState(lastVelocity);
        Context.Diagnostics.EmitLinearVelocityDelta(this, lastVelocity, _linearVelocity);
    }

    internal void ApplyCollisionAngularVelocityDelta(Vector3d velocityDelta)
    {
        velocityDelta = ProjectAngularMotion(velocityDelta);
        if (!CanRotate || IsKinematic || velocityDelta == Vector3d.Zero)
            return;

        WakeFromCollision();
        Vector3d lastVelocity = _angularVelocity;
        _angularVelocity += velocityDelta;
        RefreshAngularMotionState(lastVelocity);
        Context.Diagnostics.EmitAngularVelocityDelta(this, lastVelocity, _angularVelocity);
    }

    internal void ApplyCollisionPositionCorrection(Vector3d positionCorrection)
    {
        positionCorrection = ProjectLinearMotion(positionCorrection);
        if (!CanTranslate || IsKinematic || positionCorrection == Vector3d.Zero)
            return;

        Position3d += positionCorrection;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPosition(Vector3d position)
    {
        if (Position3d != position)
            Wake();

        Position3d = position;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddPositionCorrection(Vector3d positionCorrection) =>
        _positionCorrection += ProjectLinearMotion(positionCorrection).ToVector2d();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetHeight(Fixed64 height)
    {
        if (HeightPos != height)
            Wake();

        HeightPos = height;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetRotation(FixedQuaternion quaternion)
    {
        if (Rotation != quaternion)
            Wake();

        Rotation = quaternion;
    }

    private void ProcessMovable()
    {
        ApplyLinearForces();
        UpdateLinearVelocity();

        if (CanRotate)
        {
            ApplyAngularTorques();
            UpdateAngularVelocity();
        }

        // Non-kinematic bodies position is calculated based on current velocity
        NonKinematicUpdate();

        if (_linearSpeed > Fixed64.Zero)
            UpdateTimeScaledAcceleration();
    }

    private void ApplyLinearForces()
    {
        _linearAccelerationStore = ProjectLinearMotion(_deltaAcceleration);
        _deltaAcceleration = Vector3d.Zero;

        if (_linearSpeed <= Fixed64.Zero)
            return;

        ApplyDragForce();
        ApplyFrictionForce();
    }

    private void ApplyDragForce()
    {
        // Drag calculation and accumulation.
        Fixed64 dragMagnitude = LinearDragCoefficient * Context.Environment.AirDensity * Collider.GetFrontalArea(_linearDirection) * _linearSpeed;
        _linearAccelerationStore += (-_linearDirection * dragMagnitude);
    }

    private void ApplyFrictionForce()
    {
        if (!_isGrounded)
            return;

        Vector2d horizontalVelocity = new(_linearVelocity.X, _linearVelocity.Z);
        Fixed64 horizontalSpeed = horizontalVelocity.Magnitude;
        if (horizontalSpeed <= Fixed64.Zero)
            return;

        // Object is moving on ground, add the friction force to the accumulated force
        // Adjust the friction with the normal force magnitude
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 effectiveFriction = ResolveGroundDynamicFriction();
        if (horizontalSpeed <= environment.FrictionTransitionSpeed)
        {
            Fixed64 proportion = horizontalSpeed / environment.FrictionTransitionSpeed;
            effectiveFriction *= proportion;
        }

        Fixed64 frictionMagnitude = effectiveFriction * _normalForce.Magnitude;
        _linearAccelerationStore += (-_linearDirection * frictionMagnitude) * InverseMass;
    }

    private void UpdateLinearVelocity()
    {
        Fixed64 deltaTime = Context.DeltaTime;
        PhysicsEnvironment environment = Context.Environment;
        Vector3d lastVelocity = _linearVelocity;

        _linearVelocity += ProjectLinearMotion(_impulseStore + (_linearAccelerationStore * deltaTime));
        // LinearVelocity = _impulseStore + (_linearAccelerationStore * deltaTime);

        // Reset stores for the next frame
        _linearAccelerationStore = Vector3d.Zero;
        _impulseStore = Vector3d.Zero;

        // Apply gravity only if not grounded
        if (!IsGrounded && (_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
            _linearVelocity.Y -= environment.Gravity * _gravityScale * deltaTime;

        // Make sure we don't fall any faster than maxFallSpeed. This gives our character a terminal velocity
        _linearVelocity.Y = FixedMath.Max(_linearVelocity.Y, -environment.MaxFallSpeed);
        _linearVelocity = ProjectLinearMotion(_linearVelocity);

        RefreshLinearMotionState(lastVelocity);
    }

    private void RefreshLinearMotionState(Vector3d lastVelocity)
    {
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 desiredSpeed = _linearVelocity.Magnitude;
        if (desiredSpeed > environment.MinSpeed)
        {
            if (desiredSpeed > environment.MaxSpeed)
            {
                _linearVelocity = _linearVelocity.Normalized * environment.MaxSpeed;
                _linearSpeed = environment.MaxSpeed;
            }
            else
                _linearSpeed = desiredSpeed;
        }
        else
        {
            _linearVelocity = Vector3d.Zero;
            _linearSpeed = Fixed64.Zero;
        }

        // Update the direction of the linear velocity, if we're not moving maintain previous direction
        _linearDirection = _linearSpeed > Fixed64.Zero
            ? _linearVelocity.Normalized
            : _linearDirection;
        _linearAcceleration = _linearSpeed > Fixed64.Zero
            ? (_linearVelocity - lastVelocity) / Context.DeltaTime
            : Vector3d.Zero;
    }

    private void ApplyAngularTorques()
    {
        _angularAccelerationStore = ProjectAngularMotion(_deltaTorque);
        _deltaTorque = Vector3d.Zero;

        if (_angularSpeed <= Fixed64.Zero)
            return;

        ApplyDragTorque();
        ApplyFrictionTorque();
    }

    private void ApplyDragTorque()
    {
        // Angular drag should also be proportional to the square of the angular velocity, just like linear drag.
        Fixed64 angularDragMagnitude = AngularDragCoefficient * Context.Environment.AirDensity * Collider.GetFrontalArea(_angularDirection) * _angularSpeed;
        _angularAccelerationStore += (-_angularDirection * angularDragMagnitude);
    }

    private void ApplyFrictionTorque()
    {
        if (!IsGrounded)
            return;

        // Calculate the friction force and convert it into a torque
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 effectiveFriction = ResolveGroundDynamicFriction();
        if (_angularSpeed < environment.FrictionTransitionSpeed)
        {
            Fixed64 proportion = _angularSpeed / environment.FrictionTransitionSpeed;
            effectiveFriction *= proportion;
        }

        Fixed64 frictionMagnitude = effectiveFriction * _normalForce.Magnitude;
        _linearAccelerationStore += (-_angularDirection * frictionMagnitude) * _inverseInertiaTensor;
    }

    private void UpdateAngularVelocity()
    {
        Fixed64 deltaTime = Context.DeltaTime;
        PhysicsEnvironment environment = Context.Environment;
        Vector3d lastVelocity = _angularVelocity;
        // Apply the accumulated angular acceleration
        _angularVelocity += ProjectAngularMotion(_angularAccelerationStore * deltaTime);
        // Reset the acceleration store for the next frame
        _angularAccelerationStore = Vector3d.Zero;

        // Add damping torque, proportional to negative angular velocity
        Vector3d dampingTorque = -environment.DampingFactor * _angularVelocity;
        _angularVelocity += ProjectAngularMotion(_inverseInertiaTensor * dampingTorque * deltaTime);
        _angularVelocity = ProjectAngularMotion(_angularVelocity);

        RefreshAngularMotionState(lastVelocity);
    }

    private void RefreshAngularMotionState(Vector3d lastVelocity)
    {
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 desiredSpeed = _angularVelocity.Magnitude;
        if (desiredSpeed > environment.MinSpeed)
        {
            if (desiredSpeed > environment.MaxSpeed)
            {
                _angularVelocity = _angularVelocity.Normalized * environment.MaxSpeed;
                _angularSpeed = environment.MaxSpeed;

            }
            else
            {
                _angularSpeed = desiredSpeed;
                _angularDirection = _angularSpeed > Fixed64.Zero ? _angularVelocity.Normalized : Vector3d.Zero;
            }
        }
        else
        {
            _angularVelocity = Vector3d.Zero;
            _angularSpeed = Fixed64.Zero;
        }

        _angularAcceleration = _angularSpeed > Fixed64.Zero
            ? (_angularVelocity - lastVelocity) / Context.DeltaTime
            : Vector3d.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveGroundDynamicFriction() =>
        Collider.Material.DynamicFriction;

    private void NonKinematicUpdate()
    {
        if (IsKinematic)
            return;

        //Position2d += _positionCorrection;
        //_positionCorrection = Vector2d.Zero;

        //// if (_linearSpeed > Fixed64.Zero)
        Vector3d rotationalCcdStartPosition = Position3d;
        PositionBasedOnForce();
        Vector3d rotationalCcdProposedPosition = Position3d;
        if (CanRotate && _angularSpeed > Fixed64.Zero)
            RotationBasedOnTorque(rotationalCcdStartPosition, rotationalCcdProposedPosition);

        CheckGroundForSimulation();

        if (_isGrounded)
            HeightPos = HitPoint.Y;
        else
            ResetGroundCalculations();

        CheckChangedValues();
        if (!CanRotate && !RotationChangePending)
            return;

        UpdateInertiaTensorOrientation();
        ApplyGyroscopicPrecession();
    }

    private void PositionBasedOnForce()
    {
        Vector3d startPosition = Position3d;
        Vector3d velocityVector = startPosition + (_linearVelocity * Context.DeltaTime);
        TryResolveContinuousCollision(startPosition, ref velocityVector);
        velocityVector = startPosition + ProjectLinearMotion(velocityVector - startPosition);

        Vector2d velocityAxis = velocityVector.ToVector2d();

        // Find out how much we need to push towards the ground to avoid loosing grounding
        // when walking down a step or over a sharp change in slope.
        if (_isGrounded && (_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
        {
            velocityVector.Y -= FixedMath.Max(StepOffset, velocityVector.Magnitude);
            _lastGroundedPosition = Position3d;
        }
        else if ((_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
            HeightPos = velocityVector.Y;

        //  Apply the force
        Position2d = _positionCorrection + velocityAxis;
        _positionCorrection = Vector2d.Zero;
    }


    private void RotationBasedOnTorque(Vector3d startPosition, Vector3d proposedPosition)
    {
        FixedQuaternion startRotation = Rotation;
        FixedQuaternion proposedRotation = IntegrateAngularRotation(startRotation, Context.DeltaTime);
        TryResolveRotationalContinuousCollision(startPosition, ref proposedPosition, startRotation, ref proposedRotation);
        Position3d = proposedPosition;
        Rotation = proposedRotation;
        Collider.RebuildRuntimeShapeOnly();
    }

    internal void RefreshMassPropertiesFromColliderShape()
    {
        if (!_centerOfMassOffsetExplicit)
            _localCenterOfMassOffset = Collider.CalculateLocalCenterOfMassOffset();

        RefreshInertiaTensor();
    }

    private void UpdateInertiaTensorOrientation()
    {
        if (_inertiaTensor == Fixed3x3.Zero || _inverseLocalInertiaTensor == Fixed3x3.Zero)
        {
            _worldInertiaTensor = Fixed3x3.Zero;
            _inverseInertiaTensor = Fixed3x3.Zero;
            return;
        }

        Fixed3x3 inverseOrientation = Rotation.Conjugate().ToMatrix3x3();
        Fixed3x3 orientation = Rotation.ToMatrix3x3();

        _worldInertiaTensor = orientation * _inertiaTensor * inverseOrientation;
        _inverseInertiaTensor = orientation * _inverseLocalInertiaTensor * inverseOrientation;
    }

    private void RefreshInertiaTensor()
    {
        if (!CanUseAngularInertia)
        {
            _inertiaTensor = Fixed3x3.Zero;
            _worldInertiaTensor = Fixed3x3.Zero;
            _inverseLocalInertiaTensor = Fixed3x3.Zero;
            _inverseInertiaTensor = Fixed3x3.Zero;
            return;
        }

        _inertiaTensor = Collider.CalculateInertiaTensor(Mass, _localCenterOfMassOffset);
        _inverseLocalInertiaTensor = InertiaTensorMath.InvertForSolver(_inertiaTensor);
        UpdateInertiaTensorOrientation();
    }

    //  gyroscopic precession is a correction to the object's angular velocity based on its rotation
    private void ApplyGyroscopicPrecession()
    {
        if (!CanRotate || _worldInertiaTensor == Fixed3x3.Zero || _inverseInertiaTensor == Fixed3x3.Zero)
            return;

        _angularVelocity += ProjectAngularMotion(_inverseInertiaTensor * Vector3d.Cross(_angularVelocity, _worldInertiaTensor * _angularVelocity) * Context.DeltaTime);
        _angularVelocity = ProjectAngularMotion(_angularVelocity);
    }

}
