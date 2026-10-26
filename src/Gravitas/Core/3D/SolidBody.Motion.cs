//=======================================================================
// SolidBody.Motion.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector3d ProjectLinearMotion(Vector3d value)
    {
        if (value == Vector3d.Zero || IsPositionFullyFrozen)
            return Vector3d.Zero;

        Fixed64 x = (_freezeAxes & BodyFreezeAxes3D.PositionX) == BodyFreezeAxes3D.PositionX ? Fixed64.Zero : value.X;
        Fixed64 y = (_freezeAxes & BodyFreezeAxes3D.PositionY) == BodyFreezeAxes3D.PositionY ? Fixed64.Zero : value.Y;
        Fixed64 z = (_freezeAxes & BodyFreezeAxes3D.PositionZ) == BodyFreezeAxes3D.PositionZ ? Fixed64.Zero : value.Z;
        return new Vector3d(x, y, z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector3d ProjectLinearEndpoint(Vector3d start, Vector3d end)
    {
        Fixed64 x = (_freezeAxes & BodyFreezeAxes3D.PositionX) == BodyFreezeAxes3D.PositionX ? start.X : end.X;
        Fixed64 y = (_freezeAxes & BodyFreezeAxes3D.PositionY) == BodyFreezeAxes3D.PositionY ? start.Y : end.Y;
        Fixed64 z = (_freezeAxes & BodyFreezeAxes3D.PositionZ) == BodyFreezeAxes3D.PositionZ ? start.Z : end.Z;
        return new Vector3d(x, y, z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector3d ProjectAngularMotion(Vector3d value)
    {
        if (value == Vector3d.Zero || IsRotationFullyFrozen)
            return Vector3d.Zero;

        Fixed64 x = (_freezeAxes & BodyFreezeAxes3D.RotationX) == BodyFreezeAxes3D.RotationX ? Fixed64.Zero : value.X;
        Fixed64 y = (_freezeAxes & BodyFreezeAxes3D.RotationY) == BodyFreezeAxes3D.RotationY ? Fixed64.Zero : value.Y;
        Fixed64 z = (_freezeAxes & BodyFreezeAxes3D.RotationZ) == BodyFreezeAxes3D.RotationZ ? Fixed64.Zero : value.Z;
        return new Vector3d(x, y, z);
    }

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
    }

    private void ApplyFreezeConstraintsToMotion()
    {
        Vector3d lastLinearVelocity = _linearVelocity;
        _linearVelocity = ProjectLinearMotion(_linearVelocity);
        _linearAccelerationStore = ProjectLinearMotion(_linearAccelerationStore);
        _deltaAcceleration = ProjectLinearMotion(_deltaAcceleration);
        _linearAcceleration = ProjectLinearMotion(_linearAcceleration);
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
        if (Collider.IsPartitioned)
            Context.Collisions.RefreshPartitionAwakeState(Collider);
        Context.MixedCollisions.Refresh3DPartitionAwakeState(Collider);
    }

    private void RefreshPartitionMobility()
    {
        if (!Active || !Collider.TryGetBoundContext(out GravitasWorldContext? context))
            return;

        Collider.Simulate();

        if (context!.Settings.RuntimeMode.RunsMixedContacts())
            context.MixedCollisions.Refresh3DColliderPartition(Collider);
    }

    /// <summary>
    /// Queues a torque in mass-distance-squared-per-time-squared units for
    /// integration during the next fixed step.
    /// </summary>
    /// <param name="torque">The world-space torque to apply.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddTorque(Vector3d torque)
    {
        Vector3d accelerationDelta = ProjectAngularMotion(torque * EffectiveInverseInertiaTensor);
        if (accelerationDelta == Vector3d.Zero)
            return;

        Wake();
        _deltaTorque += accelerationDelta;
        Context.Diagnostics.EmitTorqueDelta(this, torque);
    }

    /// <summary>
    /// Queues a force in mass-distance-per-time-squared units for integration
    /// during the next fixed step.
    /// </summary>
    /// <param name="force">The world-space force to apply.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddForce(Vector3d force)
    {
        Vector3d accelerationDelta = ProjectLinearMotion(force * EffectiveInverseMass);
        if (accelerationDelta == Vector3d.Zero)
            return;

        Wake();
        _deltaAcceleration += accelerationDelta;
        Context.Diagnostics.EmitForceDelta(this, force, accelerationDelta);
    }

    /// <summary>
    /// Applies a world-space linear impulse immediately as a velocity change.
    /// The impulse is expressed in mass-distance-per-time units and does not
    /// advance the fixed-step simulation or apply a time-step factor.
    /// </summary>
    /// <param name="impulse">The world-space linear impulse to apply.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddLinearImpulse(Vector3d impulse)
    {
        Vector3d velocityDelta = ProjectLinearMotion(impulse * EffectiveInverseMass);
        if (velocityDelta == Vector3d.Zero)
            return;

        Wake();
        Vector3d lastVelocity = _linearVelocity;
        _linearVelocity += velocityDelta;
        RefreshLinearMotionState(lastVelocity);
    }

    /// <summary>
    /// Applies a world-space angular impulse immediately as an angular-velocity
    /// change. The impulse is expressed in mass-distance-squared-per-time units
    /// and does not apply a time-step factor.
    /// </summary>
    /// <param name="impulse">The world-space angular impulse to apply.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddAngularImpulse(Vector3d impulse)
    {
        Vector3d velocityDelta = ProjectAngularMotion(impulse * EffectiveInverseInertiaTensor);
        if (velocityDelta == Vector3d.Zero)
            return;

        Wake();
        _angularVelocity += velocityDelta;
        RefreshAngularMotionState(_angularVelocity - velocityDelta);
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector3d velocityDelta)
    {
        velocityDelta = ProjectLinearMotion(velocityDelta);
        if (!CanTranslate || velocityDelta == Vector3d.Zero)
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
        if (!CanRotate || velocityDelta == Vector3d.Zero)
            return;

        WakeFromCollision();
        Vector3d lastVelocity = _angularVelocity;
        _angularVelocity += velocityDelta;
        RefreshAngularMotionState(lastVelocity);
        Context.Diagnostics.EmitAngularVelocityDelta(this, lastVelocity, _angularVelocity);
    }

    internal bool CanApplyCollisionVelocityDeltas(
        Vector3d linearVelocityDelta,
        Vector3d angularVelocityDelta) =>
        TryPrepareCollisionVelocityState(
            linearVelocityDelta,
            angularVelocityDelta,
            out _,
            out _);

    internal bool TryPrepareCollisionVelocityState(
        Vector3d linearVelocityDelta,
        Vector3d angularVelocityDelta,
        out Vector3d linearVelocity,
        out Vector3d angularVelocity)
    {
        linearVelocityDelta = ProjectLinearMotion(linearVelocityDelta);
        angularVelocityDelta = ProjectAngularMotion(angularVelocityDelta);
        linearVelocity = _linearVelocity;
        angularVelocity = _angularVelocity;
        bool linearVelocityFits = !CanTranslate
            || Vector3d.TryAdd(
                _linearVelocity,
                linearVelocityDelta,
                out linearVelocity);
        bool angularVelocityFits = !CanRotate
            || Vector3d.TryAdd(
                _angularVelocity,
                angularVelocityDelta,
                out angularVelocity);
        return linearVelocityFits & angularVelocityFits;
    }

    internal void ApplyCollisionVelocityState(
        Vector3d linearVelocity,
        Vector3d angularVelocity)
    {
        linearVelocity = ProjectLinearMotion(linearVelocity);
        angularVelocity = ProjectAngularMotion(angularVelocity);
        if (_linearVelocity == linearVelocity && _angularVelocity == angularVelocity)
            return;

        WakeFromCollision();
        if (_linearVelocity != linearVelocity)
        {
            Vector3d lastLinearVelocity = _linearVelocity;
            _linearVelocity = linearVelocity;
            RefreshLinearMotionState(lastLinearVelocity);
            Context.Diagnostics.EmitLinearVelocityDelta(this, lastLinearVelocity, _linearVelocity);
        }

        if (_angularVelocity != angularVelocity)
        {
            Vector3d lastAngularVelocity = _angularVelocity;
            _angularVelocity = angularVelocity;
            RefreshAngularMotionState(lastAngularVelocity);
            Context.Diagnostics.EmitAngularVelocityDelta(this, lastAngularVelocity, _angularVelocity);
        }
    }

    internal Vector3d PrepareCollisionPositionCorrection(Vector3d positionCorrection) =>
        CanTranslate
            ? AddPositionDisplacement(Position3d, ProjectLinearMotion(positionCorrection))
            : Position3d;

    internal void ApplyCollisionPositionState(Vector3d position)
    {
        if (Position3d == position)
            return;
        Position3d = position;
    }

    // Callers project frozen axes first; numeric exhaustion is not a physical wall.
    private static Vector3d AddPositionDisplacement(Vector3d start, Vector3d displacement)
    {
        if (!Vector3d.TryAdd(start, displacement, out Vector3d position))
            throw new OverflowException("The requested body position is outside the representable coordinate domain.");
        return position;
    }


    /// <summary>Sets the authoritative world position and refreshes collider state.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPosition(Vector3d position)
    {
        PreflightStaticPoseChange();
        bool preparedPose = PrepareExplicitBodyPose(
            position,
            Rotation,
            nameof(position),
            "The requested body position produces collider geometry outside the representable coordinate domain.");
        bool poseChanged = Position3d != position;
        if (poseChanged)
            Wake();

        Position3d = position;
        PublishExplicitBodyPose(preparedPose, poseChanged);
    }

    /// <summary>Sets the authoritative world-space Y coordinate.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetHeight(Fixed64 height)
    {
        PreflightStaticPoseChange();
        Vector3d position = Position3d;
        position.Y = height;
        bool preparedPose = PrepareExplicitBodyPose(
            position,
            Rotation,
            nameof(height),
            "The requested body height produces collider geometry outside the representable coordinate domain.");
        bool poseChanged = HeightPos != height;
        if (poseChanged)
            Wake();

        HeightPos = height;
        PublishExplicitBodyPose(preparedPose, poseChanged);
    }

    /// <summary>Sets the authoritative world rotation and refreshes collider state.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetRotation(FixedQuaternion quaternion)
    {
        FixedQuaternion normalizedRotation = quaternion.Normalized;
        PreflightStaticPoseChange();
        bool preparedPose = PrepareExplicitBodyPose(
            Position3d,
            normalizedRotation,
            nameof(quaternion),
            "The requested body rotation produces collider geometry outside the representable coordinate domain.");
        bool poseChanged = Rotation != normalizedRotation;
        if (poseChanged)
            Wake();

        Rotation = normalizedRotation;
        PublishExplicitBodyPose(preparedPose, poseChanged);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool PrepareExplicitBodyPose(
        Vector3d position,
        FixedQuaternion rotation,
        string parameterName,
        string message)
    {
        if (!Active)
            return false;

        ChronicleGuard.ThrowIfTrue(
            !Collider.TryPrepareBodyPose(position, rotation),
            parameterName,
            message);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PublishExplicitBodyPose(bool preparedPose, bool poseChanged)
    {
        if (!preparedPose)
            return;

        Collider.PublishPreparedExplicitBodyPose(poseChanged);
        RefreshPartitionMobility();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PreflightStaticPoseChange()
    {
        if (Active && IsStatic)
        {
            ThrowIfRuntimeRegistrationMissing();
            Context.ThrowIfFixedStepMutationNotAllowed();
            Collider.ValidateCurrentRuntimeTransform();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PreflightResetPoseChange()
    {
        if (!Active)
            return;

        ThrowIfRuntimeRegistrationMissing();
        if (!IsStatic)
            return;

        Context.ThrowIfFixedStepMutationNotAllowed();
        Collider.ValidateCurrentRuntimeTransform();
    }

    private void ProcessMovable()
    {
        EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

        // Non-kinematic bodies position is calculated from the motion prepared
        // for every body before ordered body processing begins.
        NonKinematicUpdate(_continuousCollisionAngularVelocityStepStart);

    }

    private void ApplyLinearForces()
    {
        _linearAccelerationStore = ProjectLinearMotion(_deltaAcceleration);
        _deltaAcceleration = Vector3d.Zero;
    }

    private Vector3d ApplyDrag(Vector3d velocity, Fixed64 coefficient, Vector3d previousVelocity, Vector3d previousDirection)
    {
        Fixed64 density = Context.Environment.AirDensity;
        if (coefficient == Fixed64.Zero || density == Fixed64.Zero || velocity == Vector3d.Zero)
            return velocity;

        Vector3d direction = velocity == previousVelocity ? previousDirection : velocity.Normalized;
        Fixed64 fraction = coefficient * density * Collider.GetFrontalArea(direction) * Context.DeltaTime;
        // The existing drag law is linear in speed. Bound its radial decay before it reverses motion.
        return fraction >= Fixed64.One ? Vector3d.Zero : velocity * (Fixed64.One - fraction);
    }

    private void ApplyGroundLinearFriction()
    {
        if (!HasCurrentGroundSupport || Collider.Material.DynamicFriction == Fixed64.Zero)
            return;

        // Project within the permitted axes so friction cannot consume separating normal motion.
        Vector3d direction = Vector3d.GetNormalizedProjectionOnPlane(
            _linearVelocity, ProjectLinearMotion(_groundNormal));
        if (direction == Vector3d.Zero)
            return;

        Fixed64 speed = Vector3d.Dot(_linearVelocity, direction);
        _linearVelocity = ApplyBoundedDissipation(
            _linearVelocity, direction, direction, ResolveGroundFrictionImpulse(speed) * InverseMass);
    }

    private void UpdateLinearVelocity()
    {
        Fixed64 deltaTime = Context.DeltaTime;
        PhysicsEnvironment environment = Context.Environment;
        Vector3d lastVelocity = _linearVelocity;

        _linearVelocity += ProjectLinearMotion(_linearAccelerationStore * deltaTime);
        _linearVelocity = ApplyDrag(_linearVelocity, LinearDragCoefficient, lastVelocity, _linearDirection);
        ApplyGroundLinearFriction();

        // Reset stores for the next frame
        _linearAccelerationStore = Vector3d.Zero;

        // Apply gravity only if not grounded
        if (!HasCurrentGroundSupport && (_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
            _linearVelocity.Y -= environment.Gravity * _gravityScale * deltaTime;

        // Make sure we don't fall any faster than maxFallSpeed. This gives our character a terminal velocity
        _linearVelocity.Y = FixedMath.Max(_linearVelocity.Y, -environment.MaxFallSpeed);
        _linearVelocity = ProjectLinearMotion(_linearVelocity);

        RefreshLinearMotionState(lastVelocity);
    }

    private void RefreshLinearMotionState(Vector3d lastVelocity)
    {
        _linearVelocity = ResolveLimitedVelocity(_linearVelocity, _linearVelocity.Magnitude, out _linearSpeed);

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
    }

    private Vector3d ApplyGroundAngularFriction(Vector3d velocity)
    {
        if (!HasCurrentGroundSupport || Collider.Material.DynamicFriction == Fixed64.Zero)
            return velocity;

        Fixed64 speed = velocity == _angularVelocity ? _angularSpeed : velocity.Magnitude;
        if (speed <= Fixed64.Zero)
            return velocity;

        Vector3d direction = velocity == _angularVelocity ? _angularDirection : velocity.Normalized;
        Fixed64 impulse = ResolveGroundFrictionImpulse(speed);
        return ApplyAngularDissipation(velocity, direction, impulse);
    }

    private Vector3d ApplyAngularDamping(Vector3d velocity)
    {
        Fixed64 damping = Context.Environment.DampingFactor;
        if (damping == Fixed64.Zero || velocity == Vector3d.Zero)
            return velocity;

        Vector3d direction = velocity;
        Fixed64 speed;
        if (velocity == _angularVelocity)
        {
            direction = _angularDirection;
            speed = _angularSpeed;
        }
        else
            direction.NormalizeInPlace(out speed);
        return ApplyAngularDissipation(velocity, direction, damping * speed * Context.DeltaTime);
    }

    private Vector3d ApplyAngularDissipation(Vector3d velocity, Vector3d direction, Fixed64 impulse)
    {
        Vector3d response = ProjectAngularMotion(_inverseInertiaTensor * direction);
        return (_freezeAxes & BodyFreezeAxes3D.Rotation) != BodyFreezeAxes3D.None
            ? ApplyConstrainedAngularDissipation(velocity, response, impulse)
            : ApplyBoundedDissipation(velocity, direction, response, impulse);
    }

    private Vector3d ApplyConstrainedAngularDissipation(Vector3d velocity, Vector3d response, Fixed64 impulse)
    {
        // Projection and rotated inertia do not commute. Keep M*r exact: supported
        // slender bodies can overflow both this intermediate and its energy dot products.
        Signed192 x = Signed192.Raw(response.X), y = Signed192.Raw(response.Y), z = Signed192.Raw(response.Z);
        Signed192 mx = Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
            Signed192.Raw(_worldInertiaTensor.M11), Signed192.Raw(_worldInertiaTensor.M12), Signed192.Raw(_worldInertiaTensor.M13), x, y, z));
        Signed192 my = Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
            Signed192.Raw(_worldInertiaTensor.M21), Signed192.Raw(_worldInertiaTensor.M22), Signed192.Raw(_worldInertiaTensor.M23), x, y, z));
        Signed192 mz = Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
            Signed192.Raw(_worldInertiaTensor.M31), Signed192.Raw(_worldInertiaTensor.M32), Signed192.Raw(_worldInertiaTensor.M33), x, y, z));
        Signed320 work = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(velocity.X), Signed192.Raw(velocity.Y), Signed192.Raw(velocity.Z), mx, my, mz);
        Signed320 curvature = WideArithmetic.GetDotProduct3D(x, y, z, mx, my, mz);
        if ((work.Sign <= 0) | (curvature.Sign <= 0))
            return velocity;

        // Two-factor sums fit 130 bits; energy products fit 196 and completed
        // velocity numerators fit 260. Existing fixed-width math owns all limbs/rounding.
        Signed576 budget = WideArithmetic.MultiplySigned320(curvature, Signed192.Raw(impulse));
        Signed576 minimum = WideArithmetic.MultiplySigned320(work, Signed192.One);
        if (WideArithmetic.SubtractSigned576(budget, minimum).Sign >= 0)
        {
            return new Vector3d(
                ResolveConstrainedDissipativeVelocity(velocity.X, response.X, work, curvature),
                ResolveConstrainedDissipativeVelocity(velocity.Y, response.Y, work, curvature),
                ResolveConstrainedDissipativeVelocity(velocity.Z, response.Z, work, curvature));
        }
        return new Vector3d(
            ResolveDissipativeVelocity(velocity.X, response.X, impulse, Fixed64.One),
            ResolveDissipativeVelocity(velocity.Y, response.Y, impulse, Fixed64.One),
            ResolveDissipativeVelocity(velocity.Z, response.Z, impulse, Fixed64.One));
    }

    private static Fixed64 ResolveConstrainedDissipativeVelocity(
        Fixed64 velocity, Fixed64 response, Signed320 work, Signed320 curvature)
    {
        Signed576 numerator = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(curvature, Signed192.Raw(velocity)),
            WideArithmetic.MultiplySigned320(work, Signed192.Raw(response)));
        if (!Fixed64.TryGetSignedRawRatio(numerator, Signed576.ExtendValue(curvature), out Fixed64 result))
            throw new OverflowException("Dissipation produces an unrepresentable velocity.");
        return result;
    }

    private static Vector3d ApplyBoundedDissipation(
        Vector3d velocity, Vector3d energyResponse, Vector3d response, Fixed64 impulse)
    {
        Fixed64 curvature = Vector3d.Dot(energyResponse, response);
        Fixed64 work = Vector3d.Dot(velocity, energyResponse);
        if ((curvature <= Fixed64.Zero) | (work <= Fixed64.Zero))
            return velocity;

        // For response r and energy metric M, energy is minimal at j = (v . M r) / (r . M r).
        // Retain the tensor response: anisotropic inertia need not stop every component at once.
        Fixed64 divisor = Fixed64.One;
        if (impulse * curvature >= work)
        {
            impulse = work;
            divisor = curvature;
        }
        return new Vector3d(
            ResolveDissipativeVelocity(velocity.X, response.X, impulse, divisor),
            ResolveDissipativeVelocity(velocity.Y, response.Y, impulse, divisor),
            ResolveDissipativeVelocity(velocity.Z, response.Z, impulse, divisor));
    }

    private static Fixed64 ResolveDissipativeVelocity(
        Fixed64 velocity, Fixed64 response, Fixed64 impulse, Fixed64 divisor)
    {
        Fixed64 result;
        if (divisor == Fixed64.One)
        {
            if (Fixed64.TrySubtractProducts(velocity, Fixed64.One, response, impulse, out result))
                return result;
        }
        else
        {
            // The correction may exceed Fixed64 even when the completed velocity fits.
            Signed192 numerator = WideArithmetic.SubtractSigned192(
                Fixed64.GetExactRawProduct(velocity, divisor), Fixed64.GetExactRawProduct(response, impulse));
            if (Fixed64.TryGetSignedRawRatio(numerator, Signed192.Raw(divisor), out result))
                return result;
        }
        throw new OverflowException("Dissipation produces an unrepresentable velocity.");
    }

    private void UpdateAngularVelocity()
    {
        Vector3d lastVelocity = _angularVelocity;
        _angularVelocity = ResolveAngularVelocityForStep(_angularVelocity, _angularAccelerationStore);
        _angularAccelerationStore = Vector3d.Zero;
        RefreshAngularMotionState(lastVelocity);
    }

    private Vector3d ResolveAngularVelocityForStep(
        Vector3d startVelocity,
        Vector3d angularAcceleration)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Vector3d velocity = startVelocity
            + ProjectAngularMotion(angularAcceleration * deltaTime);
        velocity = ApplyDrag(velocity, AngularDragCoefficient, startVelocity, _angularDirection);
        velocity = ApplyGroundAngularFriction(velocity);
        velocity = ApplyAngularDamping(velocity);
        velocity = ProjectAngularMotion(velocity);

        Fixed64 desiredSpeed = velocity.Magnitude;
        if (desiredSpeed <= Context.Environment.MinSpeed)
            return Vector3d.Zero;

        return desiredSpeed > Context.Environment.MaxSpeed
            ? velocity.Normalized * Context.Environment.MaxSpeed
            : velocity;
    }

    private void RefreshAngularMotionState(Vector3d lastVelocity)
    {
        Fixed64 desiredSpeed = _angularVelocity.Magnitude;
        _angularVelocity = ResolveLimitedVelocity(_angularVelocity, desiredSpeed, out _angularSpeed);
        if (desiredSpeed > Context.Environment.MinSpeed)
            _angularDirection = _angularVelocity.Normalized;

        _angularAcceleration = _angularSpeed > Fixed64.Zero
            ? (_angularVelocity - lastVelocity) / Context.DeltaTime
            : Vector3d.Zero;
    }

    private Vector3d ResolveLimitedVelocity(Vector3d velocity, Fixed64 desiredSpeed, out Fixed64 speed)
    {
        PhysicsEnvironment environment = Context.Environment;
        if (desiredSpeed <= environment.MinSpeed)
        {
            speed = Fixed64.Zero;
            return Vector3d.Zero;
        }
        if (desiredSpeed > environment.MaxSpeed)
        {
            speed = environment.MaxSpeed;
            return velocity.Normalized * speed;
        }
        speed = desiredSpeed;
        return velocity;
    }

    private Fixed64 ResolveGroundFrictionImpulse(Fixed64 speed)
    {
        Fixed64 friction = Collider.Material.DynamicFriction;
        Fixed64 transition = Context.Environment.FrictionTransitionSpeed;
        if (speed < transition)
            friction *= speed / transition;
        return friction * _normalForce.Magnitude * Context.DeltaTime;
    }

    private void NonKinematicUpdate(Vector3d angularVelocityStateStart)
    {
        Vector3d rotationalCcdStartPosition = Position3d;
        Vector3d rotationalCcdProposedPosition = AddPositionDisplacement(
            rotationalCcdStartPosition, ProjectLinearMotion(_linearVelocity) * Context.DeltaTime);
        FixedQuaternion rotationalCcdStartRotation = Rotation;
        FixedQuaternion rotationalCcdProposedRotation = IntegrateAngularRotation(
            rotationalCcdStartRotation,
            Context.DeltaTime);
        if (TryResolveRotationalContinuousCollision(
                rotationalCcdStartPosition,
                ref rotationalCcdProposedPosition,
                rotationalCcdStartRotation,
                ref rotationalCcdProposedRotation))
        {
            ApplyResolvedPosition(rotationalCcdStartPosition, rotationalCcdProposedPosition);
            Rotation = rotationalCcdProposedRotation;
            Collider.RebuildRuntimeShapeOnly();
        }
        else
        {
            PositionBasedOnForce();
            rotationalCcdProposedPosition = Position3d;
            if (CanRotate && _angularSpeed > Fixed64.Zero)
                RotationBasedOnTorque(rotationalCcdStartPosition, rotationalCcdProposedPosition);
        }

        CheckGroundForSimulation();
        ApplyGroundedHeightOrReset();

        CheckChangedValues();
        if (!CanRotate && !RotationChangePending)
            return;

        UpdateInertiaTensorOrientation();
        ApplyGyroscopicPrecession(angularVelocityStateStart);
    }

    private void PositionBasedOnForce()
    {
        Vector3d startPosition = Position3d;
        Vector3d velocityVector = AddPositionDisplacement(
            startPosition, ProjectLinearMotion(_linearVelocity) * Context.DeltaTime);
        TryResolveContinuousCollision(startPosition, ref velocityVector);
        ApplyResolvedPosition(startPosition, velocityVector);
    }

    private void ApplyResolvedPosition(Vector3d startPosition, Vector3d velocityVector)
    {
        velocityVector = ProjectLinearEndpoint(startPosition, velocityVector);

        Vector2d velocityAxis = velocityVector.ToVector2d();

        // Find out how much we need to push towards the ground to avoid loosing grounding
        // when walking down a step or over a sharp change in slope.
        if (HasCurrentGroundSupport && (_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
        {
            velocityVector.Y -= FixedMath.Max(StepOffset, velocityVector.Magnitude);
            _lastGroundedPosition = Position3d;
        }
        else if ((_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
            HeightPos = velocityVector.Y;

        //  Apply the force
        SetPosition2d(velocityAxis);
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

    //  gyroscopic precession is a correction to the object's angular velocity based on its rotation
    private void ApplyGyroscopicPrecession(Vector3d angularVelocityStateStart)
    {
        if (!CanRotate)
            return;

        Vector3d gyroscopicCorrection = ProjectAngularMotion(
            _inverseInertiaTensor
            * Vector3d.Cross(_angularVelocity, _worldInertiaTensor * _angularVelocity)
            * Context.DeltaTime);
        if (gyroscopicCorrection == Vector3d.Zero)
            return;

        _angularVelocity -= gyroscopicCorrection;
        _angularVelocity = ProjectAngularMotion(_angularVelocity);
        RefreshAngularMotionState(angularVelocityStateStart);
    }

}
