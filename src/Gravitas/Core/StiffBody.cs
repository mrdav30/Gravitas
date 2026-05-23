using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Raycasting;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

public class StiffBody : IRecordable
{
    public bool Debug = false;

    public bool Active = false;

    private int _dynamicId = -1;
    public int DynamicId => _dynamicId;  // Physics Id, if not set it's assumed the object isn't simulated
    private bool _isSet = false;

    public bool Immovable = false;

    // Controls whether physics affects the rigidbody
    // If enabled, transform is controlled by animation or script
    // Position & Rotation updated in LateVisualize to account for animation movement
    public bool IsKinematic = false;

    private FixedTransform _positionTransform = null!;
    public FixedTransform PositionTransform => _positionTransform;

    private FixedTransform _rotationTransform = null!;
    public FixedTransform RotationTransform => _rotationTransform;

    #region Position Properties

    private bool _positionMutated;
    private bool _positionChangedBuffer;
    public bool PositionChangePending => _positionMutated || _positionChangedBuffer;

    private Vector2d _position2dUnmarked;
    public Vector2d Position2d
    {
        get { return _position2dUnmarked; }
        private set
        {
            if (_position2dUnmarked == value)
                return;
            _position2dUnmarked = value;
            _positionMutated = true;
        }
    }

    // Used to correct position after collision resolution to avoid sinking into other objects or the ground
    private Vector2d _positionCorrection;

    public Vector3d Position3d
    {
        get => _position2dUnmarked.ToVector3d(_heightPosUnmarked);
        private set
        {
            if (Position3d == value)
                return;
            _position2dUnmarked.Set(value.x, value.z);
            _heightPosUnmarked = value.y;
            _positionMutated = true;
        }
    }

    private Fixed64 _heightPosUnmarked = Fixed64.Zero;  // Actor's transform position Y
    public Fixed64 HeightPos
    {
        get => _heightPosUnmarked;
        private set
        {
            if (_heightPosUnmarked == value)
                return;
            _heightPosUnmarked = value;
            _positionMutated = true;
        }
    }

    private Vector3d _spawnedPosition;
    public Vector3d SpawnedPosition => _spawnedPosition;

    private Vector3d _lastPosition;
    public Vector3d LastPosition => _lastPosition;

    #region Grounding

    private bool _skipGroundingCheck = false;

    // how close to the actor's feet (or whatever touches the ground) do we check for grounding
    public Fixed64 GroundOriginOffset = (Fixed64)0.5f;

    public Fixed64 GroundedDistanceRay = (Fixed64)0.5f;

    public Fixed64 GroundDownDistanceOnAir = (Fixed64)0.5f;

    public Fixed64 GroundCheckSphereRadius = (Fixed64)0.2f;

    private int _lastGroundCheckFrame = 0;
    private const int _groundCheckFrameThreshold = 10;
    private readonly Fixed64 _groundCheckThreshold = (Fixed64)0.01f;

    public Fixed64 StepOffset = (Fixed64)0.5f;

    private Vector3d _groundNormal = Vector3d.Zero;
    public Vector3d GroundNormal => _groundNormal;

    private FixedTransform? _hitPlatform;
    public FixedTransform? HitPlatform { get => _hitPlatform; set => _hitPlatform = value; }

    private Vector3d _hitPoint;
    public Vector3d HitPoint => _hitPoint;

    public Action<bool>? OnGrounded;

    private bool _isGrounded;
    public bool IsGrounded
    {
        get => _isGrounded;
        private set
        {
            if (_isGrounded == value)
                return;
            _isGrounded = value;
            OnGrounded?.Invoke(true);
        }
    }

    private Vector3d _lastGroundedPosition;
    public Vector3d LastGroundedPosition => _lastGroundedPosition;

    #endregion

    public bool CanSetVisualPosition;

    private Vector3d _visualPosition;
    public Vector3d VisualPosition => _visualPosition;

    private Vector3d _lastVisualPosition;
    public Vector3d LastVisualPosition => _lastVisualPosition;

    #endregion

    #region Rotation Properties

    private bool _rotationMutated;
    private bool _rotationChangedBuffer;
    public bool RotationChangePending => _rotationMutated || _rotationChangedBuffer;

    private FixedQuaternion _rotation;

    public FixedQuaternion Rotation
    {
        get => _rotation;
        private set
        {
            if (_rotation == value)
                return;
            _rotation = value;
            _rotationMutated = true;
        }
    }

    public Fixed3x3 RotationMatrix => _rotation.ToMatrix3x3();

    public Vector3d Forward
    {
        get => _rotation.Rotate(Vector3d.Forward);
        private set
        {
            if (value == Vector3d.Zero)
                return;

            // Convert the direction vector to a rotation quaternion
            Rotation = FixedQuaternion.FromDirection(value);
        }
    }

    public Vector3d Up => _rotation.Rotate(Vector3d.Up);
    public Vector3d Right => _rotation.Rotate(Vector3d.Right);

    public bool CanSetVisualRotation;

    private FixedQuaternion _visualRotation;
    public FixedQuaternion VisualRotation => _visualRotation;

    private FixedQuaternion _lastVisualRotation;
    public FixedQuaternion LastVisualRotation => _lastVisualRotation;

    // Prevents any forces from being applied to the body that would cause it to rotate.
    public bool PreventAngularForces;

    public bool AngularForcesHalted => Immovable || PreventAngularForces;

    public Fixed64 DefaultRotationSpeed = (Fixed64)30; // 1 for NPC...

    public Fixed64 InteractionRotationSpeed = (Fixed64)3; // 0.15 for NPC...

    private Fixed64 _rotationSpeed;
    private Fixed64 _rotationInterpoleSpeed;

    #endregion

    private int _settingVisualsCounter;
    private bool SettingVisuals => _settingVisualsCounter > 0;

    /// <summary>
    /// The desiredSpeed an object has in a specific direction
    /// AKA units per second the unit is moving
    /// </summary>
    private Vector3d _linearVelocity;
    public Vector3d LinearVelocity => _linearVelocity;

    private Vector3d _linearDirection;

    /// <summary>
    /// Represents the angular velocity of the body.
    /// </summary>
    private Vector3d _angularVelocity;
    public Vector3d AngularVelocity => _angularVelocity;

    private Vector3d _angularDirection;

    /// <summary>
    /// Represents the torque applied to the body.
    /// </summary>
    private Vector3d _deltaTorque;

    private Fixed3x3 _interiaTensor;
    private Fixed3x3 _inverseInertiaTensor;
    public Fixed3x3 InverseInteriaTensor => _inverseInertiaTensor;

    /// <summary>
    /// Value between 0 (sticky) and 1 (perfectly elastic collision; i.e. not moving apart)
    /// </summary>
    public Fixed64 RestitutionCoefficient = (Fixed64)0.5f;

    public bool IsAtRest => _linearVelocity.IsZero && _angularVelocity.IsZero;

    // LinearVelocity magnitude
    private Fixed64 _linearSpeed;
    public Fixed64 LinearSpeed => _linearSpeed;

    /// <summary>
    /// Represents the total accumulated force on the object. This can be the sum of all external forces acting on the object, such as gravity, push/pull forces, etc.
    /// Changing this value directly affects the object's acceleration and, subsequently, its velocity and position.
    /// </summary>
    private Vector3d _linearAccelerationStore;
    private Vector3d _deltaAcceleration;

    private Vector3d _linearAcceleration;
    public Vector3d LinearAcceleration => _linearAcceleration;

    private Fixed64 _angularSpeed;
    public Fixed64 AngularSpeed => _angularSpeed;

    private Vector3d _angularAccelerationStore;

    private Vector3d _angularAcceleration;
    public Vector3d AngularAcceleration => _angularAcceleration;


    private Vector3d _timeScaledAcceleration;
    public Vector3d TimeScaledAcceleration => _timeScaledAcceleration;

    //Cleaner stops with more decelleration
    public Vector3d _timeScaledDeceleration;
    public Vector3d TimeScaledDeceleration => _timeScaledDeceleration;

    // If acceleration is in the opposite direction to its velocity
    private bool _decelerating;

    private bool _isVelocityConstant;

    /// <summary>
    /// Represents a body's resistance to movement, akin to air resistance.
    /// Higher values slow down the body more quickly in absence of other forces.
    /// The effect is significant when bodies are expected to slow down or stop without sustained forces.
    /// It's not constrained between 0 and 1, depends on the object's shape and the flow conditions.
    /// </summary>
    public Fixed64 LinearDragCoefficient = (Fixed64)0.75f;

    private Fixed64 AngularDragCoefficient = (Fixed64)0.75f;

    /// <summary>
    /// Represents the friction force applied when the object is moving.
    /// Higher values simulate high friction surfaces causing quick stops, while lower values simulate low friction surfaces causing prolonged slides.
    /// The usual range is between 0 (no friction) and 1 (high friction).
    /// </summary>
    private Fixed64 FrictionCoefficient = Fixed64.One;

    /// <summary>
    /// Represents the normal force on the object.
    /// It's usually perpendicular to the contact surface and prevents the object from "falling" into the surface.
    /// Can be updated to simulate changes in terrain or surface inclination.
    /// </summary>
    private Vector3d _normalForce;

    //  Mass (in kilograms) is the measure of the amount of matter in a body
    //  Divide the weight (in Newtons) by the acceleration of gravity to determine the mass of an object (measured in Kilograms).
    //  On Earth, gravity accelerates at 9.8 meters per second squared (9.8 m/s^2)
    //  ex: 150 Pounds x PhysicsEnvironment.PoundToNewton = 667 Newtons / 9.8 m/s^2 = 68 kilograms * PhysicsEnvironment.KilogramToPound = 150 Pounds
    public Fixed64 Mass;

    // InverseMass is the reciprocal of mass, which is useful for performance reasons
    // when mass is used in calculations.
    public Fixed64 InverseMass => Mass != Fixed64.Zero
        ? Fixed64.One / Mass
        : Fixed64.Zero;

    // Weight is a measure of how the force of gravity acts upon the mass.
    // Weight (in Newtons) is mass (in Kilograms) multiplied by the acceleration of gravity (g).
    // ex: 68 kg * 9.8 m/s^2 = 667 Newtons / PhysicsEnvironment.PoundToNewton = 150 Pounds
    private Fixed64 Weight => Mass * Context.Environment.Gravity;

    public IMatterAgent Agent { get; private set; } = null!;

    public GravitasWorldContext Context { get; private set; } = null!;

    public GridWorld? World => Context?.World;

    public LSCollider Collider { get; private set; } = null!;

    /// <summary>
    /// Called after visual position/rotation updated
    /// </summary>
    public Action? OnMoved;

    public StiffBody(IMatterAgent agent, LSCollider collider) => Setup(agent, collider, null, null);

    public StiffBody(
        IMatterAgent agent,
        LSCollider collider,
        FixedTransform? positionTransform,
        FixedTransform? rotationTransform) => Setup(agent, collider, positionTransform, rotationTransform);

    public void Setup(
        IMatterAgent agent,
        LSCollider collider,
        FixedTransform? positionTransform,
        FixedTransform? rotationTransform)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));

        GravitasWorldContext context = agent.Context;
        SwiftThrowHelper.ThrowIfNull(context, nameof(agent.Context));
        SwiftThrowHelper.ThrowIfArgument(
            collider.TryGetBoundContext(out GravitasWorldContext? colliderContext)
                && !ReferenceEquals(colliderContext, context),
            nameof(collider),
            "Agent and collider must be bound to the same context.");

        Agent = agent;
        Collider = collider;
        Context = context;
        Collider.BindContext(context);

        _positionTransform = positionTransform ?? agent.Transform;
        _rotationTransform = rotationTransform ?? agent.Transform;

        _rotationSpeed = DefaultRotationSpeed;
        _rotationInterpoleSpeed = Fixed64.Zero;

        _isSet = true;
    }

    public void Initialize(
        Vector3d startPosition,
        FixedQuaternion startRotation,
        bool isDynamic = true)
    {
        if (!_isSet)
        {
            GravitasLogger.Channel.Error($"StiffBody must be set up with an agent and collider before initialization.");
            return;
        }

        Active = true;

        _linearAcceleration = Vector3d.Zero;
        _linearVelocity = Vector3d.Zero;
        _angularVelocity = Vector3d.Zero;
        _linearSpeed = Fixed64.Zero;
        _normalForce = Vector3d.Zero;

        _isGrounded = true;

        _positionChangedBuffer = true;
        _position2dUnmarked = startPosition.ToVector2d();
        _lastPosition = _spawnedPosition = startPosition;
        _heightPosUnmarked = startPosition.y;

        _rotationChangedBuffer = true;
        _rotation = startRotation;

        if (!IsKinematic)
        {
            _lastVisualPosition = _visualPosition = Position3d;
            _visualRotation = startRotation;
            _lastVisualRotation = _visualRotation;
        }

        OnVisualize();

        _dynamicId = Context.Physics.AssimilateBody(this, isDynamic);
        Collider!.Initialize(this);

        if (AngularForcesHalted)
            return;

        _interiaTensor = Collider.CalculateInertiaTensor(Mass);
        _inverseInertiaTensor = _interiaTensor.InvertDiagonal();
        UpdateIntertiaTensorOrientation();
    }

    public void LateSimulate()
    {
        if (!Active) return;

        _lastPosition = Position3d;

        if (IsKinematic)
            UpdateKinematicPositionAndRotation();

        // if we can't move...then we don't and ignore any forces
        if (!Immovable)
        {
            ProcessMovable();
            Collider!.Simulate();
        }

        if (SettingVisuals)
            _settingVisualsCounter--;

        if (PositionChangePending || RotationChangePending)
            OnMoved?.Invoke();
    }

    private void UpdateKinematicPositionAndRotation()
    {
        Vector3d kinematicPosition = _positionTransform.Position;
        Position2d = kinematicPosition.ToVector2d();
        HeightPos = kinematicPosition.y;
        SetVisualPosition(kinematicPosition);

        FixedQuaternion kinematicRotation = _rotationTransform.Rotation;
        Rotation = kinematicRotation;
        SetVisualRotation(kinematicRotation);
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
    public void AddTorque(Vector3d torque) => _deltaTorque += torque * _inverseInertiaTensor;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddForce(Vector3d force) => _deltaAcceleration += force * InverseMass;

    private Vector3d _impulseStore = Vector3d.Zero;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddLinearImpulse(Vector3d impulse)
    {
        _impulseStore += (impulse * InverseMass) * Context.DeltaTime;
        // testing immediate reaction for collisions...
        UpdateLinearVelocity();
        NonKinematicUpdate();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddAngularImpulse(Vector3d impulse)
    {
        if (!AngularForcesHalted)
            _angularVelocity += (impulse * _inverseInertiaTensor) * Context.DeltaTime;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPosition(Vector3d position) => Position3d = position;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddPositionCorrection(Vector3d positionCorrection) => _positionCorrection += positionCorrection.ToVector2d();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetHeight(Fixed64 height) => HeightPos = height;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetRotation(FixedQuaternion quaternion) => Rotation = quaternion;

    private void ProcessMovable()
    {
        ApplyLinearForces();
        UpdateLinearVelocity();

        if (!AngularForcesHalted)
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
        _linearAccelerationStore = _deltaAcceleration;
        _deltaAcceleration = Vector3d.Zero;

        if (_linearSpeed <= Fixed64.Zero)
            return;

        ApplyDragForce();
        ApplyFrictionForce();
    }

    private void ApplyDragForce()
    {
        // Drag calculation and accumulation.
        Fixed64 dragMagnitude = LinearDragCoefficient * Context.Environment.AirDensity * Collider!.GetFrontalArea(_linearDirection) * _linearSpeed;
        _linearAccelerationStore += (-_linearDirection * dragMagnitude);
    }

    private void ApplyFrictionForce()
    {
        if (!_isGrounded)
            return;

        Vector2d horizontalVelocity = new(_linearVelocity.x, _linearVelocity.z);
        Fixed64 horizontalSpeed = horizontalVelocity.Magnitude;
        if (horizontalSpeed <= Fixed64.Zero)
            return;

        // Object is moving on ground, add the friction force to the accumulated force
        // Adjust the friction with the normal force magnitude
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 effectiveFriction = FrictionCoefficient;
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

        _linearVelocity += _impulseStore + (_linearAccelerationStore * deltaTime);
        // LinearVelocity = _impulseStore + (_linearAccelerationStore * deltaTime);

        // Reset stores for the next frame
        _linearAccelerationStore = Vector3d.Zero;
        _impulseStore = Vector3d.Zero;

        // Apply gravity only if not grounded
        if (!IsGrounded)
            _linearVelocity.y -= environment.Gravity * deltaTime;

        // Make sure we don't fall any faster than maxFallSpeed. This gives our character a terminal velocity
        _linearVelocity.y = FixedMath.Max(_linearVelocity.y, -environment.MaxFallSpeed);

        Fixed64 desiredSpeed = _linearVelocity.Magnitude;
        if (desiredSpeed > environment.MinSpeed)
        {
            if (desiredSpeed > environment.MaxSpeed)
            {
                _linearVelocity = _linearVelocity.Normal * environment.MaxSpeed;
                _linearSpeed = environment.MaxSpeed;
            }
            else
                _linearSpeed = desiredSpeed;
        }
        else if (desiredSpeed >= Fixed64.Zero)
        {
            _linearVelocity = Vector3d.Zero;
            _linearSpeed = Fixed64.Zero;
        }

        // Update the direction of the linear velocity, if we're not moving maintain previous direction
        _linearDirection = _linearSpeed > Fixed64.Zero
            ? _linearVelocity.Normal
            : _linearDirection;
        _linearAcceleration = _linearSpeed > Fixed64.Zero
            ? (_linearVelocity - lastVelocity) / deltaTime
            : Vector3d.Zero;
    }

    private void ApplyAngularTorques()
    {
        _angularAccelerationStore = _deltaTorque;
        _deltaTorque = Vector3d.Zero;

        if (_angularSpeed <= Fixed64.Zero)
            return;

        ApplyDragTorque();
        ApplyFrictionTorque();
    }

    private void ApplyDragTorque()
    {
        // Angular drag should also be proportional to the square of the angular velocity, just like linear drag.
        Fixed64 angularDragMagnitude = AngularDragCoefficient * Context.Environment.AirDensity * Collider!.GetFrontalArea(_angularDirection) * _angularSpeed;
        _angularAccelerationStore += (-_angularDirection * angularDragMagnitude);
    }

    private void ApplyFrictionTorque()
    {
        if (!IsGrounded)
            return;

        // Calculate the friction force and convert it into a torque
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 effectiveFriction = FrictionCoefficient;
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
        _angularVelocity += _angularAccelerationStore * deltaTime;
        // Reset the acceleration store for the next frame
        _angularAccelerationStore = Vector3d.Zero;

        // Add damping torque, proportional to negative angular velocity
        Vector3d dampingTorque = -environment.DampingFactor * _angularVelocity;
        _angularVelocity += _inverseInertiaTensor * dampingTorque * deltaTime;

        Fixed64 desiredSpeed = _angularVelocity.Magnitude;
        if (desiredSpeed > environment.MinSpeed)
        {
            if (_angularSpeed > environment.MaxSpeed)
            {
                _angularVelocity = _angularVelocity.Normal * environment.MaxSpeed;
                _angularSpeed = environment.MaxSpeed;

            }
            else
            {
                _angularSpeed = desiredSpeed;
                _angularDirection = _angularSpeed > Fixed64.Zero ? _angularVelocity.Normal : Vector3d.Zero;
            }
        }
        else if (desiredSpeed >= Fixed64.Zero)
        {
            _angularVelocity = Vector3d.Zero;
            _angularSpeed = Fixed64.Zero;
        }

        _angularAcceleration = _angularSpeed > Fixed64.Zero
            ? (_angularVelocity - lastVelocity) / deltaTime
            : Vector3d.Zero;
    }

    private void NonKinematicUpdate()
    {
        if (IsKinematic)
            return;

        //Position2d += _positionCorrection;
        //_positionCorrection = Vector2d.Zero;

        //// if (_linearSpeed > Fixed64.Zero)
        PositionBasedOnForce();
        if (!AngularForcesHalted && _angularSpeed > Fixed64.Zero)
            RotationBasedOnTorque();

        CheckGround();

        if (_isGrounded)
            HeightPos = HitPoint.y;
        else
            ResetGroundCalculations();

        CheckChangedValues();
        if (AngularForcesHalted && !RotationChangePending)
            return;

        UpdateIntertiaTensorOrientation();
        ApplyGyroscopicPrecession();
    }

    private void PositionBasedOnForce()
    {
        Vector3d velocityVector = Position3d + (_linearVelocity * Context.DeltaTime);
        Vector2d velocityAxis = velocityVector.ToVector2d();

        // Find out how much we need to push towards the ground to avoid loosing grounding
        // when walking down a step or over a sharp change in slope.
        if (_isGrounded)
        {
            velocityVector.y -= FixedMath.Max(StepOffset, velocityVector.Magnitude);
            _lastGroundedPosition = Position3d;
        }
        else
            HeightPos = velocityVector.y;

        //  Apply the force
        Position2d = _positionCorrection + velocityAxis;
        _positionCorrection = Vector2d.Zero;
    }

    private void RotationBasedOnTorque()
    {
        // Convert angular velocity to a quaternion
        FixedQuaternion angularVelocityQuaternion = new(_angularVelocity.x, _angularVelocity.y, _angularVelocity.z, Fixed64.Zero);
        FixedQuaternion spin = angularVelocityQuaternion * Rotation * Fixed64.Half * Context.DeltaTime;
        Rotation = (Rotation + spin).Normal;
    }

    private void UpdateIntertiaTensorOrientation()
    {
        if (_interiaTensor != Fixed3x3.Zero)
            return;

        Fixed3x3 inverseOrientation = Rotation.Conjugate().ToMatrix3x3();
        Fixed3x3 orientation = Rotation.ToMatrix3x3();

        _inverseInertiaTensor = orientation * _inverseInertiaTensor * inverseOrientation;
    }

    //  gyroscopic precession is a correction to the object's angular velocity based on its rotation
    private void ApplyGyroscopicPrecession()
    {
        _angularVelocity += _inverseInertiaTensor * Vector3d.Cross(_angularVelocity, _interiaTensor * _angularVelocity) * Context.DeltaTime;
    }

    public void OnVisualize()
    {
        if (!Active || Immovable || IsKinematic || !SettingVisuals)
            return;

        if (Context.ResetAccumulation)
        {
            if (CanSetVisualPosition)
                SetVisualPosition(Position3d);
            if (CanSetVisualRotation)
                SetVisualRotation(_rotation);
        }

        if (CanSetVisualPosition)
        {
            Vector3d expectedPosition = Vector3d.SpeedLerp(_lastVisualPosition, _visualPosition, Fixed64.One, Context.ExpectedAccumulation);
            _positionTransform.Position = expectedPosition;
        }

        if (!CanSetVisualRotation)
            return;

        Fixed64 targetSpeed = _rotationInterpoleSpeed > Fixed64.Zero
            ? Context.DeltaTime * _rotationInterpoleSpeed * _rotationSpeed
            : Context.ExpectedAccumulation;
        FixedQuaternion expectedRotation = FixedQuaternion.Slerp(_lastVisualRotation, _visualRotation, targetSpeed);
        _rotationTransform.Rotation = expectedRotation;
    }

    public void LateVisualize() { }

    public void Deactivate()
    {
        if (!Active)
            return;

        Collider!.Deactivate();
        Context.Physics.DessimilateBody(this);
        _dynamicId = -1;
        Active = false;
    }

    public void CheckChangedValues()
    {
        // we want to keep the buffers true until the next time we visualize, so we can be sure to update visuals at least once after a change

        if (_positionMutated)
        {
            _positionChangedBuffer = _positionMutated;
            _positionMutated = false;
            _settingVisualsCounter = Context.FrameRate;
        }
        else
            _positionChangedBuffer = false;

        if (_rotationMutated)
        {
            _rotationChangedBuffer = _rotationMutated;
            _rotationMutated = false;
            _settingVisualsCounter = Context.FrameRate;
        }
        else
            _rotationChangedBuffer = false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetVisualPosition(Vector3d position)
    {
        _lastVisualPosition = _visualPosition;
        _visualPosition = position;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetVisualRotation(FixedQuaternion rot)
    {
        _lastVisualRotation = _visualRotation;
        _visualRotation = rot;
    }

    public void UpdateAcceleration()
    {
        Vector3d lastAcceleration = _linearAcceleration;

        // if we aren't decelerating then we're...
        if ((_linearAcceleration - lastAcceleration).Magnitude.Abs() < Fixed64.Epsilon)
        {
            _isVelocityConstant = true;
            _decelerating = false;
            return;
        }

        _isVelocityConstant = false;
        if (_linearAcceleration.Magnitude > lastAcceleration.Magnitude)
            _decelerating = false;
        else
            _decelerating = true;
    }

    public void SkipGrounding(Fixed64 secs)
    {
        IsGrounded = false;
        Context.Coroutines.StartCoroutine(SkipGroundingCoroutine(secs));
    }

    private IEnumerator<ILockedYieldInstruction> SkipGroundingCoroutine(Fixed64 secs)
    {
        _skipGroundingCheck = true;
        yield return Context.Coroutines.WaitForRealSeconds(secs);
        _skipGroundingCheck = false;
    }

    public void CheckGround()
    {
        if (_skipGroundingCheck || World is null)
        {
            _isGrounded = false;
            return;
        }

        // Only perform SphereCast if enough frames have passed
        bool frameGuard = Vector3d.Distance(_lastPosition, Position3d) < _groundCheckThreshold
            && Context.FrameCount - _lastGroundCheckFrame < _groundCheckFrameThreshold;
        if (frameGuard)
            return;

        _lastGroundCheckFrame = Context.FrameCount;
        // We want origin to be close to the actor's feet
        Vector3d origin = Position3d;
        // but not to close...
        origin.y += GroundOriginOffset;

        Fixed64 dis = GroundedDistanceRay;
        if (!IsGrounded)
            dis = GroundDownDistanceOnAir;

        if (!Context.Circlecasts.CircleCast(origin, GroundCheckSphereRadius, Vector3d.Down, out LSRaycastHit hit, dis, Context.Settings.IgnoreForGroundCheck))
        {
            _isGrounded = false;
            return;
        }

        _hitPlatform = hit.Collider?.Transform;
        _hitPoint = hit.Point;
        _groundNormal = hit.Normal;

        Vector3d weightVector = Weight * Vector3d.Down;
        Fixed64 weightInNormalDirection = Vector3d.Dot(weightVector, _groundNormal);
        _normalForce = weightInNormalDirection * _groundNormal;

        _isGrounded = true;
    }

    private void ResetGroundCalculations()
    {
        _hitPlatform = null;
        _hitPoint = Vector3d.Zero;
        _groundNormal = Vector3d.Zero;
        _normalForce = Vector3d.Zero;
    }

    // https://forum.unity.com/threads/getting-impact-force-not-just-velocity.23746/
    // https://www2.chem.wisc.edu/deptfiles/genchem/netorial/modules/thermodynamics/energy/energy2.htm
    /// <summary>
    /// Energy possessed by an object in motion that describes things like how long it will take to stop
    /// and how much damage it will do in a collision (aka measurement of how strong the hit was)
    /// If kinetic energy is greater than defined break energy...do something, ex:
    ///  Debug.Log ("Impulse taken: " + result.magnitude);
    ///  if(result.magnitude > minForceToBreak){
    ///  Destroy(gameObject);
    /// </summary>
    /// <returns>
    /// Result is joules
    /// one Joule is equal to 1 kg m^2 / s^2
    /// </returns>
    public Fixed64 KineticEnergy()
    {
        // mass in kg, velocity in meters per second
        Fixed64 halfMass = Fixed64.Half * Mass;
        return halfMass * LinearSpeed;
    }

    public void UpdateRotation(FixedQuaternion targetRotation, Fixed64 bufferInterpolation)
    {
        _rotationInterpoleSpeed = bufferInterpolation;
        _rotationSpeed = Agent?.IsInteracting == true
            ? InteractionRotationSpeed
            : DefaultRotationSpeed;
        Rotation = targetRotation;
    }

    public void Rotate(Fixed64 sin, Fixed64 cos)
    {
        Rotate(sin, cos, Vector3d.Up);
    }

    public void Rotate(Fixed64 sin, Fixed64 cos, Vector3d axis)
    {
        // Apply the rotation
        Rotation = Rotation.Rotated(sin, cos, axis);
    }

    public Vector3d TransformDirection(Vector3d localDirection)
    {
        // Multiply the local direction by the rotation matrix
        Vector3d worldDirection = RotationMatrix * localDirection;
        return worldDirection;
    }

    public Vector3d InverseTransformDirection(Vector3d direction)
    {
        // Then transpose the rotation matrix
        Fixed3x3 transposedMatrix = Fixed3x3.Transpose(RotationMatrix);

        // Finally, multiply the direction by the transposed matrix
        Vector3d localDirection = transposedMatrix * direction;

        return localDirection;
    }

    /// <summary>
    /// Transforms position from local space to world space.
    /// </summary>
    /// <param name="point"></param>
    /// <returns></returns>
    public Vector3d TransformPoint(Vector3d point)
    {
        return Position3d + Rotation * Vector3d.Scale(Collider?.ScaledSize ?? Vector3d.One, point);
    }

    /// <summary>
    /// Transforms position from world space to local space.
    /// </summary>
    /// <param name="point"></param>
    /// <returns></returns>
    public Vector3d InverseTransformPoint(Vector3d point)
    {
        // first negate position  (point - transform.position)
        Vector3d translated = point - Position3d;
        // next negate the rotation (Quaternion.Inverse(rotation)
        Vector3d rotated = Rotation.Inverse() * translated;
        // Finally, negate scaling by dividing 1 by the value
        return Vector3d.Scale(Vector3d.One / (Collider?.ScaledSize ?? Vector3d.One), rotated);
    }

    public void ResetPosition(Vector3d position = default, FixedQuaternion rotation = default)
    {
        _linearAcceleration = Vector3d.Zero;
        _linearVelocity = Vector3d.Zero;
        _angularVelocity = Vector3d.Zero;
        _linearSpeed = Fixed64.Zero;
        _normalForce = Vector3d.Zero;

        Position2d = position.ToVector2d();
        HeightPos = position.y;
        _lastPosition = position;
        _lastVisualPosition = _visualPosition = position;
        _positionTransform.Position = position;
        Rotation = rotation;

        _visualRotation = rotation;
        _rotationTransform.Rotation = rotation;
    }

    public void RecordData(IChronicler chronicler)
    {
        RecordValues.Look(chronicler, ref Debug, "Debug");
        RecordValues.Look(chronicler, ref Active, "Active");
        RecordValues.Look(chronicler, ref Immovable, "Immovable");
        RecordValues.Look(chronicler, ref _positionTransform, "PositionTransform");
        RecordValues.Look(chronicler, ref _rotationTransform, "RotationTransform");
        RecordValues.Look(chronicler, ref _positionChangedBuffer, "PositionChangedBuffer");
        RecordValues.Look(chronicler, ref _rotationChangedBuffer, "RotationChangedBuffer");
        RecordValues.Look(chronicler, ref _position2dUnmarked, "Position2d");
        RecordValues.Look(chronicler, ref _heightPosUnmarked, "HeightPos");
        RecordValues.Look(chronicler, ref _spawnedPosition, "SpawnedPosition");
        RecordValues.Look(chronicler, ref _lastPosition, "LastPosition");
        RecordValues.Look(chronicler, ref GroundOriginOffset, "GroundOriginOffset");
        RecordValues.Look(chronicler, ref GroundedDistanceRay, "GroundedDistanceRay");
        RecordValues.Look(chronicler, ref GroundDownDistanceOnAir, "GroundDownDistanceOnAir");
        RecordValues.Look(chronicler, ref StepOffset, "StepOffset");
        RecordValues.Look(chronicler, ref _groundNormal, "GroundNormal");
        RecordValues.Look(chronicler, ref _hitPlatform, "HitPlatform");
        RecordValues.Look(chronicler, ref _hitPoint, "HitPoint");
        RecordValues.Look(chronicler, ref _isGrounded, "IsGrounded");
        RecordValues.Look(chronicler, ref _lastGroundedPosition, "LastGroundedPosition");
        RecordValues.Look(chronicler, ref CanSetVisualPosition, "CanSetVisualPosition");
        RecordValues.Look(chronicler, ref _visualPosition, "VisualPosition");
        RecordValues.Look(chronicler, ref _lastVisualPosition, "LastVisualPosition");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref CanSetVisualRotation, "CanSetVisualRotation");
        RecordValues.Look(chronicler, ref _visualRotation, "VisualRotation");
        RecordValues.Look(chronicler, ref _lastVisualRotation, "LastVisualRotation");
        RecordValues.Look(chronicler, ref PreventAngularForces, "PreventAngularForces");
        RecordValues.Look(chronicler, ref DefaultRotationSpeed, "DefaultRotationSpeed");
        RecordValues.Look(chronicler, ref InteractionRotationSpeed, "InteractionRotationSpeed");
        RecordValues.Look(chronicler, ref _rotationSpeed, "RotationSpeed");
        RecordValues.Look(chronicler, ref _rotationInterpoleSpeed, "RotationInterpoleSpeed");
        RecordValues.Look(chronicler, ref _settingVisualsCounter, "SettingVisualsCounter");
        RecordValues.Look(chronicler, ref _linearVelocity, "LinearVelocity");
        RecordValues.Look(chronicler, ref _linearDirection, "LinearDirection");
        RecordValues.Look(chronicler, ref _angularVelocity, "AngularVelocity");
        RecordValues.Look(chronicler, ref _angularDirection, "AngularDirection");
        RecordValues.Look(chronicler, ref RestitutionCoefficient, "RestitutionCoefficient");
        RecordValues.Look(chronicler, ref _linearSpeed, "LinearSpeed");
        RecordValues.Look(chronicler, ref _linearAccelerationStore, "LinearAccelerationStore");
        RecordValues.Look(chronicler, ref _deltaAcceleration, "DeltaAcceleration");
        RecordValues.Look(chronicler, ref _linearAcceleration, "LinearAcceleration");
        RecordValues.Look(chronicler, ref _angularSpeed, "AngularSpeed");
        RecordValues.Look(chronicler, ref _angularAccelerationStore, "AngularAccelerationStore");
        RecordValues.Look(chronicler, ref _angularAcceleration, "AngularAcceleration");
        RecordValues.Look(chronicler, ref _timeScaledAcceleration, "TimeScaledAcceleration");
        RecordValues.Look(chronicler, ref _timeScaledDeceleration, "TimeScaledDeceleration");
        RecordValues.Look(chronicler, ref _decelerating, "Decelerating");
        RecordValues.Look(chronicler, ref _isVelocityConstant, "IsVelocityConstant");
        RecordValues.Look(chronicler, ref LinearDragCoefficient, "LinearDragCoefficient");
        RecordValues.Look(chronicler, ref AngularDragCoefficient, "AngularDragCoefficient");
        RecordValues.Look(chronicler, ref FrictionCoefficient, "FrictionCoefficient");
        RecordValues.Look(chronicler, ref _normalForce, "NormalForce");
        RecordValues.Look(chronicler, ref Mass, "Mass");

        Collider?.RecordData(chronicler);
    }
}
