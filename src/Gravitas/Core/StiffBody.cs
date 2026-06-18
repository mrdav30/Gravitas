using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
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

    // Controls whether physics affects the rigidbody.
    // If enabled, transform is controlled by animation or script.
    public bool IsKinematic = false;

    private ContinuousCollisionMode _continuousCollisionMode = ContinuousCollisionMode.Inherit;
    private int _continuousCollisionFrameToken = int.MinValue;
    private Vector3d _continuousCollisionFrameStart;
    private Vector3d _continuousCollisionFrameDisplacement;

    /// <summary>
    /// Selects the deterministic tunneling guard used when this body commits frame movement.
    /// Inherited values resolve through the cached top-parent body before falling back to context settings.
    /// </summary>
    public ContinuousCollisionMode ContinuousCollisionMode
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionMode;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _continuousCollisionMode = value;
    }

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
            _position2dUnmarked.Set(value.X, value.Z);
            _heightPosUnmarked = value.Y;
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

    /// <summary>
    /// Selects the deterministic query primitive used for ground checks.
    /// </summary>
    public GroundProbeMode GroundProbeMode { get; set; } = GroundProbeMode.Auto;

    /// <summary>
    /// Optional explicit radius for swept-sphere ground probes. A zero value derives the radius from the collider.
    /// </summary>
    public Fixed64 GroundProbeRadius { get; set; }

    private int _lastGroundCheckFrame = 0;
    private const int _groundCheckFrameThreshold = 10;
    private readonly Fixed64 _groundCheckThreshold = (Fixed64)0.01f;
    private readonly SwiftList<Physics3DHit> _groundProbeHits = new();
    private readonly SwiftList<Physics3DHit> _continuousCollisionHits = new();
    private readonly SwiftList<PhysicsMixedHit> _continuousMixedCollisionHits = new();

    public Fixed64 StepOffset = (Fixed64)0.5f;

    private Vector3d _groundNormal = Vector3d.Zero;
    public Vector3d GroundNormal => _groundNormal;

    private FixedTransform? _hitPlatform;
    public FixedTransform? HitPlatform { get => _hitPlatform; set => _hitPlatform = value; }

    private Vector3d _hitPlatformPosition;

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
            OnGrounded?.Invoke(value);
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

    private bool _isSleeping;
    private int _sleepFrameCount;
    private bool _sleepEnabled = true;
    private int _sleepFrameThreshold = 16;
    private Fixed64 _sleepLinearSpeedThreshold = (Fixed64)0.001f;
    private Fixed64 _sleepAngularSpeedThreshold = (Fixed64)0.001f;

    /// <summary>
    /// Gets whether this dynamic body is currently excluded from solver work until a deterministic wake event occurs.
    /// </summary>
    public bool IsSleeping => _isSleeping;

    /// <summary>
    /// Enables deterministic sleep evaluation for this body.
    /// </summary>
    public bool SleepEnabled
    {
        get => _sleepEnabled;
        set
        {
            if (_sleepEnabled == value)
                return;

            _sleepEnabled = value;
            if (!value)
                Wake();
        }
    }

    /// <summary>
    /// Number of consecutive fixed frames below sleep thresholds required before the body sleeps.
    /// </summary>
    public int SleepFrameThreshold
    {
        get => _sleepFrameThreshold;
        set
        {
            SwiftThrowHelper.ThrowIfNegative(value, nameof(value));
            _sleepFrameThreshold = value;
        }
    }

    /// <summary>
    /// Linear speed at or below which the body can count toward sleeping.
    /// </summary>
    public Fixed64 SleepLinearSpeedThreshold
    {
        get => _sleepLinearSpeedThreshold;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value < Fixed64.Zero,
                nameof(value),
                "Sleep linear speed threshold cannot be negative.");
            _sleepLinearSpeedThreshold = value;
        }
    }

    /// <summary>
    /// Angular speed at or below which the body can count toward sleeping.
    /// </summary>
    public Fixed64 SleepAngularSpeedThreshold
    {
        get => _sleepAngularSpeedThreshold;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value < Fixed64.Zero,
                nameof(value),
                "Sleep angular speed threshold cannot be negative.");
            _sleepAngularSpeedThreshold = value;
        }
    }

    internal bool IsAwakeForCollision => Active && !Immovable && !IsSleeping;

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
    private Fixed64 _frictionCoefficient = Fixed64.One;

    /// <summary>
    /// Coulomb friction coefficient used by contact response and grounded motion.
    /// Values above one are allowed for intentionally high-friction surfaces.
    /// </summary>
    public Fixed64 FrictionCoefficient
    {
        get => _frictionCoefficient;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value < Fixed64.Zero,
                nameof(value),
                "Friction coefficient cannot be negative.");
            _frictionCoefficient = value;
        }
    }

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
        _angularSpeed = Fixed64.Zero;
        _normalForce = Vector3d.Zero;
        _isSleeping = false;
        _sleepFrameCount = 0;

        _isGrounded = false;
        _skipGroundingCheck = false;
        _lastGroundCheckFrame = int.MinValue;
        ResetGroundCalculations();

        _positionChangedBuffer = true;
        _position2dUnmarked = startPosition.ToVector2d();
        _lastGroundedPosition = _lastPosition = _spawnedPosition = startPosition;
        _heightPosUnmarked = startPosition.Y;

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
        CheckGround(force: true);

        RefreshInertiaTensor();
    }

    internal Vector3d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameStart;
    }

    internal Vector3d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameDisplacement;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        _continuousCollisionFrameToken = token;
        _continuousCollisionFrameStart = Position3d;
        _continuousCollisionFrameDisplacement = PredictContinuousCollisionDisplacement();
    }

    private Vector3d PredictContinuousCollisionDisplacement()
    {
        if (!Active || Immovable || IsKinematic || _isSleeping)
            return Vector3d.Zero;

        Fixed64 deltaTime = Context.DeltaTime;
        PhysicsEnvironment environment = Context.Environment;
        Vector3d predictedVelocity = _linearVelocity + _impulseStore + (_deltaAcceleration * deltaTime);
        if (!_isGrounded)
            predictedVelocity.Y -= environment.Gravity * deltaTime;

        predictedVelocity.Y = FixedMath.Max(predictedVelocity.Y, -environment.MaxFallSpeed);
        Fixed64 predictedSpeed = predictedVelocity.Magnitude;
        if (predictedSpeed > environment.MaxSpeed)
            predictedVelocity = predictedVelocity.Normalized * environment.MaxSpeed;
        else if (predictedSpeed <= environment.MinSpeed)
            predictedVelocity = Vector3d.Zero;

        return predictedVelocity * deltaTime;
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
            if (!IsSleeping)
            {
                ProcessMovable();
                UpdateSleepState();
            }

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
        if (Position3d != kinematicPosition)
            Wake();

        Position2d = kinematicPosition.ToVector2d();
        HeightPos = kinematicPosition.Y;
        SetVisualPosition(kinematicPosition);

        FixedQuaternion kinematicRotation = _rotationTransform.Rotation;
        if (Rotation != kinematicRotation)
            Wake();

        Rotation = kinematicRotation;
        SetVisualRotation(kinematicRotation);
    }

    /// <summary>
    /// Puts the body to sleep and keeps it partitioned for queries and deterministic wake propagation.
    /// </summary>
    public void Sleep()
    {
        if (!CanSleep)
            return;

        _sleepFrameCount = _sleepFrameThreshold;
        ClearMotionForSleep();
        if (_isSleeping)
            return;

        _isSleeping = true;
        RefreshPartitionAwakeState();
    }

    /// <summary>
    /// Wakes a sleeping body because a deterministic simulation or host stimulus changed its state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Wake()
    {
        _sleepFrameCount = 0;
        if (!_isSleeping)
            return;

        _isSleeping = false;
        RefreshPartitionAwakeState();
    }

    private bool CanSleep => Active && SleepEnabled && !Immovable && !IsKinematic;

    private bool CanUseAngularInertia => !Immovable && !IsKinematic && !PreventAngularForces;

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
        _decelerating = false;
        _isVelocityConstant = true;
    }

    private void RefreshPartitionAwakeState()
    {
        if (Collider is { IsPartitioned: true })
            Context.Collisions.RefreshPartitionAwakeState(Collider);
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
        if (torque != Vector3d.Zero)
            Wake();

        _deltaTorque += torque * _inverseInertiaTensor;
        Context.Diagnostics.EmitTorqueDelta(this, torque);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddForce(Vector3d force)
    {
        if (force != Vector3d.Zero)
            Wake();

        Vector3d accelerationDelta = force * InverseMass;
        _deltaAcceleration += accelerationDelta;
        Context.Diagnostics.EmitForceDelta(this, force, accelerationDelta);
    }

    private Vector3d _impulseStore = Vector3d.Zero;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddLinearImpulse(Vector3d impulse)
    {
        if (impulse != Vector3d.Zero)
            Wake();

        _impulseStore += (impulse * InverseMass) * Context.DeltaTime;
        // testing immediate reaction for collisions...
        UpdateLinearVelocity();
        NonKinematicUpdate();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddAngularImpulse(Vector3d impulse)
    {
        if (impulse != Vector3d.Zero)
            Wake();

        if (!AngularForcesHalted)
            _angularVelocity += (impulse * _inverseInertiaTensor) * Context.DeltaTime;
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector3d velocityDelta)
    {
        if (Immovable || IsKinematic || velocityDelta == Vector3d.Zero)
            return;

        Wake();
        Vector3d lastVelocity = _linearVelocity;
        _linearVelocity += velocityDelta;
        RefreshLinearMotionState(lastVelocity);
        Context.Diagnostics.EmitLinearVelocityDelta(this, lastVelocity, _linearVelocity);
    }

    internal void ApplyCollisionAngularVelocityDelta(Vector3d velocityDelta)
    {
        if (AngularForcesHalted || IsKinematic || velocityDelta == Vector3d.Zero)
            return;

        Wake();
        Vector3d lastVelocity = _angularVelocity;
        _angularVelocity += velocityDelta;
        RefreshAngularMotionState(lastVelocity);
        Context.Diagnostics.EmitAngularVelocityDelta(this, lastVelocity, _angularVelocity);
    }

    internal void ApplyCollisionPositionCorrection(Vector3d positionCorrection)
    {
        if (Immovable || IsKinematic || positionCorrection == Vector3d.Zero)
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
    public void AddPositionCorrection(Vector3d positionCorrection) => _positionCorrection += positionCorrection.ToVector2d();

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

        Vector2d horizontalVelocity = new(_linearVelocity.X, _linearVelocity.Z);
        Fixed64 horizontalSpeed = horizontalVelocity.Magnitude;
        if (horizontalSpeed <= Fixed64.Zero)
            return;

        // Object is moving on ground, add the friction force to the accumulated force
        // Adjust the friction with the normal force magnitude
        PhysicsEnvironment environment = Context.Environment;
        Fixed64 effectiveFriction = _frictionCoefficient;
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
            _linearVelocity.Y -= environment.Gravity * deltaTime;

        // Make sure we don't fall any faster than maxFallSpeed. This gives our character a terminal velocity
        _linearVelocity.Y = FixedMath.Max(_linearVelocity.Y, -environment.MaxFallSpeed);

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
        else if (desiredSpeed >= Fixed64.Zero)
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
        Fixed64 effectiveFriction = _frictionCoefficient;
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
        else if (desiredSpeed >= Fixed64.Zero)
        {
            _angularVelocity = Vector3d.Zero;
            _angularSpeed = Fixed64.Zero;
        }

        _angularAcceleration = _angularSpeed > Fixed64.Zero
            ? (_angularVelocity - lastVelocity) / Context.DeltaTime
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

        CheckGroundForSimulation();

        if (_isGrounded)
            HeightPos = HitPoint.Y;
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
        Vector3d startPosition = Position3d;
        Vector3d velocityVector = startPosition + (_linearVelocity * Context.DeltaTime);
        TryResolveContinuousCollision(startPosition, ref velocityVector);

        Vector2d velocityAxis = velocityVector.ToVector2d();

        // Find out how much we need to push towards the ground to avoid loosing grounding
        // when walking down a step or over a sharp change in slope.
        if (_isGrounded)
        {
            velocityVector.Y -= FixedMath.Max(StepOffset, velocityVector.Magnitude);
            _lastGroundedPosition = Position3d;
        }
        else
            HeightPos = velocityVector.Y;

        //  Apply the force
        Position2d = _positionCorrection + velocityAxis;
        _positionCorrection = Vector2d.Zero;
    }

    private bool TryResolveContinuousCollision(Vector3d startPosition, ref Vector3d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector3d displacement = proposedPosition - startPosition;
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && displacement.MagnitudeSquared <= proxyRadius * proxyRadius))
        {
            return false;
        }

        int hitCount = Context.Query3D.SweepSphereAll(
            startPosition,
            proposedPosition,
            proxyRadius,
            PhysicsLayerMask.All,
            _continuousCollisionHits,
            Collider);
        int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
            ? Context.QueryMixed.SweepSphereAgainst2DAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                PhysicsLayerMask.All,
                _continuousMixedCollisionHits,
                Collider,
                includeTriggers: false)
            : 0;

        bool found3D = TryGetFirstValidContinuousCollisionHit(hitCount, out Physics3DHit hit3D);
        bool foundDynamic3D = TryGetFirstDynamicContinuousCollisionHit(
            startPosition,
            proposedPosition,
            proxyRadius,
            out Physics3DHit dynamicHit3D,
            out Fixed64 dynamicClosingSpeed3D);
        if (ShouldReplaceContinuousCollisionHit(dynamicHit3D, dynamicClosingSpeed3D, foundDynamic3D, found3D, hit3D, Fixed64.Zero))
        {
            hit3D = dynamicHit3D;
            found3D = true;
        }

        bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(mixedHitCount, out PhysicsMixedHit hitMixed);
        bool foundDynamicMixed = TryGetFirstDynamicMixedContinuousCollisionHit(
            startPosition,
            proposedPosition,
            proxyRadius,
            out PhysicsMixedHit dynamicHitMixed,
            out Fixed64 dynamicClosingSpeedMixed);
        if (ShouldReplaceMixedContinuousCollisionHit(dynamicHitMixed, dynamicClosingSpeedMixed, foundDynamicMixed, foundMixed, hitMixed, Fixed64.Zero))
        {
            hitMixed = dynamicHitMixed;
            foundMixed = true;
        }

        if (found3D && (!foundMixed || hit3D.Distance <= hitMixed.Distance))
        {
            proposedPosition = startPosition + displacement.Normalized * hit3D.Distance;
            RemoveClosingContinuousCollisionVelocity(hit3D.Normal);
            return true;
        }

        if (foundMixed)
        {
            proposedPosition = startPosition + displacement.Normalized * hitMixed.Distance;
            RemoveClosingContinuousCollisionVelocity(hitMixed.NormalFor3DSource);
            return true;
        }

        return false;
    }

    private bool TryGetFirstValidContinuousCollisionHit(int hitCount, out Physics3DHit hit)
    {
        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit candidate = _continuousCollisionHits[i];
            if (!IsValidContinuousCollisionHit(candidate))
                continue;

            hit = candidate;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryGetFirstValidMixedContinuousCollisionHit(int hitCount, out PhysicsMixedHit hit)
    {
        for (int i = 0; i < hitCount; i++)
        {
            PhysicsMixedHit candidate = _continuousMixedCollisionHits[i];
            if (!IsValidMixedContinuousCollisionHit(candidate))
                continue;

            hit = candidate;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryGetFirstDynamicContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        out Physics3DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        bool found = false;
        Physics3DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        int peak = Context.Physics.DynamicBodyPeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (!Context.Physics.TryGetDynamicBody(i, out StiffBody target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = ResolveContinuousCollisionProxyRadius(target.Collider);
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    target.ContinuousCollisionFrameStart,
                    target.ContinuousCollisionFrameDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normal,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            Vector3d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = target.ContinuousCollisionFrameStart + target.ContinuousCollisionFrameDisplacement * normalizedTime;
            Vector3d point = ResolveDynamicContactPoint(sourceCenter, targetCenter, normal, targetRadius);
            var candidate = new Physics3DHit(target.Collider, point, normal, distance, sourceDirection);
            if (!ShouldReplaceContinuousCollisionHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool TryGetFirstDynamicMixedContinuousCollisionHit(
        Vector3d startPosition,
        Vector3d proposedPosition,
        Fixed64 proxyRadius,
        out PhysicsMixedHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector3d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceDirection = sourceDisplacement / sourceLength;
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        int peak = Context.Physics2D.DynamicBodyPeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (!Context.Physics2D.TryGetDynamicBody(i, out StiffBody2D target)
                || !IsEligibleDynamicMixed2DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = FixedMath.Max(
                target.ResolveContinuousCollisionProxyRadiusForDynamicTarget(),
                target.Collider.MixedHalfThickness);
            if (targetRadius <= Fixed64.Epsilon)
                continue;

            Vector2d targetStart2D = target.ContinuousCollisionFrameStart;
            Vector2d targetDisplacement2D = target.ContinuousCollisionFrameDisplacement;
            Vector3d targetStart = new(targetStart2D.X, target.Collider.MixedSlabCenterY, targetStart2D.Y);
            Vector3d targetDisplacement = new(targetDisplacement2D.X, Fixed64.Zero, targetDisplacement2D.Y);
            if (!ContinuousCollisionMath.TrySweepRelativeSpheres(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    targetStart,
                    targetDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normalForSource,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            Vector3d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = targetStart + targetDisplacement * normalizedTime;
            Vector3d point3D = sourceCenter - normalForSource * proxyRadius;
            Vector3d point2D = ResolveDynamicContactPoint(sourceCenter, targetCenter, normalForSource, targetRadius);
            var candidate = new PhysicsMixedHit(
                null,
                target.Collider,
                point3D,
                point2D,
                -normalForSource,
                distance,
                sourceDirection);
            if (!ShouldReplaceMixedContinuousCollisionHit(candidate, candidateClosingSpeed, true, found, best, bestClosingSpeed))
                continue;

            best = candidate;
            bestClosingSpeed = candidateClosingSpeed;
            found = true;
        }

        hit = best;
        closingSpeed = bestClosingSpeed;
        return found;
    }

    private bool IsEligibleDynamicContinuousCollisionTarget(StiffBody target)
    {
        if (ReferenceEquals(target, this)
            || !target.Active
            || target.Immovable
            || target.IsKinematic
            || target.Collider.IsTrigger
            || target.Collider.IsSibling(Collider)
            || Context.Physics.IsLayerCollisionDisabled(Collider.Layer, target.Collider.Layer))
        {
            return false;
        }

        return true;
    }

    private bool IsEligibleDynamicMixed2DTarget(StiffBody2D target)
    {
        return target.Active
            && !target.Immovable
            && !target.IsKinematic
            && !target.Collider.IsTrigger
            && Context.MixedCollisions.RequireCollisionPair(Collider, target.Collider);
    }

    private static bool ShouldReplaceContinuousCollisionHit(
        Physics3DHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        Physics3DHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        int candidateId = candidate.Collider?.Id ?? -1;
        int currentId = current.Collider?.Id ?? -1;
        return candidateId < currentId;
    }

    private static bool ShouldReplaceMixedContinuousCollisionHit(
        PhysicsMixedHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        PhysicsMixedHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        int candidate3D = candidate.Collider3D?.Id ?? -1;
        int current3D = current.Collider3D?.Id ?? -1;
        int collider3D = candidate3D.CompareTo(current3D);
        if (collider3D != 0)
            return collider3D < 0;

        int candidate2D = candidate.Collider2D?.Id ?? -1;
        int current2D = current.Collider2D?.Id ?? -1;
        return candidate2D < current2D;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveDynamicContactPoint(
        Vector3d sourceCenter,
        Vector3d targetCenter,
        Vector3d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector3d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool ShouldUseContinuousCollision(out ContinuousCollisionMode mode)
    {
        mode = ResolveContinuousCollisionMode();
        return mode == ContinuousCollisionMode.Continuous || mode == ContinuousCollisionMode.Auto;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ContinuousCollisionMode ResolveContinuousCollisionMode()
    {
        ContinuousCollisionMode mode = _continuousCollisionMode;
        if (mode != ContinuousCollisionMode.Inherit)
            return mode;

        StiffBody? parentBody = Collider.TopParent3D?.Body;
        if (parentBody != null && parentBody._continuousCollisionMode != ContinuousCollisionMode.Inherit)
            return parentBody._continuousCollisionMode;

        mode = Context.Settings.DefaultContinuousCollisionMode;
        return mode == ContinuousCollisionMode.Inherit
            ? ContinuousCollisionMode.Discrete
            : mode;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        return ResolveContinuousCollisionProxyRadius(Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadiusForDynamicTarget()
    {
        return ResolveContinuousCollisionProxyRadius(Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveContinuousCollisionProxyRadius(LSCollider collider)
    {
        return collider switch
        {
            LSSphereCollider sphere => sphere.ScaledRadius,
            LSCapsuleCollider capsule => capsule.ScaledRadius,
            LSCylinderCollider cylinder => cylinder.ScaledRadius,
            LSCuboidCollider cuboid => FixedMath.Min(
                cuboid.Bounds.Scope.X,
                FixedMath.Min(cuboid.Bounds.Scope.Y, cuboid.Bounds.Scope.Z)),
            LSCompoundCollider compound => FixedMath.Min(
                compound.Bounds.Scope.X,
                FixedMath.Min(compound.Bounds.Scope.Y, compound.Bounds.Scope.Z)),
            LSMeshCollider mesh => ResolveSmallestPositiveScope(mesh.Bounds.Scope),
            _ => Fixed64.Zero
        };
    }

    private static Fixed64 ResolveSmallestPositiveScope(Vector3d scope)
    {
        Fixed64 result = Fixed64.MaxValue;
        if (scope.X > Fixed64.Epsilon && scope.X < result)
            result = scope.X;
        if (scope.Y > Fixed64.Epsilon && scope.Y < result)
            result = scope.Y;
        if (scope.Z > Fixed64.Epsilon && scope.Z < result)
            result = scope.Z;

        return result == Fixed64.MaxValue ? Fixed64.Zero : result;
    }

    private bool IsValidContinuousCollisionHit(Physics3DHit hit)
    {
        LSCollider? hitCollider = hit.Collider;
        if (hitCollider == null
            || ReferenceEquals(hitCollider, Collider)
            || hitCollider.IsTrigger
            || hitCollider.IsSibling(Collider)
            || Context.Physics.IsLayerCollisionDisabled(Collider.Layer, hitCollider.Layer))
        {
            return false;
        }

        StiffBody? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private bool IsValidMixedContinuousCollisionHit(PhysicsMixedHit hit)
    {
        LSCollider2D? hitCollider = hit.Collider2D;
        if (hitCollider == null
            || hitCollider.IsTrigger
            || !Context.MixedCollisions.RequireCollisionPair(Collider, hitCollider))
        {
            return false;
        }

        StiffBody2D? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private void RemoveClosingContinuousCollisionVelocity(Vector3d normal)
    {
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return;

        Fixed64 closingSpeed = Vector3d.Dot(_linearVelocity, normal);
        if (closingSpeed >= Fixed64.Zero)
            return;

        Vector3d lastVelocity = _linearVelocity;
        _linearVelocity -= normal * closingSpeed;
        RefreshLinearMotionState(lastVelocity);
        Context.Diagnostics.EmitLinearVelocityDelta(this, lastVelocity, _linearVelocity);
    }

    private void RotationBasedOnTorque()
    {
        // Convert angular velocity to a quaternion
        FixedQuaternion angularVelocityQuaternion = new(_angularVelocity.X, _angularVelocity.Y, _angularVelocity.Z, Fixed64.Zero);
        FixedQuaternion spin = angularVelocityQuaternion * Rotation * Fixed64.Half * Context.DeltaTime;
        Rotation = (Rotation + spin).Normalized;
    }

    private void UpdateIntertiaTensorOrientation()
    {
        if (_interiaTensor == Fixed3x3.Zero)
            return;

        Fixed3x3 inverseOrientation = Rotation.Conjugate().ToMatrix3x3();
        Fixed3x3 orientation = Rotation.ToMatrix3x3();

        _inverseInertiaTensor = orientation * _inverseInertiaTensor * inverseOrientation;
    }

    private void RefreshInertiaTensor()
    {
        if (!CanUseAngularInertia || Collider == null)
        {
            _interiaTensor = Fixed3x3.Zero;
            _inverseInertiaTensor = Fixed3x3.Zero;
            return;
        }

        _interiaTensor = Collider.CalculateInertiaTensor(Mass);
        _inverseInertiaTensor = _interiaTensor.InvertDiagonal();
        UpdateIntertiaTensorOrientation();
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

        if (Context.ResetAccumulationThisVisualize)
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

        Fixed64 targetSpeed = ResolveVisualRotationStep();
        FixedQuaternion expectedRotation = _rotationInterpoleSpeed > Fixed64.Zero
            ? FixedQuaternion.Slerp(_rotationTransform.Rotation, _visualRotation, targetSpeed)
            : FixedQuaternion.Slerp(_lastVisualRotation, _visualRotation, targetSpeed);
        _rotationTransform.Rotation = expectedRotation;
    }

    private Fixed64 ResolveVisualRotationStep()
    {
        if (_rotationInterpoleSpeed <= Fixed64.Zero)
            return Context.ExpectedAccumulation;

        return FixedMath.Clamp01(Context.DeltaTime * _rotationInterpoleSpeed * _rotationSpeed);
    }

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
        _skipGroundingCheck = true;
        ClearGrounding();
        Context.Coroutines.StartCoroutine(SkipGroundingCoroutine(secs));
    }

    private IEnumerator<ILockedYieldInstruction> SkipGroundingCoroutine(Fixed64 secs)
    {
        yield return Context.Coroutines.WaitForRealSeconds(secs);
        _skipGroundingCheck = false;
    }

    public void CheckGround() => CheckGround(force: true);

    private void CheckGroundForSimulation() => CheckGround(force: false);

    private void CheckGround(bool force)
    {
        if (_skipGroundingCheck || World is null)
        {
            ClearGrounding();
            return;
        }

        // Only perform SphereCast if enough frames have passed
        bool hitPlatformMoved = _hitPlatform != null && _hitPlatform.Position != _hitPlatformPosition;
        bool frameGuard = !force
            && !hitPlatformMoved
            && Vector3d.Distance(_lastPosition, Position3d) < _groundCheckThreshold
            && Context.FrameCount - _lastGroundCheckFrame < _groundCheckFrameThreshold;
        if (frameGuard)
            return;

        _lastGroundCheckFrame = Context.FrameCount;
        // We want origin to be close to the actor's feet
        Vector3d origin = Position3d;
        // but not to close...
        origin.Y += GroundOriginOffset;

        Fixed64 dis = GroundedDistanceRay;
        if (!IsGrounded)
            dis = GroundDownDistanceOnAir;

        GroundProbeMode mode = ResolveGroundProbeMode();
        Fixed64 radius = mode == GroundProbeMode.SweptSphere
            ? ResolveGroundProbeRadius()
            : Fixed64.Zero;
        Vector3d end = origin + Vector3d.Down * dis;
        bool foundGround = TryFindGroundHit(mode, radius, origin, dis, out Physics3DHit hit);
        Context.Diagnostics.EmitGroundProbe(this, mode, origin, end, radius, foundGround, hit);

        if (!foundGround)
        {
            ClearGrounding();
            return;
        }

        _hitPlatform = hit.Collider?.Transform;
        _hitPlatformPosition = _hitPlatform?.Position ?? Vector3d.Zero;
        _hitPoint = hit.Point;
        _groundNormal = hit.Normal;

        Vector3d weightVector = Weight * Vector3d.Down;
        Fixed64 weightInNormalDirection = Vector3d.Dot(weightVector, _groundNormal);
        _normalForce = weightInNormalDirection * _groundNormal;

        IsGrounded = true;
    }

    private bool TryFindGroundHit(
        GroundProbeMode mode,
        Fixed64 radius,
        Vector3d origin,
        Fixed64 distance,
        out Physics3DHit hit)
    {
        if (mode == GroundProbeMode.SweptSphere && TryFindGroundHitWithSweptSphere(origin, distance, radius, out hit))
            return true;

        if (mode == GroundProbeMode.SweptSphere)
        {
            hit = default;
            return false;
        }

        return TryFindGroundHitWithRay(origin, distance, out hit);
    }

    private bool TryFindGroundHitWithRay(Vector3d origin, Fixed64 distance, out Physics3DHit hit)
    {
        Vector3d end = origin + Vector3d.Down * distance;
        int hitCount = Context.Query3D.RaycastAll(origin, end, Context.Settings.GroundCheckLayerMask, _groundProbeHits);
        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit current = _groundProbeHits[i];
            if (!IsValidGroundHit(current))
                continue;

            hit = current;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryFindGroundHitWithSweptSphere(Vector3d origin, Fixed64 distance, Fixed64 radius, out Physics3DHit hit)
    {
        if (radius <= Fixed64.Epsilon)
            return TryFindGroundHitWithRay(origin, distance, out hit);

        Vector3d end = origin + Vector3d.Down * distance;
        int hitCount = Context.Query3D.SweepSphereAll(
            origin,
            end,
            radius,
            Context.Settings.GroundCheckLayerMask,
            _groundProbeHits,
            Collider);

        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit current = _groundProbeHits[i];
            if (!IsValidGroundHit(current))
                continue;

            hit = current;
            return true;
        }

        hit = default;
        return false;
    }

    private bool IsValidGroundHit(Physics3DHit hit)
    {
        LSCollider? hitCollider = hit.Collider;
        if (hitCollider == null || ReferenceEquals(hitCollider, Collider))
            return false;

        StiffBody? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private GroundProbeMode ResolveGroundProbeMode()
    {
        if (GroundProbeMode != GroundProbeMode.Auto)
            return GroundProbeMode;

        return Collider is LSSphereCollider
            || Collider is LSCapsuleCollider
            || Collider is LSCylinderCollider
            || (Collider is LSCuboidCollider && ResolveGroundProbeRadius() > Fixed64.FromFraction(1, 8))
            || (Collider is LSCompoundCollider && ResolveGroundProbeRadius() > Fixed64.FromFraction(1, 8))
                ? GroundProbeMode.SweptSphere
                : GroundProbeMode.Ray;
    }

    private Fixed64 ResolveGroundProbeRadius()
    {
        if (GroundProbeRadius > Fixed64.Zero)
            return GroundProbeRadius;

        return Collider switch
        {
            LSSphereCollider sphere => sphere.ScaledRadius,
            LSCapsuleCollider capsule => capsule.ScaledRadius,
            LSCylinderCollider cylinder => cylinder.ScaledRadius,
            LSCuboidCollider cuboid => FixedMath.Min(cuboid.Bounds.Scope.X, cuboid.Bounds.Scope.Z),
            LSCompoundCollider compound => FixedMath.Min(compound.Bounds.Scope.X, compound.Bounds.Scope.Z),
            _ => Fixed64.Zero
        };
    }

    private void ClearGrounding()
    {
        IsGrounded = false;
        ResetGroundCalculations();
    }

    private void ResetGroundCalculations()
    {
        _hitPlatform = null;
        _hitPlatformPosition = Vector3d.Zero;
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
        return Position3d + Rotation * Vector3d.Multiply(Collider?.ScaledSize ?? Vector3d.One, point);
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
        return Vector3d.Multiply(Vector3d.One / (Collider?.ScaledSize ?? Vector3d.One), rotated);
    }

    public void ResetPosition(Vector3d position = default, FixedQuaternion rotation = default)
    {
        _linearAcceleration = Vector3d.Zero;
        _linearVelocity = Vector3d.Zero;
        _angularVelocity = Vector3d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
        _isSleeping = false;
        _sleepFrameCount = 0;
        _normalForce = Vector3d.Zero;

        Position2d = position.ToVector2d();
        HeightPos = position.Y;
        _lastPosition = position;
        _lastVisualPosition = _visualPosition = position;
        _positionTransform.Position = position;
        Rotation = rotation;

        _visualRotation = rotation;
        _rotationTransform.Rotation = rotation;
    }

    public void RecordData(IChronicler chronicler)
    {
        GroundProbeMode groundProbeMode = GroundProbeMode;
        Fixed64 groundProbeRadius = GroundProbeRadius;

        RecordValues.Look(chronicler, ref Debug, "Debug");
        RecordValues.Look(chronicler, ref Active, "Active");
        RecordValues.Look(chronicler, ref Immovable, "Immovable");
        RecordValues.Look(chronicler, ref IsKinematic, "IsKinematic", false);
        RecordValues.Look(chronicler, ref _position2dUnmarked, "Position2d");
        RecordValues.Look(chronicler, ref _heightPosUnmarked, "HeightPos");
        RecordValues.Look(chronicler, ref _spawnedPosition, "SpawnedPosition");
        RecordValues.Look(chronicler, ref _lastPosition, "LastPosition");
        RecordValues.Look(chronicler, ref GroundOriginOffset, "GroundOriginOffset");
        RecordValues.Look(chronicler, ref GroundedDistanceRay, "GroundedDistanceRay");
        RecordValues.Look(chronicler, ref GroundDownDistanceOnAir, "GroundDownDistanceOnAir");
        RecordValues.Look(chronicler, ref groundProbeMode, "GroundProbeMode", GroundProbeMode.Auto);
        RecordValues.Look(chronicler, ref groundProbeRadius, "GroundProbeRadius");
        RecordValues.Look(chronicler, ref _skipGroundingCheck, "SkipGroundingCheck", false);
        RecordValues.Look(chronicler, ref _lastGroundCheckFrame, "LastGroundCheckFrame");
        RecordValues.Look(chronicler, ref StepOffset, "StepOffset");
        RecordValues.Look(chronicler, ref _groundNormal, "GroundNormal");
        RecordValues.Look(chronicler, ref _hitPlatformPosition, "HitPlatformPosition");
        RecordValues.Look(chronicler, ref _hitPoint, "HitPoint");
        RecordValues.Look(chronicler, ref _isGrounded, "IsGrounded");
        RecordValues.Look(chronicler, ref _lastGroundedPosition, "LastGroundedPosition");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref PreventAngularForces, "PreventAngularForces");
        RecordValues.Look(chronicler, ref _linearVelocity, "LinearVelocity");
        RecordValues.Look(chronicler, ref _linearDirection, "LinearDirection");
        RecordValues.Look(chronicler, ref _angularVelocity, "AngularVelocity");
        RecordValues.Look(chronicler, ref _angularDirection, "AngularDirection");
        RecordValues.Look(chronicler, ref _deltaTorque, "DeltaTorque");
        RecordValues.Look(chronicler, ref RestitutionCoefficient, "RestitutionCoefficient");
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
        RecordValues.Look(chronicler, ref _frictionCoefficient, "FrictionCoefficient");
        RecordValues.Look(chronicler, ref _normalForce, "NormalForce");
        RecordValues.Look(chronicler, ref Mass, "Mass");

        if (chronicler.Mode == SerializationMode.Loading)
        {
            GroundProbeMode = groundProbeMode;
            GroundProbeRadius = groundProbeRadius;
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

        RefreshInertiaTensor();

        Collider?.Simulate();
    }
}
