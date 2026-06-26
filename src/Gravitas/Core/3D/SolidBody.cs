//=======================================================================
// SolidBody.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using GridForge.Grids;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody : IRecordable
{
    public bool Debug = false;

    public bool Active = false;

    private int _dynamicId = -1;
    public int DynamicId => _dynamicId;  // Physics Id, if not set it's assumed the object isn't simulated
    private bool _isSet = false;

    private bool _immovable;
    public bool Immovable
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _immovable;
        set
        {
            if (_immovable == value)
                return;

            _immovable = value;
            RefreshPartitionMobility();
        }
    }

    // Controls whether physics affects the rigidbody.
    // If enabled, transform is controlled by animation or script.
    private bool _isKinematic;
    public bool IsKinematic
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _isKinematic;
        set
        {
            if (_isKinematic == value)
                return;

            _isKinematic = value;
            RefreshPartitionMobility();
        }
    }

    private ContinuousCollisionMode _continuousCollisionMode = ContinuousCollisionMode.Inherit;
    private int _continuousCollisionFrameToken = int.MinValue;
    private Vector3d _continuousCollisionFrameStart;
    private Vector3d _continuousCollisionFrameDisplacement;
    private FixedQuaternion _continuousCollisionFrameRotation;
    private bool _continuousCollisionHandoffPending;
    private int _continuousCollisionHandoffToken = int.MinValue;
    private Fixed64 _continuousCollisionHandoffRemainingTime;

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

    /// <summary>
    /// Gets the number of continuous-collision impacts consumed by the most recent late simulation step.
    /// </summary>
    public int LastContinuousCollisionToiIterationCount { get; private set; }

    /// <summary>
    /// Gets whether the most recent late simulation step reached the configured continuous-collision TOI iteration limit.
    /// </summary>
    public bool LastContinuousCollisionToiIterationLimitReached { get; private set; }

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

    private readonly SwiftList<Physics3DHit> _continuousCollisionHits = new();
    private readonly SwiftList<PhysicsMixedHit> _continuousMixedCollisionHits = new();
    private readonly ContactManifold _rotationalContinuousCollisionManifold = new();
    private readonly SweptSphereQueryWorker _shapeExactContinuousSweepWorker = new();
    private readonly ConvexSweepQueryWorker _shapeExactContinuousConvexSweepWorker = new();
    private static readonly Fixed64 ShapeExactContinuousContactSlop = Fixed64.FromFraction(1, 2048);


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

    private Vector3d _localCenterOfMassOffset;
    private bool _centerOfMassOffsetExplicit;

    /// <summary>
    /// Gets or sets the authoritative body-local center-of-mass offset used by response and inertia.
    /// </summary>
    public Vector3d LocalCenterOfMassOffset
    {
        get => _localCenterOfMassOffset;
        set
        {
            if (_localCenterOfMassOffset == value && _centerOfMassOffsetExplicit)
                return;

            _localCenterOfMassOffset = value;
            _centerOfMassOffsetExplicit = true;
            if (!Active)
                return;

            Wake();
            RefreshInertiaTensor();
        }
    }

    /// <summary>
    /// Gets the authoritative world-space center of mass.
    /// </summary>
    public Vector3d WorldCenterOfMass => Position3d + (Rotation * _localCenterOfMassOffset);

    /// <summary>
    /// Clears an explicit center-of-mass override and derives the offset from the bound collider again.
    /// </summary>
    public void ResetCenterOfMassFromCollider()
    {
        _centerOfMassOffsetExplicit = false;
        RefreshMassPropertiesFromColliderShape();
    }

    private Fixed3x3 _inertiaTensor;
    private Fixed3x3 _worldInertiaTensor;
    private Fixed3x3 _inverseLocalInertiaTensor;
    private Fixed3x3 _inverseInertiaTensor;
    public Fixed3x3 InverseInertiaTensor => _inverseInertiaTensor;

    /// <summary>
    /// Gets whether solver-side response may translate this body.
    /// </summary>
    public bool CanTranslate => Active && !Immovable && !IsKinematic && InverseMass > Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may rotate this body.
    /// </summary>
    public bool CanRotate => CanTranslate && !PreventAngularForces && _inverseInertiaTensor != Fixed3x3.Zero;

    /// <summary>
    /// Gets the inverse mass that should be used by collision response.
    /// Immovable and kinematic bodies expose their raw mass but respond as infinite mass.
    /// </summary>
    public Fixed64 EffectiveInverseMass => CanTranslate ? InverseMass : Fixed64.Zero;

    /// <summary>
    /// Gets the inverse inertia tensor that should be used by collision response.
    /// Bodies that cannot rotate expose a zero tensor even when raw inertia is available.
    /// </summary>
    public Fixed3x3 EffectiveInverseInertiaTensor => CanRotate ? _inverseInertiaTensor : Fixed3x3.Zero;

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

    public SolidBody(IMatterAgent agent, LSCollider collider) => Setup(agent, collider, null, null);

    public SolidBody(
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
            GravitasLogger.Channel.Error($"SolidBody must be set up with an agent and collider before initialization.");
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
        RefreshMassPropertiesFromColliderShape();
        CheckGround(force: true);
    }


    public void LateSimulate() => LateSimulate(updateSleepState: true, updateColliderState: true);

    internal void LateSimulate(bool updateSleepState) => LateSimulate(updateSleepState, updateColliderState: true);

    internal void LateSimulate(bool updateSleepState, bool updateColliderState)
    {
        if (!Active) return;

        LastContinuousCollisionToiIterationCount = 0;
        LastContinuousCollisionToiIterationLimitReached = false;

        _lastPosition = Position3d;
        if (TryConsumeContinuousCollisionHandoff(updateSleepState, updateColliderState))
            return;

        if (IsKinematic)
            UpdateKinematicPositionAndRotation();

        // if we can't move...then we don't and ignore any forces
        if (!Immovable)
        {
            if (!IsSleeping)
            {
                ProcessMovable();
                if (updateSleepState)
                    UpdateSleepState();
            }

            if (updateColliderState)
                Collider!.Simulate();
        }

        if (SettingVisuals)
            _settingVisualsCounter--;

        if (PositionChangePending || RotationChangePending)
            OnMoved?.Invoke();
    }

    internal void UpdateSleepStateAfterPhysicsStep()
    {
        if (Active && !Immovable && !IsSleeping)
            UpdateSleepState();
    }

    private void UpdateKinematicPositionAndRotation()
    {
        Vector3d startPosition = Position3d;
        FixedQuaternion startRotation = Rotation;
        Vector3d kinematicPosition = _positionTransform.Position;
        FixedQuaternion kinematicRotation = _rotationTransform.Rotation;
        if (startPosition == kinematicPosition && startRotation == kinematicRotation)
            return;

        Wake();

        Vector3d resolvedPosition = kinematicPosition;
        FixedQuaternion resolvedRotation = kinematicRotation;
        CaptureKinematicContinuousCollisionFrame(startPosition, kinematicPosition, startRotation);
        TryResolveKinematicContinuousCollision(startPosition, ref resolvedPosition);
        TryResolveKinematicRotationalContinuousCollision(startPosition, ref resolvedPosition, startRotation, ref resolvedRotation);

        if (resolvedPosition != kinematicPosition)
            _positionTransform.Position = resolvedPosition;
        if (resolvedRotation != kinematicRotation)
            _rotationTransform.Rotation = resolvedRotation;

        Position2d = resolvedPosition.ToVector2d();
        HeightPos = resolvedPosition.Y;
        SetVisualPosition(resolvedPosition);

        Rotation = resolvedRotation;
        SetVisualRotation(resolvedRotation);
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

    internal void WakeFromCollision()
    {
        if (!_isSleeping)
            return;

        _sleepFrameCount = 0;
        _isSleeping = false;
        RefreshPartitionAwakeState();
    }

    private bool CanSleep => Active && SleepEnabled && !Immovable && !IsKinematic;


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
        bool wasSleeping = _isSleeping;
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

        if (wasSleeping)
            RefreshPartitionAwakeState();
    }

}
