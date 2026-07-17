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

    public bool Active { get; private set; }

    private int _dynamicId = -1;
    public int DynamicId => _dynamicId;  // Physics Id, if not set it's assumed the object isn't simulated

    private BodyFreezeAxes3D _freezeAxes;

    /// <summary>
    /// Gets or sets the 3D translational and rotational degrees of freedom
    /// frozen for solver response, integration, CCD, and partition mobility.
    /// </summary>
    public BodyFreezeAxes3D FreezeAxes
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _freezeAxes;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                (value & ~BodyFreezeAxes3D.All) != BodyFreezeAxes3D.None,
                nameof(value),
                "Unsupported 3D freeze axis bits.");

            if (_freezeAxes == value)
                return;

            _freezeAxes = value;
            ApplyFreezeConstraintsToMotion();
            RefreshInertiaTensor();
            RefreshPartitionMobility();
        }
    }

    /// <summary>
    /// Gets whether all translation axes are frozen. Such bodies behave as
    /// static-equivalent participants for solver and partition mobility.
    /// </summary>
    public bool IsPositionFullyFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (_freezeAxes & BodyFreezeAxes3D.Position) == BodyFreezeAxes3D.Position;
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
    private void SetPosition2d(Vector2d value)
    {
        if (_position2dUnmarked == value)
            return;

        _position2dUnmarked = value;
        _positionMutated = true;
    }

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

    private Vector3d _lastPosition;

    private const int DefaultBodyHitBufferCapacity = 16;
    private readonly SwiftList<Physics3DHit> _continuousCollisionHits = new(DefaultBodyHitBufferCapacity);
    private readonly SwiftList<PhysicsMixedHit> _continuousMixedCollisionHits = new(DefaultBodyHitBufferCapacity);
    private readonly ContactManifold _rotationalContinuousCollisionManifold = new();
    private readonly SweptSphereQueryWorker _shapeExactContinuousSweepWorker = new();
    private readonly ConvexSweepQueryWorker _shapeExactContinuousConvexSweepWorker = new();
    // Shape-exact CCD must remain separated even when a degenerate support
    // feature is recognized at the far edge of the convex sweep contact band.
    private static readonly Fixed64 ShapeExactContinuousContactSlop =
        ConvexSweepQueryWorker.ContactTolerance * (Fixed64)4;


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

    public Vector3d Forward => _rotation.Rotate(Vector3d.Forward);
    public Vector3d Up => _rotation.Rotate(Vector3d.Up);
    public Vector3d Right => _rotation.Rotate(Vector3d.Right);

    public bool CanSetVisualRotation;

    private FixedQuaternion _visualRotation;
    public FixedQuaternion VisualRotation => _visualRotation;

    private FixedQuaternion _lastVisualRotation;
    public FixedQuaternion LastVisualRotation => _lastVisualRotation;

    /// <summary>
    /// Gets whether all angular axes are frozen.
    /// </summary>
    public bool AngularMotionFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => IsRotationFullyFrozen;
    }

    public bool AngularForcesHalted => IsPositionFullyFrozen || AngularMotionFrozen;

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
    public bool CanTranslate => Active && _dynamicId >= 0 && !IsPositionFullyFrozen && !IsKinematic && InverseMass > Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may rotate this body.
    /// </summary>
    public bool CanRotate => CanTranslate && !IsRotationFullyFrozen && _inverseInertiaTensor != Fixed3x3.Zero;

    /// <summary>
    /// Gets the inverse mass that should be used by collision response.
    /// Fully position-frozen and kinematic bodies expose their raw mass but respond as infinite mass.
    /// </summary>
    public Fixed64 EffectiveInverseMass => CanTranslate ? InverseMass : Fixed64.Zero;

    /// <summary>
    /// Gets the inverse inertia tensor that should be used by collision response.
    /// Bodies that cannot rotate expose a zero tensor even when raw inertia is available.
    /// </summary>
    public Fixed3x3 EffectiveInverseInertiaTensor => CanRotate ? _inverseInertiaTensor : Fixed3x3.Zero;

    internal bool IsRotationFullyFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (_freezeAxes & BodyFreezeAxes3D.Rotation) == BodyFreezeAxes3D.Rotation;
    }

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
        if (value == Vector3d.Zero || IsPositionFullyFrozen || IsRotationFullyFrozen)
            return Vector3d.Zero;

        Fixed64 x = (_freezeAxes & BodyFreezeAxes3D.RotationX) == BodyFreezeAxes3D.RotationX ? Fixed64.Zero : value.X;
        Fixed64 y = (_freezeAxes & BodyFreezeAxes3D.RotationY) == BodyFreezeAxes3D.RotationY ? Fixed64.Zero : value.Y;
        Fixed64 z = (_freezeAxes & BodyFreezeAxes3D.RotationZ) == BodyFreezeAxes3D.RotationZ ? Fixed64.Zero : value.Z;
        return new Vector3d(x, y, z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 GetConstrainedInverseMass(Vector3d axis)
    {
        if (!CanTranslate || axis == Vector3d.Zero)
            return Fixed64.Zero;

        Fixed64 axisMagnitudeSquared = axis.MagnitudeSquared;
        if (axisMagnitudeSquared <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Vector3d allowedAxis = ProjectLinearMotion(axis);
        Fixed64 allowedScale = Vector3d.Dot(allowedAxis, axis) / axisMagnitudeSquared;
        return allowedScale > Fixed64.Zero ? InverseMass * allowedScale : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector3d ApplyConstrainedInverseInertia(Vector3d torqueAxis)
    {
        if (!CanRotate || torqueAxis == Vector3d.Zero)
            return Vector3d.Zero;

        return ProjectAngularMotion(_inverseInertiaTensor * torqueAxis);
    }

    private Fixed64 _gravityScale = Fixed64.One;

    /// <summary>
    /// Multiplies context environment gravity for this body. Zero disables gravity-derived acceleration and grounded weight.
    /// </summary>
    public Fixed64 GravityScale
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _gravityScale;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value < Fixed64.Zero,
                nameof(value),
                "Gravity scale cannot be negative.");
            _gravityScale = value;
            RefreshGroundNormalForce();
        }
    }

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

    internal bool IsAwakeForCollision => Active && !IsPositionFullyFrozen && !IsSleeping;

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


    /// <summary>
    /// Represents a body's resistance to movement, akin to air resistance.
    /// Higher values slow down the body more quickly in absence of other forces.
    /// The effect is significant when bodies are expected to slow down or stop without sustained forces.
    /// It's not constrained between 0 and 1, depends on the object's shape and the flow conditions.
    /// </summary>
    public Fixed64 LinearDragCoefficient = (Fixed64)0.75f;

    private Fixed64 AngularDragCoefficient = (Fixed64)0.75f;

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
    private Fixed64 Weight => Mass * Context.Environment.Gravity * _gravityScale;

    public IMatterAgent Agent { get; private set; } = null!;

    public GravitasWorldContext Context { get; private set; } = null!;

    public GridWorld World => Context.World;

    public LSCollider Collider { get; private set; } = null!;

    /// <summary>
    /// Called after visual position/rotation updated
    /// </summary>
    public Action? OnMoved;

    public SolidBody(IMatterAgent agent, LSCollider collider)
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

        _positionTransform = agent.Transform;
        _rotationTransform = agent.Transform;

        _rotationSpeed = DefaultRotationSpeed;
        _rotationInterpoleSpeed = Fixed64.Zero;
    }

    public void Initialize(
        Vector3d startPosition,
        FixedQuaternion startRotation,
        bool isDynamic = true)
    {
        SwiftThrowHelper.ThrowIfArgument(
            Collider.Id >= 0
                || (Collider.HasHostBinding && !ReferenceEquals(Collider.Body, this)),
            nameof(Collider),
            "Body collider must be unregistered and free of another host binding before initialization.");
        Collider.PreflightBodyInitialization(this);

        Active = true;

        ClearMotionForSleep();
        _normalForce = Vector3d.Zero;
        _isSleeping = false;
        _sleepFrameCount = 0;

        _isGrounded = false;
        _wasGrounded = false;
        _groundedTransitionCapturedForStep = false;
        _skipGroundingCheck = false;
        _lastGroundCheckFrame = int.MinValue;
        ResetGroundCalculations();

        _positionChangedBuffer = true;
        _position2dUnmarked = startPosition.ToVector2d();
        _lastGroundedPosition = _lastPosition = startPosition;
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

    internal void LateSimulate(bool updateSleepState, bool updateColliderState)
    {
        if (!Active) return;

        CaptureGroundedStepState();
        try
        {
            LastContinuousCollisionToiIterationCount = 0;
            LastContinuousCollisionToiIterationLimitReached = false;

            _lastPosition = Position3d;
            if (TryConsumeContinuousCollisionHandoff(updateSleepState, updateColliderState))
                return;

            if (IsKinematic)
                UpdateKinematicPositionAndRotation();

            // if we can't move...then we don't and ignore any forces
            if (!IsPositionFullyFrozen)
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
        finally
        {
            CompleteGroundedStepState();
        }
    }

    internal void UpdateSleepStateAfterPhysicsStep()
    {
        if (!IsSleeping)
            UpdateSleepState();
    }

    private void UpdateKinematicPositionAndRotation()
    {
        Vector3d startPosition = Position3d;
        FixedQuaternion startRotation = Rotation;
        Vector3d kinematicPosition = _positionTransform.WorldPosition;
        FixedQuaternion kinematicRotation = _rotationTransform.WorldRotation;
        if (startPosition == kinematicPosition && startRotation == kinematicRotation)
            return;

        Wake();

        Vector3d resolvedPosition = ProjectLinearEndpoint(startPosition, kinematicPosition);
        if (ShouldUseContinuousCollision(out _))
            _ = ContinuousCollisionSweepRange.ValidateEndpoint(startPosition, resolvedPosition, out _);
        FixedQuaternion resolvedRotation = kinematicRotation;
        CaptureKinematicContinuousCollisionFrame(startPosition, resolvedPosition, startRotation);
        TryResolveKinematicContinuousCollision(startPosition, ref resolvedPosition);
        TryResolveKinematicRotationalContinuousCollision(startPosition, ref resolvedPosition, startRotation, ref resolvedRotation);

        if (resolvedPosition != kinematicPosition)
            SetPositionTransformWorldPosition(resolvedPosition);
        if (resolvedRotation != kinematicRotation)
            SetRotationTransformWorldRotation(resolvedRotation);

        SetPosition2d(resolvedPosition.ToVector2d());
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

    private bool CanSleep => Active && SleepEnabled && !IsPositionFullyFrozen && !IsKinematic;


    internal void OnVisualize()
    {
        if (IsPositionFullyFrozen || IsKinematic || !SettingVisuals)
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
            SetPositionTransformWorldPosition(expectedPosition);
        }

        if (!CanSetVisualRotation)
            return;

        Fixed64 targetSpeed = ResolveVisualRotationStep();
        FixedQuaternion expectedRotation = _rotationInterpoleSpeed > Fixed64.Zero
            ? FixedQuaternion.Slerp(_rotationTransform.WorldRotation, _visualRotation, targetSpeed)
            : FixedQuaternion.Slerp(_lastVisualRotation, _visualRotation, targetSpeed);
        SetRotationTransformWorldRotation(expectedRotation);
    }

    private Fixed64 ResolveVisualRotationStep()
    {
        if (_rotationInterpoleSpeed <= Fixed64.Zero)
            return Context.ExpectedAccumulation;

        return FixedMath.Clamp01(Context.DeltaTime * _rotationInterpoleSpeed * _rotationSpeed);
    }

    private void SetPositionTransformWorldPosition(Vector3d position)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !_positionTransform.TrySetWorldPosition(position),
            nameof(FixedTransform),
            "Position transform cannot represent the requested world position.");
    }

    private void SetRotationTransformWorldRotation(FixedQuaternion rotation)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !_rotationTransform.TrySetWorldPose(_rotationTransform.WorldPosition, rotation),
            nameof(FixedTransform),
            "Rotation transform cannot represent the requested world rotation.");
    }

    public void Deactivate()
    {
        if (!ReferenceEquals(Collider.Body, this))
            return;

        DiscardContinuousCollisionHandoff();
        Collider.DeactivateRuntimeRegistration();
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

    public void UpdateRotation(FixedQuaternion targetRotation, Fixed64 bufferInterpolation)
    {
        _rotationInterpoleSpeed = bufferInterpolation;
        _rotationSpeed = Agent.IsInteracting
            ? InteractionRotationSpeed
            : DefaultRotationSpeed;
        Rotation = targetRotation;
    }

    /// <summary>
    /// Transforms position from local space to world space.
    /// </summary>
    /// <param name="point"></param>
    /// <returns></returns>
    public Vector3d TransformPoint(Vector3d point)
    {
        return Position3d + Rotation * Vector3d.Multiply(Collider.ScaledSize, point);
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
        return Vector3d.Multiply(Vector3d.One / Collider.ScaledSize, rotated);
    }

    public void ResetPosition(Vector3d position = default, FixedQuaternion rotation = default)
    {
        ClearMotionForSleep();
        bool wasSleeping = _isSleeping;
        _isSleeping = false;
        _sleepFrameCount = 0;
        _normalForce = Vector3d.Zero;

        SetPosition2d(position.ToVector2d());
        HeightPos = position.Y;
        _lastPosition = position;
        _lastVisualPosition = _visualPosition = position;
        SetPositionTransformWorldPosition(position);
        Rotation = rotation;

        _visualRotation = rotation;
        SetRotationTransformWorldRotation(rotation);

        if (wasSleeping)
            RefreshPartitionAwakeState();
    }

}
