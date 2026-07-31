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

    /// <summary>
    /// Gets the ephemeral simulated-body slot, or <c>-1</c> for a static or
    /// unregistered body.
    /// </summary>
    public int DynamicId => _dynamicId;

    private BodyFreezeAxes3D _freezeAxes;

    /// <summary>
    /// Gets or sets the 3D translational and rotational degrees of freedom
    /// frozen for solver response, integration, and CCD.
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
            if (Active)
                RefreshInertiaTensor();
            RefreshPartitionMobility();
        }
    }

    /// <summary>
    /// Gets whether all translation axes are frozen.
    /// </summary>
    public bool IsPositionFullyFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (_freezeAxes & BodyFreezeAxes3D.Position) == BodyFreezeAxes3D.Position;
    }

    private BodyMotionType _motionType;

    /// <summary>
    /// Gets this body's explicit solver-controlled, host-controlled, or static
    /// runtime role.
    /// </summary>
    public BodyMotionType MotionType => _motionType;

    /// <summary>
    /// Gets whether the solver controls this body.
    /// </summary>
    public bool IsDynamic => _motionType == BodyMotionType.Dynamic;

    /// <summary>
    /// Gets whether this body is excluded from per-frame motion.
    /// </summary>
    public bool IsStatic => _motionType == BodyMotionType.Static;

    /// <summary>
    /// Gets whether the host controls this body's pose.
    /// </summary>
    public bool IsKinematic => _motionType == BodyMotionType.Kinematic;

    /// <summary>
    /// Changes this registered body's runtime role between fixed-step
    /// transactions.
    /// </summary>
    /// <remarks>
    /// The transition preserves body, collider, pair, joint, and host identity,
    /// but clears incompatible motion, sleep, CCD, and solver-cache state before
    /// repartitioning. Freeze axes remain independent of the selected role.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="motionType"/> is undefined.
    /// </exception>
    /// <exception cref="InvalidOperationException">
    /// The body is not currently registered, its context registration has been
    /// reset, or a simulation transaction or callback is active.
    /// </exception>
    public void SetMotionType(BodyMotionType motionType)
    {
        if (!PrepareMotionTypeTransition(motionType))
            return;

        CommitMotionTypeTransition(motionType);
    }

    internal bool PrepareMotionTypeTransition(BodyMotionType motionType)
    {
        motionType.ThrowIfInvalid(nameof(motionType));
        ThrowIfRuntimeRegistrationMissing();
        if (_motionType == motionType)
            return false;

        Context.ThrowIfFixedStepMutationNotAllowed();
        Collider.ValidateCurrentRuntimeTransform();
        if (motionType != BodyMotionType.Dynamic)
            PublishAuthoritativePose();

        return true;
    }

    internal void CommitMotionTypeTransition(BodyMotionType motionType)
    {
        ReconcileMotionTypeRegistration(motionType);

        ClearMotionForSleep();
        _isSleeping = false;
        _sleepFrameCount = 0;
        InvalidateContinuousCollisionTrajectory();
        ApplyFreezeConstraintsToMotion();
        RefreshInertiaTensor();
        RefreshPartitionMobility();
    }

    internal void ApplyLoadedMotionType(BodyMotionType motionType)
    {
        motionType.ThrowIfInvalid(nameof(motionType));
        if (_motionType == motionType)
            return;

        if (!Active)
        {
            _motionType = motionType;
            return;
        }

        ReconcileMotionTypeRegistration(motionType);
        InvalidateContinuousCollisionTrajectory();
    }

    internal void PreflightLoadedMotionType(BodyMotionType motionType)
    {
        motionType.ThrowIfInvalid(nameof(motionType));
        if (!Active)
            return;

        ThrowIfRuntimeRegistrationMissing();
        Context.ThrowIfFixedStepMutationNotAllowed();
        Collider.ValidateCurrentRuntimeTransform();
    }

    private void ThrowIfRuntimeRegistrationMissing()
    {
        SwiftThrowHelper.ThrowIfTrue(
            !Active,
            nameof(SolidBody),
            "Body runtime state cannot change before initialization or after deactivation.");
        SwiftThrowHelper.ThrowIfTrue(
            !Context.Physics.TryGetColliderById(Collider.Id, out LSCollider? registeredCollider)
                || !ReferenceEquals(registeredCollider, Collider),
            nameof(SolidBody),
            "Body runtime state cannot change after its registration has been reset or replaced.");
    }

    private void ReconcileMotionTypeRegistration(BodyMotionType motionType)
    {
        BodyMotionType previousMotionType = _motionType;
        Context.Physics.ClearWarmStartCachesForCollider(Collider);
        Context.Constraints3D.ClearSolverCachesForBody(this);
        Context.Physics.InvalidateContinuousCollisionStateForMotionTypeChange(this, DynamicId);

        _motionType = motionType;
        Context.Physics.RefreshBodyMotionTypeRegistration(this, previousMotionType);
        Context.Physics.RefreshColliderServiceRefreshRegistration(Collider);
    }

    internal void SetDynamicId(int dynamicId) => _dynamicId = dynamicId;

    private ContinuousCollisionMode _continuousCollisionMode = ContinuousCollisionMode.Inherit;
    private int _continuousCollisionFrameToken = int.MinValue;
    private readonly SwiftList<ContinuousCollisionMotionSegment3D> _continuousCollisionTrajectory =
        new(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations + 1);
    private Vector3d _continuousCollisionAngularVelocityStepStart;
    private bool _continuousCollisionHandoffPending;
    private int _continuousCollisionHandoffToken = int.MinValue;
    private Fixed64 _continuousCollisionHandoffRemainingTime;

    /// <summary>
    /// Selects the deterministic tunneling guard used when this body commits frame movement.
    /// Inherited values resolve through the cached top-parent body before falling back to context settings.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is not a declared continuous-collision mode.</exception>
    public ContinuousCollisionMode ContinuousCollisionMode
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionMode;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            value.ThrowIfInvalid(nameof(value));
            _continuousCollisionMode = value;
        }
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
    private readonly SwiftList<int> _rotationalContinuousCollisionCandidateIds = new(DefaultBodyHitBufferCapacity);
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
    /// Gets whether all rotation axes are frozen.
    /// </summary>
    public bool IsRotationFullyFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (_freezeAxes & BodyFreezeAxes3D.Rotation) == BodyFreezeAxes3D.Rotation;
    }

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

    private Fixed3x3 _inertiaTensor;
    private Fixed3x3 _worldInertiaTensor;
    private Fixed3x3 _inverseLocalInertiaTensor;
    private Fixed3x3 _inverseInertiaTensor;
    public Fixed3x3 InverseInertiaTensor => _inverseInertiaTensor;

    /// <summary>
    /// Gets whether solver-side response may translate this body.
    /// </summary>
    public bool CanTranslate => Active && _dynamicId >= 0 && IsDynamic && !IsPositionFullyFrozen && InverseMass > Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may rotate this body.
    /// </summary>
    public bool CanRotate => Active
        && _dynamicId >= 0
        && IsDynamic
        && !IsRotationFullyFrozen
        && _inverseInertiaTensor != Fixed3x3.Zero;

    internal bool HasSolverMobility => CanTranslate || CanRotate;

    /// <summary>
    /// Gets the inverse mass that should be used by collision response.
    /// Translation-frozen, static, and kinematic bodies expose their raw mass
    /// but contribute zero constrained inverse mass.
    /// </summary>
    public Fixed64 EffectiveInverseMass => CanTranslate ? InverseMass : Fixed64.Zero;

    /// <summary>
    /// Gets the inverse inertia tensor that should be used by collision response.
    /// Bodies that cannot rotate expose a zero tensor even when raw inertia is available.
    /// </summary>
    public Fixed3x3 EffectiveInverseInertiaTensor => CanRotate ? _inverseInertiaTensor : Fixed3x3.Zero;

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

    internal Fixed3x3 GetConstrainedInverseInertiaTensor() =>
        new(
            ApplyConstrainedInverseInertia(Vector3d.Right),
            ApplyConstrainedInverseInertia(Vector3d.Up),
            ApplyConstrainedInverseInertia(Vector3d.Forward));

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

    internal bool IsAwakeForCollision => HasSolverMobility && !IsSleeping;

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
    /// Called after authoritative position or rotation changes have been
    /// committed during simulation.
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
        BodyMotionType motionType = BodyMotionType.Dynamic)
    {
        motionType.ThrowIfInvalid(nameof(motionType));
        SwiftThrowHelper.ThrowIfArgument(
            Collider.Id >= 0
                || (Collider.HasHostBinding && !ReferenceEquals(Collider.Body, this)),
            nameof(Collider),
            "Body collider must be unregistered and free of another host binding before initialization.");
        FixedQuaternion normalizedRotation = startRotation.Normalized;
        Collider.PreflightBodyInitialization(this, startPosition, normalizedRotation);

        _motionType = motionType;
        Active = true;

        InvalidateContinuousCollisionTrajectory();
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
        _rotation = normalizedRotation;

        if (!IsKinematic)
        {
            _lastVisualPosition = _visualPosition = Position3d;
            _visualRotation = normalizedRotation;
            _lastVisualRotation = _visualRotation;
        }

        OnVisualize();

        _dynamicId = Context.Physics.AssimilateBody(this, motionType);
        Collider!.Initialize(this);
        RefreshMassPropertiesFromColliderShape();
        CheckGround(force: true);
    }


    public void LateSimulate()
    {
        Context.EnterSimulationPhase();
        try
        {
            LateSimulate(updateSleepState: true, updateColliderState: true);
        }
        finally
        {
            Context.ExitSimulationPhase();
        }
    }

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

            if (HasSolverMobility)
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
        EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        Vector3d startPosition = Position3d;
        FixedQuaternion startRotation = Rotation;
        Vector3d requestedPosition = _positionTransform.WorldPosition;
        FixedQuaternion requestedRotation = _rotationTransform.WorldRotation;
        Vector3d kinematicPosition = ContinuousCollisionFrameEnd;
        FixedQuaternion kinematicRotation = ContinuousCollisionFrameTargetRotation;
        if (startPosition == kinematicPosition && startRotation == kinematicRotation)
        {
            SetPositionTransformWorldPosition(kinematicPosition);
            SetRotationTransformWorldRotation(kinematicRotation);
            return;
        }

        Wake();

        Vector3d resolvedPosition = ProjectLinearEndpoint(startPosition, kinematicPosition);
        if (ShouldUseContinuousCollision(out _))
            _ = ContinuousCollisionSweepRange.ValidateEndpoint(startPosition, resolvedPosition, out _);
        FixedQuaternion resolvedRotation = kinematicRotation;
        if (!TryResolveKinematicRotationalContinuousCollision(
                startPosition,
                ref resolvedPosition,
                startRotation,
                ref resolvedRotation))
        {
            TryResolveKinematicContinuousCollision(startPosition, ref resolvedPosition);
        }

        if (resolvedPosition != requestedPosition)
            SetPositionTransformWorldPosition(resolvedPosition);
        if (resolvedRotation != requestedRotation)
            SetRotationTransformWorldRotation(resolvedRotation);

        SetPosition2d(resolvedPosition.ToVector2d());
        HeightPos = resolvedPosition.Y;
        SetVisualPosition(resolvedPosition);

        Rotation = resolvedRotation;
        StoreVisualRotation(resolvedRotation);
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

    private bool CanSleep => SleepEnabled && HasSolverMobility;


    private void SetTransformWorldPose(Vector3d position, FixedQuaternion rotation)
    {
        Vector3d originalLocalPosition = _positionTransform.LocalPosition;
        FixedQuaternion originalLocalRotation = _positionTransform.LocalRotation;
        SwiftThrowHelper.ThrowIfTrue(
            !_positionTransform.TrySetWorldPose(position, rotation),
            nameof(FixedTransform),
            "Host transform cannot represent the requested world pose.");

        try
        {
            if (Active)
            {
                SwiftThrowHelper.ThrowIfTrue(
                    !Collider.TryPrepareBodyPose(position, rotation),
                    nameof(position),
                    "The requested body pose produces collider geometry outside the representable coordinate domain.");
            }
        }
        catch
        {
            _positionTransform.LocalPosition = originalLocalPosition;
            _positionTransform.LocalRotation = originalLocalRotation;
            throw;
        }
    }

    public void Deactivate()
    {
        if (!ReferenceEquals(Collider.Body, this))
            return;

        DiscardContinuousCollisionHandoff();
        InvalidateContinuousCollisionTrajectory();
        Collider.DeactivateRuntimeRegistration();
        Context.Physics.DessimilateBody(this);
        _dynamicId = -1;
        Active = false;
    }

    public void UpdateRotation(FixedQuaternion targetRotation, Fixed64 bufferInterpolation)
    {
        FixedQuaternion normalizedRotation = targetRotation.Normalized;
        PreflightStaticPoseChange();
        bool preparedPose = PrepareExplicitBodyPose(
            Position3d,
            normalizedRotation,
            nameof(targetRotation),
            "The requested target rotation produces collider geometry outside the representable coordinate domain.");
        _rotationInterpoleSpeed = bufferInterpolation;
        _rotationSpeed = Agent.IsInteracting
            ? InteractionRotationSpeed
            : DefaultRotationSpeed;
        Rotation = normalizedRotation;
        PublishExplicitBodyPose(preparedPose);
    }

    /// <summary>
    /// Transforms a point from body-local space to world space using the host transform's world scale.
    /// </summary>
    /// <param name="point">The body-local point.</param>
    /// <returns>The corresponding world-space point.</returns>
    /// <exception cref="InvalidOperationException">
    /// The host transform's world scale or the final world-space point is not representable.
    /// </exception>
    public Vector3d TransformPoint(Vector3d point)
    {
        bool transformed = TryTransformPoint(point, out Vector3d result);
        SwiftThrowHelper.ThrowIfTrue(
            !transformed,
            nameof(Agent.Transform),
            "Cannot transform the point because the host world scale or final world-space point is not representable.");
        return result;
    }

    /// <summary>
    /// Attempts to transform a point from body-local space to world space using the host transform's world scale.
    /// </summary>
    /// <param name="point">The body-local point.</param>
    /// <param name="result">The world-space point on success; otherwise zero.</param>
    /// <returns><see langword="true"/> when the host world scale and final point are representable.</returns>
    public bool TryTransformPoint(Vector3d point, out Vector3d result)
    {
        if (!Agent.Transform.TryGetLossyScale(out Vector3d scale))
        {
            result = Vector3d.Zero;
            return false;
        }

        return Rotation.TryTransformScaledPoint(Position3d, point, scale, out result);
    }

    /// <summary>
    /// Transforms a point from world space to body-local space using the host transform's world scale.
    /// </summary>
    /// <param name="point">The world-space point.</param>
    /// <returns>The corresponding body-local point.</returns>
    /// <exception cref="InvalidOperationException">
    /// The host transform's world scale is unavailable or singular, or the final body-local point is not representable.
    /// </exception>
    public Vector3d InverseTransformPoint(Vector3d point)
    {
        bool transformed = TryInverseTransformPoint(point, out Vector3d result);
        SwiftThrowHelper.ThrowIfTrue(
            !transformed,
            nameof(Agent.Transform),
            "Cannot inverse-transform the point because the host world scale is unavailable or singular, or the final body-local point is not representable.");
        return result;
    }

    /// <summary>
    /// Attempts to transform a point from world space to body-local space using the host transform's world scale.
    /// </summary>
    /// <param name="point">The world-space point.</param>
    /// <param name="result">The body-local point on success; otherwise zero.</param>
    /// <returns>
    /// <see langword="true"/> when the host world scale is available and nonsingular and the final point is representable.
    /// </returns>
    public bool TryInverseTransformPoint(Vector3d point, out Vector3d result)
    {
        if (!Agent.Transform.TryGetLossyScale(out Vector3d scale))
        {
            result = Vector3d.Zero;
            return false;
        }

        return Rotation.TryInverseTransformScaledPoint(Position3d, point, scale, out result);
    }

    public void ResetPosition(Vector3d position = default, FixedQuaternion rotation = default)
    {
        FixedQuaternion normalizedRotation = rotation.Normalized;
        PreflightResetPoseChange();
        SetTransformWorldPose(position, normalizedRotation);
        InvalidateContinuousCollisionTrajectory();
        ClearMotionForSleep();
        bool wasSleeping = _isSleeping;
        _isSleeping = false;
        _sleepFrameCount = 0;
        _normalForce = Vector3d.Zero;

        SetPosition2d(position.ToVector2d());
        HeightPos = position.Y;
        _lastPosition = position;
        _lastVisualPosition = _visualPosition = position;
        Rotation = normalizedRotation;

        _visualRotation = normalizedRotation;

        PublishExplicitBodyPose(Active);
        if (wasSleeping)
            RefreshPartitionAwakeState();
    }

}
