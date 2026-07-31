//=======================================================================
// SolidBody2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// First-class pure 2D deterministic body state.
/// </summary>
public sealed partial class SolidBody2D : IRecordable
{
    private Vector2d _position;
    private Fixed64 _rotation;
    private Vector2d _linearVelocity;
    private Vector2d _linearAccelerationStore;
    private Vector2d _deltaAcceleration;
    private Fixed64 _linearSpeed;
    private Fixed64 _mass;
    private Vector2d _localCenterOfMassOffset;
    private bool _centerOfMassOffsetExplicit;
    private Fixed64 _momentOfInertia;
    private Fixed64 _inverseMomentOfInertia;
    private Fixed64 _angularVelocity;
    private Fixed64 _angularAccelerationStore;
    private Fixed64 _deltaAngularAcceleration;
    private Fixed64 _angularSpeed;
    private bool _isSleeping;
    private BodyMotionType _motionType;
    private int _sleepFrameCount;
    private bool _sleepEnabled = true;
    private int _sleepFrameThreshold = 16;
    private Fixed64 _sleepLinearSpeedThreshold = (Fixed64)0.001f;
    private Fixed64 _sleepAngularSpeedThreshold = (Fixed64)0.001f;
    private ContinuousCollisionMode _continuousCollisionMode = ContinuousCollisionMode.Inherit;
    private int _continuousCollisionFrameToken = int.MinValue;
    private readonly SwiftList<ContinuousCollisionMotionSegment2D> _continuousCollisionTrajectory =
        new(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations + 1);
    private bool _continuousCollisionHandoffPending;
    private int _continuousCollisionHandoffToken = int.MinValue;
    private Fixed64 _continuousCollisionHandoffRemainingTime;
    private readonly SwiftList<Physics2DHit> _continuousCollisionHits = new();
    private readonly SwiftList<int> _rotationalContinuousCollisionCandidateIds = new();
    private readonly SwiftList<PhysicsMixedHit> _continuousMixedCollisionHits = new();

    public SolidBody2D(IMatterAgent agent, LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        Agent = agent;
        Context = agent.Context;
        Collider = collider;
    }

    public IMatterAgent Agent { get; }

    public GravitasWorldContext Context { get; }

    public LSCollider2D Collider { get; }

    /// <summary>
    /// Gets the ephemeral simulated-body slot, or <c>-1</c> for a static or
    /// unregistered body.
    /// </summary>
    public int DynamicId { get; internal set; } = -1;

    public bool Active { get; private set; }

    private BodyFreezeAxes2D _freezeAxes;

    /// <summary>
    /// Gets or sets the planar translational and yaw rotational degrees of
    /// freedom frozen for pure 2D solver response, integration, CCD, and
    /// rotational CCD.
    /// </summary>
    public BodyFreezeAxes2D FreezeAxes
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _freezeAxes;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                (value & ~BodyFreezeAxes2D.All) != BodyFreezeAxes2D.None,
                nameof(value),
                "Unsupported 2D freeze axis bits.");

            if (_freezeAxes == value)
                return;

            _freezeAxes = value;
            ApplyFreezeConstraintsToMotion();
            RefreshPartitionMobility();
        }
    }

    /// <summary>
    /// Gets whether both planar translation axes are frozen.
    /// </summary>
    public bool IsPositionFullyFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (_freezeAxes & BodyFreezeAxes2D.Position) == BodyFreezeAxes2D.Position;
    }

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
    /// <exception cref="System.ArgumentOutOfRangeException">
    /// <paramref name="motionType"/> is undefined.
    /// </exception>
    /// <exception cref="System.InvalidOperationException">
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

        ClearMotionForMotionTypeTransition();
        _isSleeping = false;
        _sleepFrameCount = 0;
        InvalidateContinuousCollisionFrame();
        ApplyFreezeConstraintsToMotion();
        RefreshMassPropertiesFromColliderShape();
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
        InvalidateContinuousCollisionFrame();
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
            nameof(SolidBody2D),
            "2D body runtime state cannot change before initialization or after deactivation.");
        SwiftThrowHelper.ThrowIfTrue(
            !Context.Physics2D.TryGetColliderById(Collider.Id, out LSCollider2D? registeredCollider)
                || !ReferenceEquals(registeredCollider, Collider),
            nameof(SolidBody2D),
            "2D body runtime state cannot change after its registration has been reset or replaced.");
    }

    private void ReconcileMotionTypeRegistration(BodyMotionType motionType)
    {
        BodyMotionType previousMotionType = _motionType;
        Context.Physics2D.ClearWarmStartCachesForCollider(Collider);
        Context.Constraints2D.ClearSolverCachesForBody(this);
        Context.Physics2D.InvalidateContinuousCollisionStateForMotionTypeChange(this, DynamicId);

        _motionType = motionType;
        Context.Physics2D.RefreshBodyMotionTypeRegistration(this, previousMotionType);
        Context.Physics2D.RefreshColliderServiceRefreshRegistration(Collider);
    }

    private void ClearMotionForMotionTypeTransition()
    {
        _linearVelocity = Vector2d.Zero;
        _linearAccelerationStore = Vector2d.Zero;
        _deltaAcceleration = Vector2d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
    }

    /// <summary>
    /// Selects the deterministic tunneling guard used when this pure 2D body commits frame movement.
    /// </summary>
    /// <exception cref="System.ArgumentOutOfRangeException">The value is not a declared continuous-collision mode.</exception>
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
    /// Gets the number of continuous-collision impacts consumed by the most recent 2D late simulation step.
    /// </summary>
    public int LastContinuousCollisionToiIterationCount { get; private set; }

    /// <summary>
    /// Gets whether the most recent 2D late simulation step reached the configured continuous-collision TOI iteration limit.
    /// </summary>
    public bool LastContinuousCollisionToiIterationLimitReached { get; private set; }

    /// <summary>
    /// Gets whether pure 2D yaw rotation is frozen.
    /// </summary>
    public bool IsRotationFullyFrozen
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (_freezeAxes & BodyFreezeAxes2D.Rotation) == BodyFreezeAxes2D.Rotation;
    }

    public Fixed64 Mass
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _mass;
        set
        {
            if (_mass == value)
                return;

            _mass = value;
            RefreshMassPropertiesFromColliderShape();
        }
    }

    public Fixed64 InverseMass => _mass > Fixed64.Zero ? Fixed64.One / _mass : Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may translate this pure 2D body.
    /// </summary>
    public bool CanTranslate => Active && DynamicId >= 0 && IsDynamic && !IsPositionFullyFrozen && InverseMass > Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may rotate this pure 2D body around its yaw axis.
    /// </summary>
    public bool CanRotate => Active
        && DynamicId >= 0
        && IsDynamic
        && !IsRotationFullyFrozen
        && _inverseMomentOfInertia > Fixed64.Zero;

    internal bool HasSolverMobility => CanTranslate || CanRotate;

    /// <summary>
    /// Gets the inverse mass that should be used by pure 2D and mixed response.
    /// </summary>
    public Fixed64 EffectiveInverseMass => CanTranslate ? InverseMass : Fixed64.Zero;

    /// <summary>
    /// Gets the inverse scalar moment that should be used by pure 2D angular response.
    /// </summary>
    public Fixed64 EffectiveInverseMomentOfInertia => CanRotate ? _inverseMomentOfInertia : Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector2d ProjectLinearMotion(Vector2d value)
    {
        if (value == Vector2d.Zero || IsPositionFullyFrozen)
            return Vector2d.Zero;

        Fixed64 x = (_freezeAxes & BodyFreezeAxes2D.PositionX) == BodyFreezeAxes2D.PositionX ? Fixed64.Zero : value.X;
        Fixed64 y = (_freezeAxes & BodyFreezeAxes2D.PositionY) == BodyFreezeAxes2D.PositionY ? Fixed64.Zero : value.Y;
        return new Vector2d(x, y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector2d ProjectLinearEndpoint(Vector2d start, Vector2d end)
    {
        Fixed64 x = (_freezeAxes & BodyFreezeAxes2D.PositionX) == BodyFreezeAxes2D.PositionX ? start.X : end.X;
        Fixed64 y = (_freezeAxes & BodyFreezeAxes2D.PositionY) == BodyFreezeAxes2D.PositionY ? start.Y : end.Y;
        return new Vector2d(x, y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 GetConstrainedInverseMass(Vector2d axis)
    {
        if (!CanTranslate || axis == Vector2d.Zero)
            return Fixed64.Zero;

        Fixed64 axisMagnitudeSquared = axis.MagnitudeSquared;
        if (axisMagnitudeSquared <= Fixed64.Epsilon)
            return Fixed64.Zero;

        Vector2d allowedAxis = ProjectLinearMotion(axis);
        Fixed64 allowedScale = Vector2d.Dot(allowedAxis, axis) / axisMagnitudeSquared;
        return allowedScale > Fixed64.Zero ? InverseMass * allowedScale : Fixed64.Zero;
    }

    public Fixed64 MomentOfInertia => _momentOfInertia;

    public Fixed64 InverseMomentOfInertia => _inverseMomentOfInertia;

    public Fixed64 AngularVelocity => _angularVelocity;

    public Fixed64 AngularAcceleration => _angularAccelerationStore;

    public Fixed64 AngularSpeed => _angularSpeed;

    public Vector2d Position
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _position;
    }

    public Fixed64 Rotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _rotation;
    }

    public Vector2d LinearVelocity => _linearVelocity;

    public Fixed64 LinearSpeed => _linearSpeed;

    public Vector2d Gravity { get; set; } = Vector2d.Zero;

    private Fixed64 _gravityScale = Fixed64.One;

    /// <summary>
    /// Multiplies this body's planar gravity vector. Zero disables body-authored gravity acceleration.
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
                "2D gravity scale cannot be negative.");
            _gravityScale = value;
        }
    }

    /// <summary>
    /// Enables deterministic sleep evaluation for this body. Disabling sleep wakes a sleeping body immediately.
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
                "2D sleep linear speed threshold cannot be negative.");
            _sleepLinearSpeedThreshold = value;
        }
    }

    /// <summary>
    /// Angular speed at or below which the body can count toward sleeping.
    /// </summary>
    public Fixed64 SleepAngularSpeedThreshold
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _sleepAngularSpeedThreshold;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value < Fixed64.Zero,
                nameof(value),
                "2D angular sleep threshold cannot be negative.");
            _sleepAngularSpeedThreshold = value;
        }
    }

    public bool IsSleeping => _isSleeping;

    internal bool IsAwakeForCollision => HasSolverMobility && !IsSleeping;

    public void Initialize(
        Vector2d position,
        Fixed64 rotation = default,
        BodyMotionType motionType = BodyMotionType.Dynamic)
    {
        motionType.ThrowIfInvalid(nameof(motionType));
        SwiftThrowHelper.ThrowIfTrue(Active, nameof(SolidBody2D), "2D body is already initialized.");
        SwiftThrowHelper.ThrowIfArgument(
            Collider.Id >= 0
                || (Collider.HasHostBinding && !ReferenceEquals(Collider.Body, this)),
            nameof(Collider),
            "Body collider must be unregistered and free of another host binding before initialization.");
        Fixed64 canonicalRotation = CanonicalizeRotation(rotation);
        Collider.PreflightBodyInitialization(this, position, canonicalRotation);

        _position = position;
        _rotation = canonicalRotation;
        _linearVelocity = Vector2d.Zero;
        _linearAccelerationStore = Vector2d.Zero;
        _deltaAcceleration = Vector2d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
        _isSleeping = false;
        _motionType = motionType;
        _sleepFrameCount = 0;
        InvalidateContinuousCollisionFrame();
        ResetGroundingForInitialize(position);
        Active = true;
        Collider.Initialize(this);
        RefreshMassPropertiesFromColliderShape();
        Context.Physics2D.AssimilateBody(this, motionType);
        CheckGroundForSimulation();
    }

    /// <summary>
    /// Gets a world-space point from the authoritative 2D body pose and committed collider scale.
    /// </summary>
    public Vector2d GetWorldPoint(Vector2d point)
    {
        bool transformed = TryGetWorldPoint(point, out Vector2d result);
        SwiftThrowHelper.ThrowIfTrue(
            !transformed,
            nameof(Collider),
            "Cannot get the world point because the 2D collider has no committed scale or the final coordinate is not representable.");
        return result;
    }

    /// <summary>
    /// Attempts to get a world-space point from the authoritative 2D body pose and committed collider scale.
    /// </summary>
    public bool TryGetWorldPoint(Vector2d point, out Vector2d result)
    {
        if (!Collider.TryGetCommittedOwnerScale(out Vector2d scale))
        {
            result = Vector2d.Zero;
            return false;
        }

        return Vector2d.TryTransformScaledPoint(
            Position,
            point,
            scale,
            Rotation,
            out result);
    }

    /// <summary>
    /// Gets a body-local point from the authoritative 2D body pose and committed collider scale.
    /// </summary>
    public Vector2d GetLocalPoint(Vector2d point)
    {
        bool transformed = TryGetLocalPoint(point, out Vector2d result);
        SwiftThrowHelper.ThrowIfTrue(
            !transformed,
            nameof(Collider),
            "Cannot get the local point because the 2D collider has no committed scale, its scale is singular, or the final coordinate is not representable.");
        return result;
    }

    /// <summary>
    /// Attempts to get a body-local point from the authoritative 2D body pose and committed collider scale.
    /// </summary>
    public bool TryGetLocalPoint(Vector2d point, out Vector2d result)
    {
        if (!Collider.TryGetCommittedOwnerScale(out Vector2d scale))
        {
            result = Vector2d.Zero;
            return false;
        }

        return Vector2d.TryInverseTransformScaledPoint(
            Position,
            point,
            scale,
            Rotation,
            out result);
    }

    /// <summary>
    /// Resets authoritative 2D pose and clears accumulated linear/angular motion for deterministic fixture reuse.
    /// </summary>
    /// <param name="position">The new X/Z-plane position.</param>
    /// <param name="rotation">The new yaw rotation in radians.</param>
    public void ResetPosition(Vector2d position = default, Fixed64 rotation = default)
    {
        PreflightStaticPoseChange();
        Fixed64 canonicalRotation = CanonicalizeRotation(rotation);
        SwiftThrowHelper.ThrowIfTrue(
            !Collider.TryPrepareBodyPose(position, canonicalRotation),
            nameof(position),
            "The requested 2D body pose produces collider geometry outside the representable coordinate domain.");
        _linearVelocity = Vector2d.Zero;
        _linearAccelerationStore = Vector2d.Zero;
        _deltaAcceleration = Vector2d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
        bool wasSleeping = _isSleeping;
        _isSleeping = false;
        _sleepFrameCount = 0;
        _position = position;
        _rotation = canonicalRotation;
        InvalidateContinuousCollisionFrame();
        ResetGroundingForInitialize(position);

        if (!Active)
            return;

        Collider.PublishPreparedExplicitBodyPose();
        RefreshStaticColliderAfterExplicitPoseChange();
        if (wasSleeping)
            Context.Collisions2D.RefreshPartitionAwakeState(Collider);
    }

    public void Sleep()
    {
        if (!CanSleep)
            return;

        _isSleeping = true;
        _sleepFrameCount = SleepFrameThreshold;
        _linearVelocity = Vector2d.Zero;
        _linearAccelerationStore = Vector2d.Zero;
        _deltaAcceleration = Vector2d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
        Context.Collisions2D.RefreshPartitionAwakeState(Collider);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Wake()
    {
        bool wasSleeping = _isSleeping;
        _sleepFrameCount = 0;
        _isSleeping = false;
        if (Active && wasSleeping)
            Context.Collisions2D.RefreshPartitionAwakeState(Collider);
    }

    internal void WakeFromCollision()
    {
        if (!_isSleeping)
            return;

        _sleepFrameCount = 0;
        _isSleeping = false;
        Context.Collisions2D.RefreshPartitionAwakeState(Collider);
    }

    internal void LateSimulate() => LateSimulate(updateSleepState: true, updateColliderState: true);

    internal void LateSimulate(bool updateSleepState, bool updateColliderState)
    {
        if (!Active)
            return;

        CaptureGroundedStepState();
        try
        {
            LastContinuousCollisionToiIterationCount = 0;
            LastContinuousCollisionToiIterationLimitReached = false;

            if (TryConsumeContinuousCollisionHandoff(updateSleepState, updateColliderState))
                return;

            EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);

            if (IsKinematic)
                UpdateKinematicPositionAndRotation(updateColliderState);

            if (!HasSolverMobility)
            {
                _linearAccelerationStore = Vector2d.Zero;
                _deltaAcceleration = Vector2d.Zero;
                _angularAccelerationStore = Fixed64.Zero;
                _deltaAngularAcceleration = Fixed64.Zero;
                return;
            }

            if (_isSleeping)
                return;

            Fixed64 startRotation = _rotation;
            Fixed64 proposedRotation = startRotation + _angularVelocity * Context.DeltaTime;

            Vector2d startPosition = _position;
            Vector2d proposedPosition = startPosition + _linearVelocity * Context.DeltaTime;
            if (ShouldUseRotationalContinuousCollisionArbiter(
                    startPosition,
                    proposedPosition,
                    startRotation,
                    proposedRotation,
                    forceContinuous: false))
            {
                TryResolveRotationalContinuousCollision(
                    startPosition,
                    ref proposedPosition,
                    startRotation,
                    ref proposedRotation);
            }
            else
            {
                TryResolveContinuousCollision(startPosition, ref proposedPosition);
            }
            proposedPosition = ProjectLinearEndpoint(startPosition, proposedPosition);
            _position = proposedPosition;
            _rotation = CanonicalizeRotation(proposedRotation);
            if (updateColliderState)
                Collider.Rebuild();

            if (updateSleepState)
                UpdateSleepState();
        }
        finally
        {
            CompleteGroundedStepState();
        }
    }

    internal void UpdateSleepStateAfterPhysicsStep()
    {
        if (!_isSleeping)
            UpdateSleepState();
    }

    internal void OnVisualize()
    {
        if (IsKinematic)
            return;

        SetHostWorldPose(Agent.Transform, _position, _rotation);
    }

    private void UpdateKinematicPositionAndRotation(bool updateColliderState)
    {
        Vector2d startPosition = _position;
        Fixed64 startRotation = _rotation;
        FixedTransform transform = Agent.Transform;
        Vector2d requestedHostPosition = transform.WorldPositionXZ;
        Fixed64 requestedHostRotation = CanonicalizeRotation(
            transform.WorldRotationXZRadians);
        Vector2d kinematicPosition = ContinuousCollisionFrameEnd;
        Fixed64 kinematicRotation = ContinuousCollisionFrameTargetRotation;
        if (startPosition == kinematicPosition && startRotation == kinematicRotation)
        {
            SetHostWorldPose(transform, kinematicPosition, kinematicRotation);
            return;
        }

        Wake();
        Vector2d resolvedPosition = kinematicPosition;
        Fixed64 resolvedRotation = kinematicRotation;
        if (ShouldUseRotationalContinuousCollisionArbiter(
                startPosition,
                resolvedPosition,
                startRotation,
                resolvedRotation,
                forceContinuous: false))
        {
            TryResolveKinematicRotationalContinuousCollision(
                startPosition,
                ref resolvedPosition,
                startRotation,
                ref resolvedRotation);
        }
        else
        {
            TryResolveKinematicContinuousCollision(startPosition, ref resolvedPosition);
        }

        resolvedRotation = CanonicalizeRotation(resolvedRotation);
        if (resolvedPosition != requestedHostPosition
            || resolvedRotation != requestedHostRotation)
            SetHostWorldPose(transform, resolvedPosition, resolvedRotation);

        _position = resolvedPosition;
        _rotation = resolvedRotation;
        if (updateColliderState)
            Collider.Rebuild();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 CanonicalizeRotation(Fixed64 rotation) =>
        PlanarRotation.Canonicalize(rotation);

    private static void SetHostWorldPose(
        FixedTransform transform,
        Vector2d position,
        Fixed64 rotation)
    {
        Vector3d currentWorldPosition = transform.WorldPosition;
        Vector3d worldPosition = new(position.X, currentWorldPosition.Y, position.Y);
        FixedQuaternion worldRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation);
        SwiftThrowHelper.ThrowIfTrue(
            !transform.TrySetWorldPose(worldPosition, worldRotation),
            nameof(FixedTransform),
            "Host transform cannot represent the requested world-space 2D pose.");
    }

    private void PublishAuthoritativePose() =>
        SetHostWorldPose(Agent.Transform, _position, _rotation);

    /// <summary>
    /// Removes this body and its collider from the pure 2D runtime service.
    /// </summary>
    public void Deactivate()
    {
        if (!ReferenceEquals(Collider.Body, this))
            return;

        DiscardContinuousCollisionHandoff();
        InvalidateContinuousCollisionFrame();
        Context.Physics2D.DessimilateBody(this);
        Active = false;
        DynamicId = -1;
        Collider.IsActive = false;
        Collider.ClearPhysicsState();
        Collider.ClearBindingState();
    }
}
