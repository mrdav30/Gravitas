//=======================================================================
// StiffBody2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// First-class pure 2D deterministic body state.
/// </summary>
public sealed partial class StiffBody2D : IRecordable
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
    private bool _isDynamic;
    private int _sleepFrameCount;
    private Fixed64 _sleepAngularSpeedThreshold = (Fixed64)0.001f;
    private ContinuousCollisionMode _continuousCollisionMode = ContinuousCollisionMode.Inherit;
    private int _continuousCollisionFrameToken = int.MinValue;
    private Vector2d _continuousCollisionFrameStart;
    private Vector2d _continuousCollisionFrameDisplacement;
    private Fixed64 _continuousCollisionFrameRotation;
    private bool _continuousCollisionHandoffPending;
    private int _continuousCollisionHandoffToken = int.MinValue;
    private Fixed64 _continuousCollisionHandoffRemainingTime;
    private readonly SwiftList<Physics2DHit> _continuousCollisionHits = new();
    private readonly SwiftList<PhysicsMixedHit> _continuousMixedCollisionHits = new();

    public StiffBody2D(IMatterAgent agent, LSCollider2D collider)
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

    public int DynamicId { get; internal set; } = -1;

    public bool Active { get; private set; }

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

    /// <summary>
    /// Selects the deterministic tunneling guard used when this pure 2D body commits frame movement.
    /// </summary>
    public ContinuousCollisionMode ContinuousCollisionMode
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionMode;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _continuousCollisionMode = value;
    }

    /// <summary>
    /// Gets the number of continuous-collision impacts consumed by the most recent 2D late simulation step.
    /// </summary>
    public int LastContinuousCollisionToiIterationCount { get; private set; }

    /// <summary>
    /// Gets whether the most recent 2D late simulation step reached the configured continuous-collision TOI iteration limit.
    /// </summary>
    public bool LastContinuousCollisionToiIterationLimitReached { get; private set; }

    public bool PreventAngularForces;

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
    public bool CanTranslate => Active && _isDynamic && !Immovable && !IsKinematic && InverseMass > Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may rotate this pure 2D body around its yaw axis.
    /// </summary>
    public bool CanRotate => CanTranslate && !PreventAngularForces && _inverseMomentOfInertia > Fixed64.Zero;

    /// <summary>
    /// Gets the inverse mass that should be used by pure 2D and mixed response.
    /// </summary>
    public Fixed64 EffectiveInverseMass => CanTranslate ? InverseMass : Fixed64.Zero;

    /// <summary>
    /// Gets the inverse scalar moment that should be used by pure 2D angular response.
    /// </summary>
    public Fixed64 EffectiveInverseMomentOfInertia => CanRotate ? _inverseMomentOfInertia : Fixed64.Zero;

    /// <summary>
    /// Gets or sets the authoritative body-local center-of-mass offset in the X/Z simulation plane.
    /// </summary>
    public Vector2d LocalCenterOfMassOffset
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
            RefreshMassPropertiesFromColliderShape();
        }
    }

    /// <summary>
    /// Gets the authoritative world-space center of mass in the X/Z simulation plane.
    /// </summary>
    public Vector2d WorldCenterOfMass =>
        _position + ClampNearZero(Vector2d.Rotate(_localCenterOfMassOffset, _rotation));

    public Fixed64 MomentOfInertia => _momentOfInertia;

    public Fixed64 InverseMomentOfInertia => _inverseMomentOfInertia;

    public Fixed64 AngularVelocity => _angularVelocity;

    public Fixed64 AngularAcceleration => _angularAccelerationStore;

    public Fixed64 AngularSpeed => _angularSpeed;

    /// <summary>
    /// Clears an explicit center-of-mass override and derives the offset from the bound collider again.
    /// </summary>
    public void ResetCenterOfMassFromCollider()
    {
        _centerOfMassOffsetExplicit = false;
        RefreshMassPropertiesFromColliderShape();
    }

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

    public Fixed64 RestitutionCoefficient { get; set; } = Fixed64.Half;

    public Fixed64 FrictionCoefficient { get; set; } = Fixed64.One;

    public Vector2d Gravity { get; set; } = Vector2d.Zero;

    public bool SleepEnabled { get; set; } = true;

    public int SleepFrameThreshold { get; set; } = 16;

    public Fixed64 SleepLinearSpeedThreshold { get; set; } = (Fixed64)0.001f;

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

    internal bool IsAwakeForCollision => Active && !Immovable && !IsSleeping;

    public void Initialize(Vector2d position, Fixed64 rotation = default, bool isDynamic = true)
    {
        SwiftThrowHelper.ThrowIfTrue(Active, nameof(StiffBody2D), "2D body is already initialized.");

        _position = position;
        _rotation = rotation;
        _linearVelocity = Vector2d.Zero;
        _linearAccelerationStore = Vector2d.Zero;
        _deltaAcceleration = Vector2d.Zero;
        _linearSpeed = Fixed64.Zero;
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
        _isSleeping = false;
        _isDynamic = isDynamic;
        _sleepFrameCount = 0;
        Active = true;
        Collider.Initialize(this);
        RefreshMassPropertiesFromColliderShape();
        Context.Physics2D.AssimilateBody(this, isDynamic);
    }

    /// <summary>
    /// Resets authoritative 2D pose and clears accumulated linear/angular motion for deterministic fixture reuse.
    /// </summary>
    /// <param name="position">The new X/Z-plane position.</param>
    /// <param name="rotation">The new yaw rotation in radians.</param>
    public void ResetPosition(Vector2d position = default, Fixed64 rotation = default)
    {
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
        _rotation = rotation;

        if (!Active)
            return;

        Collider.Rebuild();
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
        if (Active)
            Context.Collisions2D.RefreshPartitionAwakeState(Collider);
    }

    internal void LateSimulate() => LateSimulate(updateSleepState: true, updateColliderState: true);

    internal void LateSimulate(bool updateSleepState, bool updateColliderState)
    {
        if (!Active)
            return;

        LastContinuousCollisionToiIterationCount = 0;
        LastContinuousCollisionToiIterationLimitReached = false;

        if (TryConsumeContinuousCollisionHandoff(updateSleepState, updateColliderState))
            return;

        if (IsKinematic)
            UpdateKinematicPositionAndRotation(updateColliderState);

        if (!CanTranslate)
        {
            _linearAccelerationStore = Vector2d.Zero;
            _deltaAcceleration = Vector2d.Zero;
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
            return;
        }

        if (_isSleeping)
            return;

        _linearAccelerationStore = _deltaAcceleration + Gravity;
        _deltaAcceleration = Vector2d.Zero;
        _linearVelocity += _linearAccelerationStore * Context.DeltaTime;
        _linearAccelerationStore = Vector2d.Zero;
        RefreshLinearSpeed();

        Fixed64 startRotation = _rotation;
        Fixed64 proposedRotation = startRotation;
        if (CanRotate)
        {
            _angularAccelerationStore = _deltaAngularAcceleration;
            _deltaAngularAcceleration = Fixed64.Zero;
            _angularVelocity += _angularAccelerationStore * Context.DeltaTime;
            proposedRotation += _angularVelocity * Context.DeltaTime;
            RefreshAngularSpeed();
        }
        else
        {
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
        }

        Vector2d startPosition = _position;
        Vector2d proposedPosition = startPosition + _linearVelocity * Context.DeltaTime;
        TryResolveContinuousCollision(startPosition, ref proposedPosition);
        TryResolveRotationalContinuousCollision(startPosition, ref proposedPosition, startRotation, ref proposedRotation);
        _position = proposedPosition;
        _rotation = proposedRotation;
        if (updateColliderState)
            Collider.Rebuild();

        if (updateSleepState)
            UpdateSleepState();
    }

    internal void UpdateSleepStateAfterPhysicsStep()
    {
        if (Active && !_isSleeping)
            UpdateSleepState();
    }

    internal void OnVisualize()
    {
        if (!Active || IsKinematic)
            return;

        FixedTransform transform = Agent.Transform;
        Vector3d currentPosition = transform.Position;
        transform.Position = new Vector3d(_position.X, currentPosition.Y, _position.Y);
        transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            FixedMath.RadToDeg(_rotation),
            Fixed64.Zero);
    }

    private void UpdateKinematicPositionAndRotation(bool updateColliderState)
    {
        Vector2d startPosition = _position;
        Fixed64 startRotation = _rotation;
        Vector2d kinematicPosition = Agent.Transform.Position.ToVector2d();
        Fixed64 kinematicRotation = FixedMath.DegToRad(Agent.Transform.EulerAngles.Y);
        if (startPosition == kinematicPosition && startRotation == kinematicRotation)
            return;

        Wake();
        Vector2d resolvedPosition = kinematicPosition;
        Fixed64 resolvedRotation = kinematicRotation;
        CaptureKinematicContinuousCollisionFrame(startPosition, kinematicPosition, startRotation);
        TryResolveKinematicContinuousCollision(startPosition, ref resolvedPosition);
        TryResolveKinematicRotationalContinuousCollision(startPosition, ref resolvedPosition, startRotation, ref resolvedRotation);

        if (resolvedPosition != kinematicPosition)
        {
            Vector3d hostPosition = Agent.Transform.Position;
            Agent.Transform.Position = new Vector3d(resolvedPosition.X, hostPosition.Y, resolvedPosition.Y);
        }

        if (resolvedRotation != kinematicRotation)
        {
            Agent.Transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                FixedMath.RadToDeg(resolvedRotation),
                Fixed64.Zero);
        }

        _position = resolvedPosition;
        _rotation = resolvedRotation;
        if (updateColliderState)
            Collider.Rebuild();
    }

    /// <summary>
    /// Removes this body and its collider from the pure 2D runtime service.
    /// </summary>
    public void Deactivate()
    {
        if (!Active)
            return;

        Context.Physics2D.DessimilateBody(this);
        Active = false;
        _isDynamic = false;
        DynamicId = -1;
        Collider.IsActive = false;
        Collider.ClearPhysicsState();
        Collider.ClearBindingState();
    }
}
