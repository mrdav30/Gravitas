using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// First-class pure 2D deterministic body state.
/// </summary>
public sealed class StiffBody2D : IRecordable
{
    private Vector2d _position;
    private Fixed64 _rotation;
    private Vector2d _linearVelocity;
    private Vector2d _linearAccelerationStore;
    private Vector2d _deltaAcceleration;
    private Fixed64 _linearSpeed;
    private bool _isSleeping;
    private bool _isDynamic;
    private int _sleepFrameCount;
    private ContinuousCollisionMode _continuousCollisionMode = ContinuousCollisionMode.Inherit;
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

    public bool Immovable { get; set; }

    public bool IsKinematic { get; set; }

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

    public Fixed64 Mass { get; set; }

    public Fixed64 InverseMass => Mass > Fixed64.Zero ? Fixed64.One / Mass : Fixed64.Zero;

    public bool CanMove => Active && _isDynamic && !Immovable && !IsKinematic && InverseMass > Fixed64.Zero;

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
        _isSleeping = false;
        _isDynamic = isDynamic;
        _sleepFrameCount = 0;
        Active = true;
        Collider.Initialize(this);
        Context.Physics2D.AssimilateBody(this, isDynamic);
    }

    public void AddForce(Vector2d force)
    {
        if (force != Vector2d.Zero)
            Wake();

        _deltaAcceleration += force * InverseMass;
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
        if (_rotation != rotation)
            Wake();

        _rotation = rotation;
        Collider.Rebuild();
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

    internal void LateSimulate()
    {
        if (!Active)
            return;

        if (IsKinematic)
            UpdateKinematicPositionAndRotation();

        if (!CanMove)
        {
            _linearAccelerationStore = Vector2d.Zero;
            _deltaAcceleration = Vector2d.Zero;
            return;
        }

        if (_isSleeping)
            return;

        _linearAccelerationStore = _deltaAcceleration + Gravity;
        _deltaAcceleration = Vector2d.Zero;
        _linearVelocity += _linearAccelerationStore * Context.DeltaTime;
        _linearAccelerationStore = Vector2d.Zero;
        RefreshLinearSpeed();

        Vector2d proposedPosition = _position + _linearVelocity * Context.DeltaTime;
        TryResolveContinuousCollision(_position, ref proposedPosition);
        _position = proposedPosition;
        Collider.Rebuild();

        UpdateSleepState();
    }

    internal void OnVisualize()
    {
        if (!Active || IsKinematic)
            return;

        FixedTransform transform = Agent.Transform;
        Vector3d currentPosition = transform.Position;
        transform.Position = new Vector3d(_position.x, currentPosition.y, _position.y);
        transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            FixedMath.RadToDeg(_rotation),
            Fixed64.Zero);
    }

    internal void LateVisualize() { }

    private void UpdateKinematicPositionAndRotation()
    {
        Vector2d kinematicPosition = Agent.Transform.Position.ToVector2d();
        Fixed64 kinematicRotation = FixedMath.DegToRad(Agent.Transform.EulerAngles.y);
        bool changed = _position != kinematicPosition || _rotation != kinematicRotation;
        if (!changed)
            return;

        Wake();
        _position = kinematicPosition;
        _rotation = kinematicRotation;
        Collider.Rebuild();
    }

    internal void ApplyCollisionPositionCorrection(Vector2d positionCorrection)
    {
        if (!CanMove || positionCorrection == Vector2d.Zero)
            return;

        _position += positionCorrection;
        Collider.Rebuild();
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector2d velocityDelta)
    {
        if (!CanMove || velocityDelta == Vector2d.Zero)
            return;

        Wake();
        _linearVelocity += velocityDelta;
        RefreshLinearSpeed();
    }

    private bool TryResolveContinuousCollision(Vector2d startPosition, ref Vector2d proposedPosition)
    {
        if (!ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        Vector2d displacement = proposedPosition - startPosition;
        if (displacement.SqrMagnitude <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && displacement.SqrMagnitude <= proxyRadius * proxyRadius))
        {
            return false;
        }

        int hitCount = Context.Query2D.SweepCircleAll(
            startPosition,
            proposedPosition,
            proxyRadius,
            PhysicsLayerMask.All,
            _continuousCollisionHits,
            Collider,
            includeTriggers: false);
        int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
            ? Context.QueryMixed.SweepCircleAgainst3DAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                Collider.MixedSlabCenterY,
                Collider.MixedHalfThickness,
                PhysicsLayerMask.All,
                _continuousMixedCollisionHits,
                Collider,
                includeTriggers: false)
            : 0;

        bool found2D = TryGetFirstValidContinuousCollisionHit(hitCount, out Physics2DHit hit2D);
        bool foundMixed = TryGetFirstValidMixedContinuousCollisionHit(mixedHitCount, out PhysicsMixedHit hitMixed);
        if (found2D && (!foundMixed || hit2D.Distance <= hitMixed.Distance))
        {
            proposedPosition = startPosition + displacement.Normal * hit2D.Distance;
            RemoveClosingContinuousCollisionVelocity(hit2D.Normal);
            return true;
        }

        if (foundMixed)
        {
            proposedPosition = startPosition + displacement.Normal * hitMixed.Distance;
            RemoveClosingContinuousCollisionVelocity(hitMixed.NormalFor2DSource);
            return true;
        }

        return false;
    }

    private bool TryGetFirstValidContinuousCollisionHit(int hitCount, out Physics2DHit hit)
    {
        for (int i = 0; i < hitCount; i++)
        {
            Physics2DHit candidate = _continuousCollisionHits[i];
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

        StiffBody2D? parentBody = Collider.TopParent2D?.Body;
        if (parentBody != null && parentBody._continuousCollisionMode != ContinuousCollisionMode.Inherit)
            return parentBody._continuousCollisionMode;

        mode = Context.Settings.DefaultContinuousCollisionMode;
        return mode == ContinuousCollisionMode.Inherit
            ? ContinuousCollisionMode.Discrete
            : mode;
    }

    private Fixed64 ResolveContinuousCollisionProxyRadius()
    {
        return Collider switch
        {
            LSCircleCollider2D circle => circle.Radius,
            LSAABBoxCollider2D box => box.HalfExtents.Magnitude,
            _ => ResolveConvexContinuousCollisionProxyRadius()
        };
    }

    private Fixed64 ResolveConvexContinuousCollisionProxyRadius()
    {
        int vertexCount = Collider.VertexCount;
        if (vertexCount <= 0)
            return Fixed64.Zero;

        Vector2d center = Collider.Center;
        Fixed64 bestDistanceSquared = Fixed64.Zero;
        for (int i = 0; i < vertexCount; i++)
        {
            Fixed64 distanceSquared = Vector2d.SqrDistance(center, Collider.GetVertexUnchecked(i));
            if (distanceSquared > bestDistanceSquared)
                bestDistanceSquared = distanceSquared;
        }

        return bestDistanceSquared > Fixed64.Zero
            ? FixedMath.Sqrt(bestDistanceSquared)
            : Fixed64.Zero;
    }

    private bool IsValidContinuousCollisionHit(Physics2DHit hit)
    {
        LSCollider2D? hitCollider = hit.Collider;
        if (hitCollider == null
            || ReferenceEquals(hitCollider, Collider)
            || hitCollider.IsTrigger
            || !Context.Physics2D.RequireCollisionPair(Collider, hitCollider))
        {
            return false;
        }

        StiffBody2D? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private bool IsValidMixedContinuousCollisionHit(PhysicsMixedHit hit)
    {
        LSCollider? hitCollider = hit.Collider3D;
        if (hitCollider == null
            || hitCollider.IsTrigger
            || !Context.MixedCollisions.RequireCollisionPair(hitCollider, Collider))
        {
            return false;
        }

        StiffBody? hitBody = hitCollider.Body;
        return hitBody == null || hitBody.Immovable || hitBody.IsKinematic;
    }

    private void RemoveClosingContinuousCollisionVelocity(Vector2d normal)
    {
        if (normal.SqrMagnitude <= Fixed64.Epsilon)
            return;

        Fixed64 closingSpeed = Vector2d.Dot(_linearVelocity, normal);
        if (closingSpeed >= Fixed64.Zero)
            return;

        _linearVelocity -= normal * closingSpeed;
        RefreshLinearSpeed();
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

    private bool CanSleep => SleepEnabled && CanMove;

    private void UpdateSleepState()
    {
        if (!CanSleep)
        {
            _sleepFrameCount = 0;
            return;
        }

        if (_linearSpeed > SleepLinearSpeedThreshold)
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

    public void RecordData(IChronicler chronicler)
    {
        RecordValues.Look(chronicler, ref _position, "Position");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref _linearVelocity, "LinearVelocity");
        RecordValues.Look(chronicler, ref _linearSpeed, "LinearSpeed");
        RecordValues.Look(chronicler, ref _isSleeping, "IsSleeping");
        RecordValues.Look(chronicler, ref _sleepFrameCount, "SleepFrameCount");
        RecordValues.Look(chronicler, ref _continuousCollisionMode, "ContinuousCollisionMode", ContinuousCollisionMode.Inherit);
    }
}
