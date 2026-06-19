using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
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

    internal Vector2d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameStart;
    }

    internal Vector2d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameDisplacement;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        _continuousCollisionFrameToken = token;
        _continuousCollisionFrameStart = _position;
        _continuousCollisionFrameDisplacement = PredictContinuousCollisionDisplacement();
    }

    private Vector2d PredictContinuousCollisionDisplacement()
    {
        if (!CanTranslate || _isSleeping)
            return Vector2d.Zero;

        Fixed64 deltaTime = Context.DeltaTime;
        Vector2d predictedVelocity = _linearVelocity + (_deltaAcceleration + Gravity) * deltaTime;
        return predictedVelocity.MagnitudeSquared > Fixed64.Epsilon
            ? predictedVelocity * deltaTime
            : Vector2d.Zero;
    }

    public void AddForce(Vector2d force)
    {
        if (force != Vector2d.Zero)
            Wake();

        _deltaAcceleration += force * InverseMass;
    }

    public void AddTorque(Fixed64 torque)
    {
        if (torque == Fixed64.Zero || !CanRotate)
            return;

        Wake();
        _deltaAngularAcceleration += torque * EffectiveInverseMomentOfInertia;
    }

    public void AddAngularImpulse(Fixed64 impulse)
    {
        if (impulse == Fixed64.Zero || !CanRotate)
            return;

        Wake();
        _angularVelocity += impulse * EffectiveInverseMomentOfInertia;
        RefreshAngularSpeed();
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
        _angularVelocity = Fixed64.Zero;
        _angularAccelerationStore = Fixed64.Zero;
        _deltaAngularAcceleration = Fixed64.Zero;
        _angularSpeed = Fixed64.Zero;
        Context.Collisions2D.RefreshPartitionAwakeState(Collider);
    }

    private void RefreshPartitionMobility()
    {
        if (!Active)
            return;

        Context.Collisions2D.RefreshColliderPartition(Collider);
        if (Context.Settings.RuntimeMode.RunsMixedContacts())
            Context.MixedCollisions.Refresh2DColliderPartition(Collider);
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

        if (CanRotate)
        {
            _angularAccelerationStore = _deltaAngularAcceleration;
            _deltaAngularAcceleration = Fixed64.Zero;
            _angularVelocity += _angularAccelerationStore * Context.DeltaTime;
            _rotation += _angularVelocity * Context.DeltaTime;
            RefreshAngularSpeed();
        }
        else
        {
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
        }

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
        transform.Position = new Vector3d(_position.X, currentPosition.Y, _position.Y);
        transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            FixedMath.RadToDeg(_rotation),
            Fixed64.Zero);
    }

    private void UpdateKinematicPositionAndRotation()
    {
        Vector2d kinematicPosition = Agent.Transform.Position.ToVector2d();
        Fixed64 kinematicRotation = FixedMath.DegToRad(Agent.Transform.EulerAngles.Y);
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
        if (!CanTranslate || positionCorrection == Vector2d.Zero)
            return;

        _position += positionCorrection;
        Collider.Rebuild();
    }

    internal void ApplyCollisionLinearVelocityDelta(Vector2d velocityDelta)
    {
        if (!CanTranslate || velocityDelta == Vector2d.Zero)
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
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
            return false;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        if (proxyRadius <= Fixed64.Epsilon
            || (mode == ContinuousCollisionMode.Auto && displacement.MagnitudeSquared <= proxyRadius * proxyRadius))
        {
            return false;
        }

        int hitCount = Context.Query2D.SweepCircleAgainstStaticAll(
            startPosition,
            proposedPosition,
            proxyRadius,
            PhysicsLayerMask.All,
            _continuousCollisionHits,
            Collider,
            includeTriggers: false);
        int mixedHitCount = Context.Settings.RuntimeMode.RunsMixedContacts()
            ? Context.QueryMixed.SweepCircleAgainstStatic3DAll(
                startPosition,
                proposedPosition,
                proxyRadius,
                Collider.MixedSlabCenterY,
                Collider.MixedHalfThickness,
                PhysicsLayerMask.All,
                _continuousMixedCollisionHits,
                Collider,
                includeTriggers: false,
                cacheTargetPartitions: true)
            : 0;

        bool found2D = TryGetFirstValidContinuousCollisionHit(hitCount, out Physics2DHit hit2D);
        bool foundDynamic2D = TryGetFirstDynamicContinuousCollisionHit(
            startPosition,
            proposedPosition,
            proxyRadius,
            out Physics2DHit dynamicHit2D,
            out Fixed64 dynamicClosingSpeed2D);
        if (ShouldReplaceContinuousCollisionHit(dynamicHit2D, dynamicClosingSpeed2D, foundDynamic2D, found2D, hit2D, Fixed64.Zero))
        {
            hit2D = dynamicHit2D;
            found2D = true;
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

        if (found2D && (!foundMixed || hit2D.Distance <= hitMixed.Distance))
        {
            proposedPosition = startPosition + displacement.Normalized * hit2D.Distance;
            RemoveClosingContinuousCollisionVelocity(hit2D.Normal);
            return true;
        }

        if (foundMixed)
        {
            proposedPosition = startPosition + displacement.Normalized * hitMixed.Distance;
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

    private bool TryGetFirstDynamicContinuousCollisionHit(
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        out Physics2DHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        Vector2d sourceDisplacement = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        bool found = false;
        Physics2DHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics2D.QueryPlanarContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptCircleBounds(startPosition, sourceDisplacement, proxyRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics2D.TryGetDynamicBody(dynamicId, out StiffBody2D target)
                || !IsEligibleDynamicContinuousCollisionTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeCircles(
                    startPosition,
                    sourceDisplacement,
                    proxyRadius,
                    target.ContinuousCollisionFrameStart,
                    target.ContinuousCollisionFrameDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector2d normal,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            Vector2d sourceCenter = startPosition + sourceDisplacement * normalizedTime;
            Vector2d targetCenter = target.ContinuousCollisionFrameStart + target.ContinuousCollisionFrameDisplacement * normalizedTime;
            Vector2d point = ResolveDynamicContactPoint(sourceCenter, targetCenter, normal, targetRadius);
            var candidate = new Physics2DHit(target.Collider, point, normal, distance);
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
        Vector2d startPosition,
        Vector2d proposedPosition,
        Fixed64 proxyRadius,
        out PhysicsMixedHit hit,
        out Fixed64 closingSpeed)
    {
        hit = default;
        closingSpeed = Fixed64.Zero;

        if (!Context.Settings.RuntimeMode.RunsMixedContacts())
            return false;

        Vector2d sourceDisplacement2D = proposedPosition - startPosition;
        Fixed64 sourceLength = sourceDisplacement2D.Magnitude;
        if (sourceLength <= Fixed64.Epsilon)
            return false;

        Vector3d sourceStart = new(startPosition.X, Collider.MixedSlabCenterY, startPosition.Y);
        Vector3d sourceDisplacement = new(sourceDisplacement2D.X, Fixed64.Zero, sourceDisplacement2D.Y);
        Fixed64 sourceRadius = FixedMath.Max(proxyRadius, Collider.MixedHalfThickness);
        bool found = false;
        PhysicsMixedHit best = default;
        Fixed64 bestClosingSpeed = Fixed64.Zero;
        int token = Context.LateSimulateToken;
        SwiftList<int> candidateIds = Context.Physics.QueryContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(sourceStart, sourceDisplacement, sourceRadius));
        for (int candidateIndex = 0; candidateIndex < candidateIds.Count; candidateIndex++)
        {
            int dynamicId = candidateIds[candidateIndex];
            if (!Context.Physics.TryGetDynamicBody(dynamicId, out StiffBody target)
                || !IsEligibleDynamicMixed3DTarget(target))
            {
                continue;
            }

            target.EnsureContinuousCollisionFramePrepared(token);
            Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadiusForDynamicTarget();
            if (targetRadius <= Fixed64.Epsilon
                || !ContinuousCollisionMath.TrySweepRelativeSpheres(
                    sourceStart,
                    sourceDisplacement,
                    sourceRadius,
                    target.ContinuousCollisionFrameStart,
                    target.ContinuousCollisionFrameDisplacement,
                    targetRadius,
                    out Fixed64 normalizedTime,
                    out Vector3d normalForSource,
                    out Fixed64 candidateClosingSpeed))
            {
                continue;
            }

            Fixed64 distance = sourceLength * normalizedTime;
            Vector3d sourceCenter = sourceStart + sourceDisplacement * normalizedTime;
            Vector3d targetCenter = target.ContinuousCollisionFrameStart + target.ContinuousCollisionFrameDisplacement * normalizedTime;
            Vector3d point2D = sourceCenter - normalForSource * sourceRadius;
            Vector3d point3D = ResolveDynamicContactPoint(sourceCenter, targetCenter, normalForSource, targetRadius);
            var candidate = new PhysicsMixedHit(
                target.Collider,
                null,
                point3D,
                point2D,
                normalForSource,
                distance,
                sourceDisplacement.MagnitudeSquared > Fixed64.Epsilon ? sourceDisplacement.Normalized : Vector3d.Zero);
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

    private bool IsEligibleDynamicContinuousCollisionTarget(StiffBody2D target)
    {
        if (ReferenceEquals(target, this)
            || !target.Active
            || target.Immovable
            || target.IsKinematic
            || target.Collider.IsTrigger
            || !Context.Physics2D.RequireCollisionPair(Collider, target.Collider))
        {
            return false;
        }

        return true;
    }

    private bool IsEligibleDynamicMixed3DTarget(StiffBody target)
    {
        return target.Active
            && !target.Immovable
            && !target.IsKinematic
            && !target.Collider.IsTrigger
            && Context.MixedCollisions.RequireCollisionPair(target.Collider, Collider);
    }

    private static bool ShouldReplaceContinuousCollisionHit(
        Physics2DHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        Physics2DHit current,
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

        return candidate.Collider.Id < current.Collider.Id;
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
    private static Vector2d ResolveDynamicContactPoint(
        Vector2d sourceCenter,
        Vector2d targetCenter,
        Vector2d normalForSource,
        Fixed64 targetRadius)
    {
        if (normalForSource.MagnitudeSquared > Fixed64.Epsilon)
            return targetCenter + normalForSource * targetRadius;

        Vector2d fallback = sourceCenter - targetCenter;
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? targetCenter + fallback.Normalized * targetRadius
            : targetCenter;
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
            LSCircleCollider2D circle => circle.ScaledRadius,
            LSAABBoxCollider2D box => box.ScaledHalfExtents.Magnitude,
            LSCompoundCollider2D compound => compound.ScaledRadius,
            _ => ResolveConvexContinuousCollisionProxyRadius()
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed64 ResolveContinuousCollisionProxyRadiusForDynamicTarget()
    {
        return ResolveContinuousCollisionProxyRadius();
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
            Fixed64 distanceSquared = Vector2d.DistanceSquared(center, Collider.GetVertexUnchecked(i));
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
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
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

    private bool CanSleep => SleepEnabled && CanTranslate;

    private void UpdateSleepState()
    {
        if (!CanSleep)
        {
            _sleepFrameCount = 0;
            return;
        }

        if (_linearSpeed > SleepLinearSpeedThreshold || _angularSpeed > _sleepAngularSpeedThreshold)
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

    private void RefreshAngularSpeed()
    {
        _angularSpeed = _angularVelocity.Abs();
        if (_angularSpeed <= Fixed64.Epsilon)
        {
            _angularVelocity = Fixed64.Zero;
            _angularSpeed = Fixed64.Zero;
        }
    }

    internal void RefreshMassPropertiesFromColliderShape()
    {
        if (!_centerOfMassOffsetExplicit)
            _localCenterOfMassOffset = Collider.CalculateLocalCenterOfMassOffset();

        if (_mass <= Fixed64.Zero)
        {
            _momentOfInertia = Fixed64.Zero;
            _inverseMomentOfInertia = Fixed64.Zero;
            return;
        }

        _momentOfInertia = Collider.CalculateMomentOfInertia(_mass, _localCenterOfMassOffset);
        _inverseMomentOfInertia = _momentOfInertia > Fixed64.Zero
            ? Fixed64.One / _momentOfInertia
            : Fixed64.Zero;
    }

    public void RecordData(IChronicler chronicler)
    {
        bool active = Active;
        bool immovable = Immovable;
        bool isKinematic = IsKinematic;
        Fixed64 mass = Mass;
        Fixed64 restitutionCoefficient = RestitutionCoefficient;
        Fixed64 frictionCoefficient = FrictionCoefficient;
        Vector2d gravity = Gravity;
        bool sleepEnabled = SleepEnabled;
        int sleepFrameThreshold = SleepFrameThreshold;
        Fixed64 sleepLinearSpeedThreshold = SleepLinearSpeedThreshold;
        Fixed64 sleepAngularSpeedThreshold = SleepAngularSpeedThreshold;

        RecordValues.Look(chronicler, ref active, "Active", false);
        RecordValues.Look(chronicler, ref immovable, "Immovable", false);
        RecordValues.Look(chronicler, ref isKinematic, "IsKinematic", false);
        RecordValues.Look(chronicler, ref _position, "Position");
        RecordValues.Look(chronicler, ref _rotation, "Rotation");
        RecordValues.Look(chronicler, ref PreventAngularForces, "PreventAngularForces", false);
        RecordValues.Look(chronicler, ref _localCenterOfMassOffset, "LocalCenterOfMassOffset");
        RecordValues.Look(chronicler, ref _centerOfMassOffsetExplicit, "CenterOfMassOffsetExplicit", false);
        RecordValues.Look(chronicler, ref _linearVelocity, "LinearVelocity");
        RecordValues.Look(chronicler, ref _linearAccelerationStore, "LinearAccelerationStore");
        RecordValues.Look(chronicler, ref _deltaAcceleration, "DeltaAcceleration");
        RecordValues.Look(chronicler, ref _linearSpeed, "LinearSpeed");
        RecordValues.Look(chronicler, ref _angularVelocity, "AngularVelocity");
        RecordValues.Look(chronicler, ref _angularAccelerationStore, "AngularAccelerationStore");
        RecordValues.Look(chronicler, ref _deltaAngularAcceleration, "DeltaAngularAcceleration");
        RecordValues.Look(chronicler, ref _angularSpeed, "AngularSpeed");
        RecordValues.Look(chronicler, ref _isSleeping, "IsSleeping");
        RecordValues.Look(chronicler, ref _sleepFrameCount, "SleepFrameCount");
        RecordValues.Look(chronicler, ref mass, "Mass");
        RecordValues.Look(chronicler, ref restitutionCoefficient, "RestitutionCoefficient", Fixed64.Half);
        RecordValues.Look(chronicler, ref frictionCoefficient, "FrictionCoefficient", Fixed64.One);
        RecordValues.Look(chronicler, ref gravity, "Gravity", Vector2d.Zero);
        RecordValues.Look(chronicler, ref sleepEnabled, "SleepEnabled", true);
        RecordValues.Look(chronicler, ref sleepFrameThreshold, "SleepFrameThreshold", 16);
        RecordValues.Look(chronicler, ref sleepLinearSpeedThreshold, "SleepLinearSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref sleepAngularSpeedThreshold, "SleepAngularSpeedThreshold", (Fixed64)0.001f);
        RecordValues.Look(chronicler, ref _continuousCollisionMode, "ContinuousCollisionMode", ContinuousCollisionMode.Inherit);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            Active = active;
            _immovable = immovable;
            _isKinematic = isKinematic;
            Mass = mass;
            RestitutionCoefficient = restitutionCoefficient;
            FrictionCoefficient = frictionCoefficient;
            Gravity = gravity;
            SleepEnabled = sleepEnabled;
            SleepFrameThreshold = sleepFrameThreshold;
            SleepLinearSpeedThreshold = sleepLinearSpeedThreshold;
            SleepAngularSpeedThreshold = sleepAngularSpeedThreshold;
        }

        Collider.RecordData(chronicler);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            RefreshMassPropertiesFromColliderShape();
            ApplyLoadedState();
        }
    }

    private void ApplyLoadedState()
    {
        FixedTransform transform = Agent.Transform;
        Vector3d currentPosition = transform.Position;
        transform.Position = new Vector3d(_position.X, currentPosition.Y, _position.Y);
        transform.Rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            FixedMath.RadToDeg(_rotation),
            Fixed64.Zero);
        Collider.Rebuild();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ClampNearZero(Vector2d value)
    {
        Fixed64 x = value.X.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.X;
        Fixed64 y = value.Y.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.Y;
        return new Vector2d(x, y);
    }
}
