//=======================================================================
// CollisionPair.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Grids;
using GridForge.Grids.Topology;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal enum CollisionResponseDispatchMode
{
    Immediate,
    Deferred
}

/// <summary>
/// Handles collision pairs between various types of colliders using the Separating Axis Theorem and maintains related state information.
/// </summary>
public partial class CollisionPair
{
    public bool Debug = true;

    public bool Active { get; private set; }

    private bool _isPooledForDeactivation;

    public GravitasWorldContext Context { get; private set; } = null!;

    public GridWorld World => Context.World;

    // stores order in which they come in
    public int Id1 { get; private set; }
    public int Id2 { get; private set; }

    public LSCollider ColliderA { get; private set; } = null!;
    public LSCollider ColliderB { get; private set; } = null!;

    public uint PartitionVersion;
    public ushort PairVersion = 1;

    public int LastFrame { get; private set; }
    public int LastCollidedFrame { get; private set; }

    private Fixed64 _fastCollideDistance;
    private Fixed64 _fastDistance;
    public CollisionType CollisionType { get; private set; }
    private bool _doPhysics = true;

    public short CullCounter { get; private set; }
    private bool _preventDistanceCull;
    private Fixed64 _fastDistanceOffset;
    private uint _lastColliderABroadPhaseVersion;
    private uint _lastColliderBBroadPhaseVersion;

    private bool _isColliding;
    private bool _isCollidingChanged;
    private bool _notificationInProgress;
    private bool _separationPending;
    private bool _colliderANotified;
    private bool _colliderBNotified;
    private SolidBody? _pendingBodyA;
    private SolidBody? _pendingBodyB;
    private long _lifetimeVersion;

    internal bool IsNotificationInProgress => _notificationInProgress;

    internal long LifetimeVersion => _lifetimeVersion;

    public ContactManifold Manifold { get; } = new();

    private ContactWarmStartCache _warmStart;

    internal CollisionPair(LSCollider c1, LSCollider c2) => Initialize(c1, c2);

    /// <summary>
    /// Initializes the CollisionPair with the given colliders.
    /// </summary>
    /// <param name="c1">The first collider.</param>
    /// <param name="c2">The second collider.</param>
    internal void Initialize(LSCollider c1, LSCollider c2)
    {
        SwiftThrowHelper.ThrowIfNull(c1, nameof(c1));
        SwiftThrowHelper.ThrowIfNull(c2, nameof(c2));
        SwiftThrowHelper.ThrowIfArgument(c1 == c2, nameof(c2), "Cannot create a CollisionPair with the same collider.");
        GravitasWorldContext context = c1.Context;
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(context, c2.Context),
            nameof(c2),
            "Colliders must be in the same context to create a CollisionPair.");

        Context = context;
        _lifetimeVersion++;

        Reset();

        AssignPriority(c1, c2);
        Id1 = ColliderA.Id;
        Id2 = ColliderB.Id;

        CollisionType = ColliderSettings.GetCollisionType(ColliderA.Shape, ColliderB.Shape);

        // Calculate the square of the sum of the radii of the bounding spheres
        _fastCollideDistance = ColliderA!.Bounds.Scope.Magnitude + ColliderB!.Bounds.Scope.Magnitude;
        _fastCollideDistance *= _fastCollideDistance;

        _doPhysics = ColliderA!.Body != null && ColliderB!.Body != null && !ColliderA!.IsTrigger && !ColliderB!.IsTrigger;

        // Immediately check collision. If collision distance is too large, do
        // not cull based on distance.
        CullCounter = 0;
        _preventDistanceCull = _fastCollideDistance > Context.Environment.CullFastDistanceMax;
        _fastDistanceOffset = Fixed64.FromRaw((int)_fastCollideDistance) + (Fixed64.One * 2) * (Fixed64.One * 2);

        LastCollidedFrame = Context.FrameCount;
        RefreshBroadPhaseVersions();
        PairVersion++;
        Active = true;
    }

    public void AssignPriority(LSCollider c1, LSCollider c2)
    {
        if (ShouldFirstColliderLead(c1, c2))
        {
            ColliderA = c1;
            ColliderB = c2;
            return;
        }

        ColliderA = c2;
        ColliderB = c1;
    }

    private static bool ShouldFirstColliderLead(LSCollider c1, LSCollider c2)
    {
        if (c1.Priority != c2.Priority)
            return c1.Priority > c2.Priority;

        if (c1.Body == null || c2.Body == null)
            return true;

        if (c1.Body.LinearSpeed != c2.Body.LinearSpeed)
            return c1.Body.LinearSpeed > c2.Body.LinearSpeed;

        return true;
    }

    /// <summary>
    /// Checks and distributes collisions between colliders.
    /// Called by Partition Manager every fixed update if 2 colliders are on the same partion.
    /// </summary>
    public void UpdateCollision() => UpdateCollision(CollisionResponseDispatchMode.Immediate);

    internal void UpdateCollisionDeferred() => UpdateCollision(CollisionResponseDispatchMode.Deferred);

    private void UpdateCollision(CollisionResponseDispatchMode responseMode)
    {
        if (!Active)
            return;

        UpdateLastFrame();
        DeactivateAndPoolIfRequired();

        if (IsCullStateInvalidated())
            CullCounter = 0;

        if (CullCounter <= 0)
        {
            ProcessCollision(responseMode);
            RefreshBroadPhaseVersions();
            if (_isCollidingChanged && !_isColliding)
                Manifold.Reset();

            HandleCullingIfNotColliding();
            return;
        }

        CullCounter--;  // Culled and one step closer to checking again.
    }

    private void UpdateLastFrame() => LastFrame = Context.FrameCount;

    private void DeactivateAndPoolIfRequired()
    {
        if (_isPooledForDeactivation)
            return;

        Context.Physics.PoolForDeactivation(this);
        _isPooledForDeactivation = true;
    }

    private void ProcessCollision(CollisionResponseDispatchMode responseMode)
    {
        if (!ShouldPerformCollisionCheck())
        {
            _isCollidingChanged = _isColliding;
            _isColliding = false;
            Manifold.Reset();
            _warmStart.Clear();
            return;
        }

        bool result = CheckCollision();
        if (result)
            Context.Diagnostics.EmitContact(this, result);

        if (result ^ _isColliding)
        {
            _isColliding = result;
            _isCollidingChanged = true;
        }

        if (!result || !Manifold.HasContact)
        {
            _warmStart.Clear();
            return;
        }

        if (!_doPhysics)
            return;

        if (responseMode == CollisionResponseDispatchMode.Deferred)
        {
            Context.Physics.QueueDiscreteResponsePair(this);
            return;
        }

        WakeSleepingBodiesForCollision();
        CollisionResponse.CalculateImpulse(this);
    }

    internal void WakeSleepingBodiesForCollision()
    {
        SolidBody? bodyA = ColliderA.Body;
        SolidBody? bodyB = ColliderB.Body;
        if (bodyA == null || bodyB == null)
            return;

        bool bodyAAwake = bodyA.IsAwakeForCollision;
        bool bodyBAwake = bodyB.IsAwakeForCollision;

        if (bodyA.IsSleeping && bodyBAwake)
            bodyA.Wake();
        if (bodyB.IsSleeping && bodyAAwake)
            bodyB.Wake();
    }

    public void NotifyCollidersOfContact()
    {
        bool isColliding = _isColliding;
        bool isChanged = _isCollidingChanged;

        var registrationA = new ColliderLifetimeToken(ColliderA);
        var registrationB = new ColliderLifetimeToken(ColliderB);
        LSCollider colliderA = registrationA.Collider;
        LSCollider colliderB = registrationB.Collider;
        SolidBody? bodyA = colliderA.Body;
        SolidBody? bodyB = colliderB.Body;
        bool isTriggerPair = colliderA.IsTrigger || colliderB.IsTrigger;
        bool shouldRaiseTriggerA = isTriggerPair && ColliderTriggerEventPolicy.ShouldRaise(colliderA, colliderB);
        bool shouldRaiseTriggerB = isTriggerPair && ColliderTriggerEventPolicy.ShouldRaise(colliderB, colliderA);
        _notificationInProgress = true;
        SwiftList<Exception>? notificationExceptions = null;
        try
        {
            if (isColliding)
            {
                bool notifyEnterA = isChanged && !_colliderANotified;
                _colliderANotified = true;
                colliderA.NotifyContact(
                    colliderB,
                    bodyB,
                    isColliding: true,
                    notifyEnterA,
                    allowInactive: false,
                    registrationA,
                    registrationB,
                    isTriggerPair,
                    shouldRaiseTriggerA);
                if (registrationA.IsActive && registrationB.IsActive && !_separationPending)
                {
                    bool notifyEnterB = isChanged && !_colliderBNotified;
                    _colliderBNotified = true;
                    colliderB.NotifyContact(
                        colliderA,
                        bodyA,
                        isColliding: true,
                        notifyEnterB,
                        allowInactive: false,
                        registrationB,
                        registrationA,
                        isTriggerPair,
                        shouldRaiseTriggerB);
                }
            }
            else
            {
                NotifySeparation(
                    registrationA,
                    registrationB,
                    bodyA,
                    bodyB,
                    isChanged,
                    isTriggerPair,
                    shouldRaiseTriggerA,
                    shouldRaiseTriggerB);
            }
        }
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
        }

        try
        {
            EndNotification(
                registrationA,
                registrationB,
                isTriggerPair,
                shouldRaiseTriggerA,
                shouldRaiseTriggerB);
        }
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
        }

        _isCollidingChanged &= _isColliding
            & !(_colliderANotified & _colliderBNotified);

        CollisionNotificationExceptions.ThrowIfAny(notificationExceptions);
    }

    private void EndNotification(
        in ColliderLifetimeToken registrationA,
        in ColliderLifetimeToken registrationB,
        bool isTriggerPair,
        bool shouldRaiseTriggerA,
        bool shouldRaiseTriggerB)
    {
        try
        {
            if (!_separationPending)
                return;

            SolidBody? bodyA = _pendingBodyA;
            SolidBody? bodyB = _pendingBodyB;
            ClearPendingNotificationState();
            NotifySeparation(
                registrationA,
                registrationB,
                bodyA,
                bodyB,
                isChanged: true,
                isTriggerPair,
                shouldRaiseTriggerA,
                shouldRaiseTriggerB);
        }
        finally
        {
            ClearPendingNotificationState();
            _notificationInProgress = false;
        }
    }

    private void NotifySeparation(
        in ColliderLifetimeToken registrationA,
        in ColliderLifetimeToken registrationB,
        SolidBody? bodyA,
        SolidBody? bodyB,
        bool isChanged,
        bool isTriggerPair,
        bool shouldRaiseTriggerA,
        bool shouldRaiseTriggerB)
    {
        bool notifyA = _colliderANotified;
        bool notifyB = _colliderBNotified;
        _colliderANotified = false;
        _colliderBNotified = false;

        SwiftList<Exception>? notificationExceptions = null;
        if (notifyA)
        {
            try
            {
                registrationA.Collider.NotifyContact(
                    registrationB.Collider,
                    bodyB,
                    isColliding: false,
                    isChanged,
                    allowInactive: true,
                    registrationA,
                    registrationB,
                    isTriggerPair,
                    shouldRaiseTriggerA);
            }
            catch (Exception exception)
            {
                CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
            }
        }

        if (notifyB && registrationB.IsCurrentLifetime)
        {
            try
            {
                registrationB.Collider.NotifyContact(
                    registrationA.Collider,
                    bodyA,
                    isColliding: false,
                    isChanged,
                    allowInactive: true,
                    registrationB,
                    registrationA,
                    isTriggerPair,
                    shouldRaiseTriggerB);
            }
            catch (Exception exception)
            {
                CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
            }
        }

        CollisionNotificationExceptions.ThrowIfAny(notificationExceptions);
    }

    private void ClearPendingNotificationState()
    {
        _separationPending = false;
        _pendingBodyA = null;
        _pendingBodyB = null;
    }

    private void HandleCullingIfNotColliding()
    {
        if (_isColliding)
        {
            LastCollidedFrame = Context.FrameCount;
            return;
        }

        CalculateCullScore();
    }

    internal bool TryPreserveSleepingRestingContact()
    {
        if (!_isColliding || !Manifold.HasContact)
            return false;

        if (ColliderA.Body?.IsSleeping != true && ColliderB.Body?.IsSleeping != true)
            return false;

        LastCollidedFrame = Context.FrameCount;
        return true;
    }

    private bool CheckCollision()
    {
        if (!BroadPhaseVersionChanged() && _isColliding)
            return _isColliding;

        return CollisionDetection.DoCollisionCheck(this);
    }

    private bool IsCullStateInvalidated()
    {
        return ColliderA.PartitionChanged
            || ColliderB.PartitionChanged
            || BroadPhaseVersionChanged();
    }

    private bool BroadPhaseVersionChanged()
    {
        return ColliderA.BroadPhaseVersion != _lastColliderABroadPhaseVersion
            || ColliderB.BroadPhaseVersion != _lastColliderBBroadPhaseVersion;
    }

    private void RefreshBroadPhaseVersions()
    {
        _lastColliderABroadPhaseVersion = ColliderA.BroadPhaseVersion;
        _lastColliderBBroadPhaseVersion = ColliderB.BroadPhaseVersion;
    }

    private bool ShouldPerformCollisionCheck()
    {
        // Center distance remains a scheduling signal only. Canonical geometry
        // may extend beyond a scalar face while its conservative broad-phase
        // bounds are clipped to the representable domain, which can shorten
        // Bounds.Scope without shortening the physical shape.
        _fastDistance = Vector3d.DistanceSquared(ColliderA.Center, ColliderB.Center);

        // Inclusive bounds overlap preserves zero-depth touching contacts for the manifold pass.
        return BoundsOverlapInclusive(ColliderA, ColliderB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlapInclusive(LSCollider colliderA, LSCollider colliderB)
    {
        return colliderA.BoundsMin.X <= colliderB.BoundsMax.X
            && colliderA.BoundsMax.X >= colliderB.BoundsMin.X
            && colliderA.BoundsMin.Y <= colliderB.BoundsMax.Y
            && colliderA.BoundsMax.Y >= colliderB.BoundsMin.Y
            && colliderA.BoundsMin.Z <= colliderB.BoundsMax.Z
            && colliderA.BoundsMax.Z >= colliderB.BoundsMin.Z;
    }

    private void CalculateCullScore()
    {
        int distanceScore = 0;
        int velocityScore = 0;
        if (!_preventDistanceCull)
        {
            int distanceMax = Context.Environment.CullDistanceMax;
            if (distanceMax > 0)
            {
                int step = GetCullDistanceStep(World!);
                distanceScore = Math.Clamp((int)(_fastDistance - _fastDistanceOffset) / step + Context.Collisions.CullDistributor, 0, distanceMax);
            }

            int cullVelocityStep = Context.Environment.CullVelocityStep;
            if (cullVelocityStep > 0)
                velocityScore = Math.Clamp((int)(ColliderA.Velocity - ColliderB.Velocity).Magnitude / cullVelocityStep, 0, Context.Environment.CullVelocityMax);
        }

        int timeScore = 0;
        int cullTimeStep = Context.Environment.CullTimeStep;
        if (cullTimeStep > 0)
            timeScore = Math.Clamp((Context.FrameCount - LastCollidedFrame) / cullTimeStep, 0, Context.Environment.CullTimeMax);

        CullCounter = (short)Math.Clamp(distanceScore + timeScore - velocityScore, 0, short.MaxValue);
    }

    /// <summary>
    /// Defines the step value for distance-based culling. The score is increased
    /// when the distance between objects increases. Higher values make the culling more aggressive for distant objects.
    /// </summary>
    private int GetCullDistanceStep(GridWorld world)
    {
        int distanceMax = Context.Environment.CullDistanceMax;
        Fixed64 cellEdge = GridTopologyMetricUtility.GetRepresentativeCellEdge(world);
        int step = ((cellEdge + Fixed64.One * 2) * (cellEdge + Fixed64.One * 2) / distanceMax).CeilToInt();
        return Math.Max(1, step);
    }

    public void Reset()
    {
        Manifold.Reset();
        _warmStart.Clear();
        _isColliding = false;
        _isCollidingChanged = false;
        _isPooledForDeactivation = false;
        _notificationInProgress = false;
        _colliderANotified = false;
        _colliderBNotified = false;
        ClearPendingNotificationState();
    }

    internal void StoreWarmStartImpulse(
        ulong contactId,
        Vector3d normal,
        Fixed64 normalImpulse,
        Fixed64 tangentImpulse,
        Fixed64 secondaryTangentImpulse = default) =>
        _warmStart.Set(contactId, normal, normalImpulse, tangentImpulse, secondaryTangentImpulse);

    internal bool TryGetWarmStartImpulse(ulong contactId, out ContactWarmStartImpulse impulse) =>
        _warmStart.TryGet(contactId, out impulse);

    internal void ClearWarmStart() => _warmStart.Clear();

    /// <summary>
    /// Deactivates the CollisionPair.
    /// </summary>
    public void Deactivate()
    {
        bool notifySeparation = _isColliding || _colliderANotified || _colliderBNotified;
        if (notifySeparation)
        {
            _isColliding = false;
            _isCollidingChanged = true;
            if (_notificationInProgress)
            {
                _separationPending = true;
                _pendingBodyA = ColliderA.Body;
                _pendingBodyB = ColliderB.Body;
            }
        }

        Manifold.Reset();
        _warmStart.Clear();
        _isPooledForDeactivation = false;
        Active = false;

        if (notifySeparation && !_notificationInProgress)
            NotifyCollidersOfContact();
    }
}
