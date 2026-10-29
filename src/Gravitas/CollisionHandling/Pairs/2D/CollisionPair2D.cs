//=======================================================================
// CollisionPair2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>Owns one deterministic pure 2D contact pair and its lifecycle.</summary>
/// <content>Owns pair identity, contact notifications, warm starts and collision-pass freshness.</content>
internal sealed partial class CollisionPair2D
{
    private bool _isColliding;
    private ContactWarmStartCache2D _warmStart;
    internal CollisionResponse2D.GroupedSolveState? ResponseState;
    private bool _notificationInProgress;
    private bool _separationPending;
    private bool _colliderANotified;
    private bool _colliderBNotified;
    private SolidBody2D? _pendingBodyA;
    private SolidBody2D? _pendingBodyB;
    private long _lifetimeVersion;
    private uint _lastColliderAContactDependencyVersion;
    private uint _lastColliderBContactDependencyVersion;
    private bool _surfaceContactChanged;
    private ulong? _surfaceCollisionLayers;
    private bool _wasTriggerPair;
    private SolidBody2D? _recoveryBody;
    private Vector2d _recoveryPosition;

    public CollisionPair2D(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ColliderA = colliderA;
        ColliderB = colliderB;
        Initialize(colliderA, colliderB);
    }

    public LSCollider2D ColliderA { get; private set; }

    public LSCollider2D ColliderB { get; private set; }

    public int Id1 { get; private set; }

    public int Id2 { get; private set; }

    public CollisionType2D CollisionType { get; private set; }

    public long LastFrame { get; private set; } = -1;

    // Collision-pass freshness is independent of hosts advancing the public frame clock.
    internal long LastVisitToken { get; private set; } = -1;

    public bool IsColliding => _isColliding;

    internal bool IsNotificationInProgress => _notificationInProgress;

    internal long LifetimeVersion => _lifetimeVersion;

    internal bool ContactDependenciesChanged =>
        _surfaceContactChanged || _lastColliderAContactDependencyVersion != ColliderA.ContactDependencyVersion
        || _lastColliderBContactDependencyVersion != ColliderB.ContactDependencyVersion
        || _surfaceCollisionLayers.HasValue && _surfaceCollisionLayers != CurrentSurfaceCollisionLayers;

    private ulong? CurrentSurfaceCollisionLayers
    {
        get
        {
            if (ColliderA.IsTrigger || ColliderB.IsTrigger) return null;
            LSCollider2D? mover = ColliderA.StaticSurface != null ? ColliderB
                : ColliderB.StaticSurface != null ? ColliderA : null;
            return mover == null ? null : ColliderA.Context.Settings.GetCollisionLayerMask(mover.Layer, default, true, false);
        }
    }

    internal (ulong? Layers, ContactManifold2D? Before, Vector2d? Recovery) BeginCollisionLayerUpdate(ContactManifold2D target)
    {
        ulong? layers = CurrentSurfaceCollisionLayers;
        if (!_isColliding || !layers.HasValue || !_surfaceCollisionLayers.HasValue || layers == _surfaceCollisionLayers)
            return (layers, null, null);
        ContactManifold2D before = ColliderA.Context.CollisionScratch.SurfaceContactSnapshot2D;
        before.CopyFrom(target, target.LastUpdatedFrame);
        return (layers, before, HasRecovery ? _recoveryPosition : null);
    }

    internal void CompleteCollisionLayerUpdate(ContactManifold2D target,
        (ulong? Layers, ContactManifold2D? Before, Vector2d? Recovery) previous)
    {
        if (previous.Before != null)
        {
            Vector2d? recovery = HasRecovery ? _recoveryPosition : null;
            if (!previous.Before.HasSameContacts(target) || previous.Recovery != recovery)
            {
                ClearWarmStart(); ClearResponseSnapshot();
                WakeBodiesForContactChange();
            }
        }
        _surfaceCollisionLayers = previous.Layers;
    }

    public ContactManifold2D Manifold { get; } = new();

    internal bool HasRecovery => _recoveryBody != null;

    internal bool HasResponse => Manifold.HasContact || HasRecovery;

    internal void ClearRecovery() => _recoveryBody = null;

    internal void SetRecovery(LSCollider2D mover, Vector2d center)
    {
        SolidBody2D body = mover.Body!;
        // The immediately preceding certificate proved these bounded operations
        // and the actual publishable body pose; no callback intervenes here.
        _recoveryBody = body;
        _recoveryPosition = body.Position + (center - mover.Center);
    }

    internal void ApplyRecovery()
    {
        if (_recoveryBody is not { } body) return;
        if (!Vector2d.TrySubtract(_recoveryPosition, body.Position, out Vector2d delta)
            || body.PrepareCollisionPositionCorrection(delta) != _recoveryPosition)
            throw new InvalidOperationException("Pure 2D static-surface recovery is outside the movable body's supported pose.");
        StaticSurface2D? surface = ColliderA.StaticSurface ?? ColliderB.StaticSurface;
        // An earlier island response may have moved a final-pose blocker.
        if (surface == null || !Vector2d.TryAdd(body.Collider.Center, delta, out Vector2d center)
            || !surface.IsRecoveryClear(body.Collider, center))
            throw new InvalidOperationException("Pure 2D static-surface recovery no longer has a certified clear final pose.");
        ClearRecovery();
        body.ApplyCollisionPositionState(_recoveryPosition);
    }

    public void Initialize(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        _lifetimeVersion++;
        AssignPriority(colliderA, colliderB);
        Id1 = ColliderA.Id;
        Id2 = ColliderB.Id;
        CollisionType = ColliderSettings2D.GetCollisionType(ColliderA.Shape, ColliderB.Shape);
        ResetPairState();
    }

    private void AssignPriority(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        if (ShouldFirstColliderLead(colliderA, colliderB))
        {
            ColliderA = colliderA;
            ColliderB = colliderB;
            return;
        }

        ColliderA = colliderB;
        ColliderB = colliderA;
    }

    internal static bool ShouldFirstColliderLead(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        if (colliderA.Priority != colliderB.Priority)
            return colliderA.Priority > colliderB.Priority;

        SolidBody2D? bodyA = colliderA.Body;
        SolidBody2D? bodyB = colliderB.Body;
        if (bodyA == null || bodyB == null)
            return colliderA.Id <= colliderB.Id;

        if (bodyA.LinearSpeed != bodyB.LinearSpeed)
            return bodyA.LinearSpeed > bodyB.LinearSpeed;

        return colliderA.Id <= colliderB.Id;
    }

    public void MarkColliding(long frame)
    {
        if (!Manifold.HasContact && ColliderA.StaticSurface == null && ColliderB.StaticSurface == null)
            return;

        bool changed = MarkCollidingState(frame);

        if (!ColliderA.IsTrigger && !ColliderB.IsTrigger)
        {
            CollisionResponse2D.Resolve(this);
            WakeSleepingBodiesForCollision();
        }

        NotifyColliders(isColliding: true, changed);
    }

    internal void MarkCollidingDeferred(long frame)
    {
        if (!Manifold.HasContact && ColliderA.StaticSurface == null && ColliderB.StaticSurface == null)
            return;

        bool changed = MarkCollidingState(frame);
        NotifyColliders(isColliding: true, changed);
    }

    public void MarkResting(long frame)
    {
        RefreshContactDependencies();
        if (!Manifold.HasContact) ClearWarmStart();
        LastFrame = frame;
        LastVisitToken = ColliderA.Context.LateSimulateToken;
    }

    public void MarkSeparated()
    {
        bool wasColliding = _isColliding;
        if (wasColliding && !_wasTriggerPair)
            WakeBodiesForContactChange();
        ResetCollisionState();
        if (!wasColliding)
            return;

        if (_notificationInProgress)
        {
            _separationPending = true;
            _pendingBodyA = ColliderA.Body;
            _pendingBodyB = ColliderB.Body;
            return;
        }

        NotifyColliders(isColliding: false, isChanged: true);
    }

    internal void StoreWarmStartImpulse(in ContactGroupKey group, ulong contactId,
        Fixed64 normalImpulse, Fixed64 tangentImpulse) =>
        _warmStart.Set(group, contactId, normalImpulse, tangentImpulse);

    internal bool TryGetWarmStartImpulse(in ContactGroupKey group, ulong contactId,
        out ContactWarmStartImpulse impulse) => _warmStart.TryGet(group, contactId, out impulse);

    internal void RemoveWarmStartImpulse(in ContactGroupKey group, ulong contactId) =>
        _warmStart.Remove(group, contactId);

    internal void RetainWarmStart() => _warmStart.Retain(Manifold);

    internal void ClearResponseSnapshot()
    {
        if (ResponseState != null) ResponseState.Impact = default;
    }

    internal void ClearWarmStart() => _warmStart.Clear();

    internal void InvalidateSurfaceContact(LSCollider2D member, SwiftHashSet<int> suppliers)
    {
        if (!Manifold.ReferencesSuppliers(suppliers, ColliderA == member, member.Id + 1)) return;
        _surfaceContactChanged = true;
        ClearWarmStart();
        ClearResponseSnapshot();
        if (_isColliding && !_wasTriggerPair) WakeBodiesForContactChange();
    }

    private void ResetPairState()
    {
        ResetCollisionState();
        _notificationInProgress = false;
        _colliderANotified = false;
        _colliderBNotified = false;
        ClearPendingNotificationState();
    }

    private void ResetCollisionState()
    {
        ClearRecovery();
        ClearResponseSnapshot();
        _isColliding = false;
        _lastColliderAContactDependencyVersion = 0;
        _lastColliderBContactDependencyVersion = 0;
        _surfaceContactChanged = false;
        _surfaceCollisionLayers = null;
        _wasTriggerPair = false;
        LastFrame = -1;
        LastVisitToken = -1;
        Manifold.Reset();
        _warmStart.Clear();
    }

    private void NotifyColliders(bool isColliding, bool isChanged)
    {
        var registrationA = new ColliderLifetimeToken2D(ColliderA);
        var registrationB = new ColliderLifetimeToken2D(ColliderB);
        LSCollider2D colliderA = registrationA.Collider;
        LSCollider2D colliderB = registrationB.Collider;
        SolidBody2D? bodyA = colliderA.Body;
        SolidBody2D? bodyB = colliderB.Body;
        bool isTriggerPair = colliderA.IsTrigger || colliderB.IsTrigger;
        bool shouldRaiseTriggerA = isTriggerPair && ColliderTriggerEventPolicy.ShouldRaise(colliderA, colliderB);
        bool shouldRaiseTriggerB = isTriggerPair && ColliderTriggerEventPolicy.ShouldRaise(colliderB, colliderA);
        _notificationInProgress = true;
        SwiftList<Exception>? notificationExceptions = null;
        try
        {
            if (isColliding)
            {
                bool notifyEnterA = isChanged || !_colliderANotified;
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
                    bool notifyEnterB = isChanged || !_colliderBNotified;
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
                    allowInactive: false,
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

        CollisionNotificationExceptions.ThrowIfAny(notificationExceptions);
    }

    private void EndNotification(
        in ColliderLifetimeToken2D registrationA,
        in ColliderLifetimeToken2D registrationB,
        bool isTriggerPair,
        bool shouldRaiseTriggerA,
        bool shouldRaiseTriggerB)
    {
        try
        {
            if (!_separationPending)
                return;

            SolidBody2D? bodyA = _pendingBodyA;
            SolidBody2D? bodyB = _pendingBodyB;
            ClearPendingNotificationState();
            NotifySeparation(
                registrationA,
                registrationB,
                bodyA,
                bodyB,
                isChanged: true,
                allowInactive: true,
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
        in ColliderLifetimeToken2D registrationA,
        in ColliderLifetimeToken2D registrationB,
        SolidBody2D? bodyA,
        SolidBody2D? bodyB,
        bool isChanged,
        bool allowInactive,
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
                    allowInactive,
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
                    allowInactive,
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

    private bool MarkCollidingState(long frame)
    {
        RefreshContactDependencies();
        if (!Manifold.HasContact) ClearWarmStart();
        bool changed = !_isColliding;
        _isColliding = true;
        LastFrame = frame;
        LastVisitToken = ColliderA.Context.LateSimulateToken;
        return changed;
    }

    internal void RefreshContactDependencies()
    {
        uint versionA = ColliderA.ContactDependencyVersion;
        uint versionB = ColliderB.ContactDependencyVersion;
        bool triggerPair = ColliderA.IsTrigger || ColliderB.IsTrigger;
        // Even continued overlap can replace a sleeping body's load-bearing
        // normal or friction constraint; unchanged solid contacts retain sleep.
        if (_isColliding)
        {
            if (triggerPair != _wasTriggerPair)
            {
                ClearWarmStart();
                WakeBodiesForContactChange();
            }
            else if (!triggerPair)
            {
                if (versionB != _lastColliderBContactDependencyVersion) ColliderA.Body?.WakeForContactChange();
                if (versionA != _lastColliderAContactDependencyVersion) ColliderB.Body?.WakeForContactChange();
            }
        }

        _lastColliderAContactDependencyVersion = versionA;
        _lastColliderBContactDependencyVersion = versionB;
        _surfaceContactChanged = false;
        _wasTriggerPair = triggerPair;
    }

    private void WakeBodiesForContactChange()
    {
        ColliderA.Body?.WakeForContactChange();
        ColliderB.Body?.WakeForContactChange();
    }

    internal void WakeSleepingBodiesForCollision()
    {
        SolidBody2D? bodyA = ColliderA.Body;
        SolidBody2D? bodyB = ColliderB.Body;
        if (bodyA == null || bodyB == null)
            return;

        bool bodyAAwake = !bodyA.IsSleeping;
        bool bodyBAwake = !bodyB.IsSleeping;
        if (bodyA.IsSleeping && bodyBAwake)
            bodyA.Wake();
        if (bodyB.IsSleeping && bodyAAwake)
            bodyB.Wake();
    }
}
