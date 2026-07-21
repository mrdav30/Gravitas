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

internal sealed partial class CollisionPair2D
{
    private bool _isColliding;
    private ContactWarmStartCache2D _warmStart;
    private bool _notificationInProgress;
    private bool _separationPending;
    private bool _colliderANotified;
    private bool _colliderBNotified;
    private SolidBody2D? _pendingBodyA;
    private SolidBody2D? _pendingBodyB;
    private long _lifetimeVersion;

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

    public int LastFrame { get; private set; } = -1;

    public bool IsColliding => _isColliding;

    internal bool IsNotificationInProgress => _notificationInProgress;

    internal long LifetimeVersion => _lifetimeVersion;

    public ContactManifold2D Manifold { get; } = new();

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

    public void MarkColliding(int frame)
    {
        if (!Manifold.HasContact)
            return;

        bool changed = MarkCollidingState(frame);

        if (!ColliderA.IsTrigger && !ColliderB.IsTrigger)
        {
            CollisionResponse2D.Resolve(this);
            WakeSleepingBodiesForCollision();
        }

        NotifyColliders(isColliding: true, changed);
    }

    internal void MarkCollidingDeferred(int frame)
    {
        if (!Manifold.HasContact)
            return;

        bool changed = MarkCollidingState(frame);
        NotifyColliders(isColliding: true, changed);
    }

    public void MarkResting(int frame)
    {
        LastFrame = frame;
    }

    public void MarkSeparated()
    {
        bool wasColliding = _isColliding;
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

    internal void StoreWarmStartImpulse(ulong contactId, Fixed64 normalImpulse, Fixed64 tangentImpulse) =>
        _warmStart.Set(contactId, normalImpulse, tangentImpulse);

    internal bool TryGetWarmStartImpulse(ulong contactId, out ContactWarmStartImpulse impulse) =>
        _warmStart.TryGet(contactId, out impulse);

    internal void ClearWarmStart() => _warmStart.Clear();

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
        _isColliding = false;
        LastFrame = -1;
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

    private bool MarkCollidingState(int frame)
    {
        bool changed = !_isColliding;
        _isColliding = true;
        LastFrame = frame;
        return changed;
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
