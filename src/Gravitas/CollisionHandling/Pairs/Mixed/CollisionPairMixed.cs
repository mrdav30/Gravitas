//=======================================================================
// CollisionPairMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Owns one stable mixed 3D/2D collision pair identity and contact lifecycle.
/// </summary>
internal sealed partial class CollisionPairMixed
{
    private bool _isColliding;
    private bool _isTriggerPair;
    private bool _notificationInProgress;
    private bool _separationPending;
    private bool _collider3DNotified;
    private bool _collider2DNotified;
    private long _lifetimeVersion;

    public CollisionPairMixed(LSCollider collider3D, LSCollider2D collider2D)
    {
        Collider3D = collider3D;
        Collider2D = collider2D;
        Initialize(collider3D, collider2D);
    }

    public LSCollider Collider3D { get; private set; }

    public LSCollider2D Collider2D { get; private set; }

    public int Collider3DId { get; private set; }

    public int Collider2DId { get; private set; }

    public ulong Key { get; private set; }

    public GravitasWorldContext Context => Collider3D.Context;

    public int LastFrame { get; private set; } = -1;

    public MixedContact Contact { get; private set; }

    internal long LifetimeVersion => _lifetimeVersion;

    internal bool IsNotificationInProgress => _notificationInProgress;

    public void Initialize(LSCollider collider3D, LSCollider2D collider2D)
    {
        SwiftThrowHelper.ThrowIfNull(collider3D, nameof(collider3D));
        SwiftThrowHelper.ThrowIfNull(collider2D, nameof(collider2D));

        _lifetimeVersion++;
        Collider3D = collider3D;
        Collider2D = collider2D;
        Collider3DId = collider3D.Id;
        Collider2DId = collider2D.Id;
        Key = MixedColliderKey.CreateKey(Collider3DId, Collider2DId);
        LastFrame = -1;
        _isColliding = false;
        _isTriggerPair = collider3D.IsTrigger || collider2D.IsTrigger;
        _notificationInProgress = false;
        _separationPending = false;
        _collider3DNotified = false;
        _collider2DNotified = false;
        Contact = default;
    }

    public void MarkColliding(int frame, MixedContact contact)
    {
        bool changed = !_isColliding;
        _isColliding = true;
        _isTriggerPair = Collider3D.IsTrigger || Collider2D.IsTrigger;
        Contact = contact;
        LastFrame = frame;

        Context.Diagnostics.EmitMixedContact(this, contact);

        var registration3D = new ColliderLifetimeToken(Collider3D);
        var registration2D = new ColliderLifetimeToken2D(Collider2D);
        bool shouldRaiseTrigger = ColliderTriggerEventPolicy.ShouldRaise(Collider3D, Collider2D);
        _notificationInProgress = true;
        try
        {
            bool notifyEnter3D = changed || !_collider3DNotified;
            _collider3DNotified = true;
            Collider3D.NotifyMixedContact(
                Collider2D,
                true,
                notifyEnter3D,
                _isTriggerPair,
                allowInactive: false,
                registration3D,
                registration2D,
                shouldRaiseTrigger);
            if (_isColliding && registration3D.IsActive && registration2D.IsActive && !_separationPending)
            {
                bool notifyEnter2D = changed || !_collider2DNotified;
                _collider2DNotified = true;
                Collider2D.NotifyMixedContact(
                    Collider3D,
                    true,
                    notifyEnter2D,
                    _isTriggerPair,
                    allowInactive: false,
                    registration2D,
                    registration3D,
                    shouldRaiseTrigger);
            }
        }
        finally
        {
            EndNotification(
                registration3D,
                registration2D,
                shouldRaiseTrigger);
        }
    }

    public void MarkResting(int frame)
    {
        LastFrame = frame;
    }

    public void MarkResting(int frame, MixedContact contact)
    {
        Contact = contact;
        LastFrame = frame;
    }

    public void MarkSeparated()
    {
        // The 2D side is admitted only after the 3D side, so it cannot be the sole notified side.
        if (!_isColliding && !_collider3DNotified)
            return;

        _isColliding = false;
        Contact = default;
        if (_notificationInProgress)
        {
            _separationPending = true;
            return;
        }

        var registration3D = new ColliderLifetimeToken(Collider3D);
        var registration2D = new ColliderLifetimeToken2D(Collider2D);
        NotifySeparation(
            registration3D,
            registration2D,
            ColliderTriggerEventPolicy.ShouldRaise(Collider3D, Collider2D));
    }

    private void EndNotification(
        in ColliderLifetimeToken registration3D,
        in ColliderLifetimeToken2D registration2D,
        bool shouldRaiseTrigger)
    {
        _notificationInProgress = false;
        if (!_separationPending)
            return;

        _separationPending = false;
        NotifySeparation(registration3D, registration2D, shouldRaiseTrigger);
    }

    private void NotifySeparation(
        in ColliderLifetimeToken registration3D,
        in ColliderLifetimeToken2D registration2D,
        bool shouldRaiseTrigger)
    {
        bool notify2D = _collider2DNotified;
        _collider3DNotified = false;
        _collider2DNotified = false;
        Collider3D.NotifyMixedContact(
            Collider2D,
            false,
            true,
            _isTriggerPair,
            allowInactive: true,
            registration3D,
            registration2D,
            shouldRaiseTrigger);
        if (notify2D)
        {
            Collider2D.NotifyMixedContact(
                Collider3D,
                false,
                true,
                _isTriggerPair,
                allowInactive: true,
                registration2D,
                registration3D,
                shouldRaiseTrigger);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void WakeSleepingBodiesForCollision()
    {
        SolidBody? body3D = Collider3D.Body;
        SolidBody2D? body2D = Collider2D.Body;
        if (body3D == null || body2D == null)
            return;

        bool body3DAwake = body3D.IsAwakeForCollision;
        bool body2DAwake = body2D.IsAwakeForCollision;
        if (body3D.IsSleeping && body2DAwake)
            body3D.Wake();
        if (body2D.IsSleeping && body3DAwake)
            body2D.Wake();
    }
}
