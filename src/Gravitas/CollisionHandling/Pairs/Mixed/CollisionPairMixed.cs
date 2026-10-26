//=======================================================================
// CollisionPairMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using FixedMathSharp;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Owns one stable mixed 3D/2D collision pair identity and contact lifecycle.
/// </summary>
/// <content>Owns retained mixed manifold rows, per-step impulses, notifications and replay state.</content>
internal sealed partial class CollisionPairMixed
{
    private bool _isColliding;
    private bool _isTriggerPair;
    private bool _notificationInProgress;
    private bool _separationPending;
    private bool _collider3DNotified;
    private bool _collider2DNotified;
    private long _lifetimeVersion;
    private uint _lastCollider3DContactDependencyVersion;
    private uint _lastCollider2DContactDependencyVersion;
    private bool _hasManifoldCache;
    private uint _manifold3DBroadPhaseVersion, _manifold2DBroadPhaseVersion;
    private uint _manifold3DContactDependencyVersion, _manifold2DContactDependencyVersion;
    private Vector2d _manifold2DCenter;
    private Fixed64 _manifold2DRotation;

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

    public long LastFrame { get; private set; } = -1;

    // Collision-pass freshness is independent of hosts advancing the public frame clock.
    internal long LastVisitToken { get; private set; } = -1;

    public MixedContact Contact { get; private set; }

    public ContactManifold Manifold { get; } = new();

    // Accumulation belongs to one solve pass; mixed pairs do not warm-start across frames.
    internal ContactWarmStartCache SolveImpulses;

    internal ExactContactImpulseState?[]? ExactSolveImpulses;

    internal ContactResponseSnapshot ResponseSnapshot { get; set; }

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
        LastVisitToken = -1;
        _isColliding = false;
        _hasManifoldCache = false;
        _lastCollider3DContactDependencyVersion = 0;
        _lastCollider2DContactDependencyVersion = 0;
        _isTriggerPair = collider3D.IsTrigger || collider2D.IsTrigger;
        _notificationInProgress = false;
        _separationPending = false;
        _collider3DNotified = false;
        _collider2DNotified = false;
        Contact = default;
        Manifold.Reset();
        ClearSolveState();
    }

    public void MarkColliding(long frame, MixedContact contact)
    {
        PublishContact(frame, contact);
        NotifyCollision(frame);
    }

    public void MarkColliding(long frame, ContactManifold manifold)
    {
        PublishManifold(frame, manifold);
        NotifyCollision(frame);
    }

    private void NotifyCollision(long frame)
    {
        bool changed = !_isColliding;
        _isColliding = true;
        _isTriggerPair = Collider3D.IsTrigger || Collider2D.IsTrigger;
        LastFrame = frame;
        LastVisitToken = Context.LateSimulateToken;

        Context.Diagnostics.EmitMixedContact(this, Contact);

        var registration3D = new ColliderLifetimeToken(Collider3D);
        var registration2D = new ColliderLifetimeToken2D(Collider2D);
        bool shouldRaiseTrigger = ColliderTriggerEventPolicy.ShouldRaise(Collider3D, Collider2D);
        _notificationInProgress = true;
        SwiftList<Exception>? notificationExceptions = null;
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
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
        }

        try
        {
            EndNotification(
                registration3D,
                registration2D,
                shouldRaiseTrigger);
        }
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
        }

        CollisionNotificationExceptions.ThrowIfAny(notificationExceptions);
    }

    public void MarkResting(long frame)
    {
        _hasManifoldCache = false;
        RefreshContactDependencies();
        LastFrame = frame;
        LastVisitToken = Context.LateSimulateToken;
        ClearSolveState();
    }

    public void MarkResting(long frame, ContactManifold manifold)
    {
        PublishManifold(frame, manifold);
        LastFrame = frame;
        LastVisitToken = Context.LateSimulateToken;
    }

    private void PublishContact(long frame, MixedContact contact)
    {
        _hasManifoldCache = false;
        RefreshContactDependencies(incomingPlanar: contact.HasContact && contact.Normal3DTo2D.ToVector2d() != Vector2d.Zero);
        Manifold.BeginUpdate(frame);
        if (contact.HasContact)
            Manifold.AddContact(default, contact.ToManifoldContact());
        Contact = contact;
        ClearSolveState();
    }

    private void PublishManifold(long frame, ContactManifold source)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        _hasManifoldCache = false;
        RefreshContactDependencies(source);
        Manifold.CopyFrom(source, frame);
        Contact = Manifold.HasContact ? MixedContact.FromManifoldContact(Manifold.PrimaryContact) : default;
        _manifold3DBroadPhaseVersion = Collider3D.BroadPhaseVersion;
        _manifold2DBroadPhaseVersion = Collider2D.BroadPhaseVersion;
        _manifold3DContactDependencyVersion = Collider3D.ContactDependencyVersion;
        _manifold2DContactDependencyVersion = Collider2D.ContactDependencyVersion;
        _manifold2DCenter = Collider2D.Center;
        _manifold2DRotation = Collider2D.Rotation;
        _hasManifoldCache = true;
        ClearSolveState();
    }

    // Dynamic 2D pose can change without a partition or contact-dependency revision.
    internal bool HasCurrentManifoldCache => _isColliding && _hasManifoldCache
        && _manifold3DBroadPhaseVersion == Collider3D.BroadPhaseVersion
        && _manifold2DBroadPhaseVersion == Collider2D.BroadPhaseVersion
        && _manifold3DContactDependencyVersion == Collider3D.ContactDependencyVersion
        && _manifold2DContactDependencyVersion == Collider2D.ContactDependencyVersion
        && _manifold2DCenter == Collider2D.Center
        && _manifold2DRotation == Collider2D.Rotation;

    internal bool TryCopyCachedManifold(long frame, ContactManifold target)
    {
        if (!HasCurrentManifoldCache) return false;
        target.CopyFrom(Manifold, frame);
        return true;
    }

    private void ClearSolveState()
    {
        SolveImpulses.Clear();
        if (ExactSolveImpulses != null)
            foreach (ExactContactImpulseState? state in ExactSolveImpulses) state?.Clear();
        ResponseSnapshot = default;
    }
    public void MarkSeparated()
    {
        _hasManifoldCache = false;
        if (_isColliding && !_isTriggerPair)
            WakeBodiesForContactChange();
        Manifold.Reset();
        ClearSolveState();
        Contact = default;
        // The 2D side is admitted only after the 3D side, so it cannot be the sole notified side.
        if (!_isColliding && !_collider3DNotified)
            return;

        _isColliding = false;
        if (_notificationInProgress)
        {
            _separationPending = true;
            return;
        }

        var registration3D = new ColliderLifetimeToken(Collider3D);
        var registration2D = new ColliderLifetimeToken2D(Collider2D);
        _notificationInProgress = true;
        try
        {
            NotifySeparation(
                registration3D,
                registration2D,
                ColliderTriggerEventPolicy.ShouldRaise(Collider3D, Collider2D));
        }
        finally
        {
            _separationPending = false;
            _notificationInProgress = false;
        }
    }

    private void EndNotification(
        in ColliderLifetimeToken registration3D,
        in ColliderLifetimeToken2D registration2D,
        bool shouldRaiseTrigger)
    {
        try
        {
            if (!_separationPending)
                return;

            _separationPending = false;
            NotifySeparation(registration3D, registration2D, shouldRaiseTrigger);
        }
        finally
        {
            _separationPending = false;
            _notificationInProgress = false;
        }
    }

    private void NotifySeparation(
        in ColliderLifetimeToken registration3D,
        in ColliderLifetimeToken2D registration2D,
        bool shouldRaiseTrigger)
    {
        bool notify2D = _collider2DNotified;
        _collider3DNotified = false;
        _collider2DNotified = false;
        SwiftList<Exception>? notificationExceptions = null;
        try
        {
            Collider3D.NotifyMixedContact(
                Collider2D,
                false,
                true,
                _isTriggerPair,
                allowInactive: true,
                registration3D,
                registration2D,
                shouldRaiseTrigger);
        }
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
        }

        if (notify2D && registration2D.IsCurrentLifetime)
        {
            try
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
            catch (Exception exception)
            {
                CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
            }
        }

        CollisionNotificationExceptions.ThrowIfAny(notificationExceptions);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RefreshContactDependencies(ContactManifold? incoming = null, bool incomingPlanar = false)
    {
        uint version3D = Collider3D.ContactDependencyVersion;
        uint version2D = Collider2D.ContactDependencyVersion;
        bool triggerPair = Collider3D.IsTrigger || Collider2D.IsTrigger;
        // Replacing a planar constraint with a vertical one still invalidates
        // its former 2D dependency. Purely vertical rows never couple 2D wake.
        if (_isColliding)
        {
            if (triggerPair != _isTriggerPair) WakeBodiesForContactChange(incoming, incomingPlanar);
            else if (!triggerPair)
            {
                if (version2D != _lastCollider2DContactDependencyVersion) Collider3D.Body?.WakeForContactChange();
                if (version3D != _lastCollider3DContactDependencyVersion)
                    Wake2DBodyForContactChange(incoming, incomingPlanar);
            }
        }

        _lastCollider3DContactDependencyVersion = version3D;
        _lastCollider2DContactDependencyVersion = version2D;
        _isTriggerPair = triggerPair;
    }

    private void WakeBodiesForContactChange(ContactManifold? incoming = null, bool incomingPlanar = false)
    {
        Collider3D.Body?.WakeForContactChange();
        Wake2DBodyForContactChange(incoming, incomingPlanar);
    }

    private void Wake2DBodyForContactChange(ContactManifold? incoming, bool incomingPlanar)
    {
        SolidBody2D? body = Collider2D.Body;
        if (body?.IsSleeping != true)
            return;

        if (incomingPlanar || CollisionResponseMixed.HasPlanarResponseCoupling(Manifold)
            || (incoming != null && CollisionResponseMixed.HasPlanarResponseCoupling(incoming)))
            body.WakeForContactChange();
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
