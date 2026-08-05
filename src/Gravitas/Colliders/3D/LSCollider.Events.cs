//=======================================================================
// LSCollider.Events.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas.Colliders;

internal readonly struct ColliderLifetimeToken
{
    internal ColliderLifetimeToken(LSCollider collider)
    {
        Collider = collider;
        LifetimeVersion = collider.LifetimeVersion;
    }

    internal LSCollider Collider { get; }

    internal long LifetimeVersion { get; }

    internal bool IsActive => Collider.IsActive
        && !Collider.IsDeactivationInProgress
        && IsCurrentLifetime;

    internal bool IsCurrentLifetime => Collider.LifetimeVersion == LifetimeVersion;
}

public abstract partial class LSCollider
{
    internal int CollisionPairCount => _pairState.CollisionPairCount;

    internal int CollisionPairHolderCount => _pairState.CollisionPairHolderCount;

    internal SwiftDictionary<int, CollisionPair>? CollisionPairs => _pairState.CollisionPairs;

    internal SwiftHashSet<int>? CollisionPairHolders => _pairState.CollisionPairHolders;

    /// <summary>Handles a 3D contact notification for the other body.</summary>
    public delegate void BodyCollisionFunc(SolidBody other);
    /// <summary>Raised while this collider is touching another collider that owns a 3D body.</summary>
    public event BodyCollisionFunc? OnContact;
    /// <summary>Raised on the first simulation frame this collider touches another 3D body.</summary>
    public event BodyCollisionFunc? OnContactEnter;
    /// <summary>Raised when this collider stops touching another 3D body.</summary>
    public event BodyCollisionFunc? OnContactExit;

    /// <summary>Handles a 3D trigger notification for the other collider.</summary>
    public delegate void TriggerCollisionFunc(LSCollider other);

    /// <summary>
    /// Raised on the first simulation frame this collider participates in a valid trigger pair.
    /// </summary>
    public event TriggerCollisionFunc? OnTriggerEnter;

    /// <summary>
    /// Raised each simulation frame this collider participates in an overlapped valid trigger pair.
    /// </summary>
    public event TriggerCollisionFunc? OnTriggerStay;

    /// <summary>
    /// Raised when this collider stops participating in a valid trigger pair.
    /// </summary>
    public event TriggerCollisionFunc? OnTriggerExit;

    /// <summary>Handles a mixed contact or trigger notification for the other 2D collider.</summary>
    public delegate void MixedCollisionFunc(LSCollider2D other);
    /// <summary>Raised while this collider has a physical mixed contact with a 2D collider.</summary>
    public event MixedCollisionFunc? OnMixedContact;
    /// <summary>Raised when this collider begins a physical mixed contact with a 2D collider.</summary>
    public event MixedCollisionFunc? OnMixedContactEnter;
    /// <summary>Raised when this collider ends a physical mixed contact with a 2D collider.</summary>
    public event MixedCollisionFunc? OnMixedContactExit;

    /// <summary>
    /// Raised on the first mixed 3D/2D simulation frame this collider participates in a valid trigger pair.
    /// </summary>
    public event MixedCollisionFunc? OnMixedTriggerEnter;

    /// <summary>
    /// Raised each mixed 3D/2D simulation frame this collider participates in an overlapped valid trigger pair.
    /// </summary>
    public event MixedCollisionFunc? OnMixedTriggerStay;

    /// <summary>
    /// Raised when this collider stops participating in a valid mixed 3D/2D trigger pair.
    /// </summary>
    public event MixedCollisionFunc? OnMixedTriggerExit;

    internal void NotifyContact(LSCollider other, bool isColliding, bool isChanged) =>
        NotifyContact(
            other,
            other.Body,
            isColliding,
            isChanged,
            allowInactive: false,
            new ColliderLifetimeToken(this),
            new ColliderLifetimeToken(other),
            IsTrigger || other.IsTrigger,
            ColliderTriggerEventPolicy.ShouldRaise(this, other));

    internal void NotifyContact(
        LSCollider other,
        SolidBody? otherBody,
        bool isColliding,
        bool isChanged,
        bool allowInactive,
        in ColliderLifetimeToken registration,
        in ColliderLifetimeToken otherRegistration,
        bool isTriggerPair,
        bool shouldRaiseTrigger)
    {
        if (isColliding
            ? !registration.IsActive || !otherRegistration.IsActive
            : allowInactive ? !registration.IsCurrentLifetime : !registration.IsActive)
        {
            return;
        }

        if (isColliding)
        {
            if (isTriggerPair)
            {
                if (shouldRaiseTrigger)
                {
                    if (isChanged)
                    {
                        OnTriggerEnter?.Invoke(other);
                        if (!registration.IsActive || !otherRegistration.IsActive)
                            return;
                    }

                    OnTriggerStay?.Invoke(other);
                }

                return;
            }

            if (isChanged && otherBody != null)
            {
                OnContactEnter?.Invoke(otherBody);
                if (!registration.IsActive || !otherRegistration.IsActive)
                    return;
            }

            if (otherBody != null)
                OnContact?.Invoke(otherBody);

            return;
        }

        if (!isChanged)
            return;

        if (isTriggerPair)
        {
            if (shouldRaiseTrigger)
                OnTriggerExit?.Invoke(other);

            return;
        }

        if (otherBody != null)
            OnContactExit?.Invoke(otherBody);
    }

    internal void NotifyMixedContact(LSCollider2D other, bool isColliding, bool isChanged, bool isTriggerPair) =>
        NotifyMixedContact(
            other,
            isColliding,
            isChanged,
            isTriggerPair,
            allowInactive: false,
            new ColliderLifetimeToken(this),
            new ColliderLifetimeToken2D(other),
            ColliderTriggerEventPolicy.ShouldRaise(this, other));

    internal void NotifyMixedContact(
        LSCollider2D other,
        bool isColliding,
        bool isChanged,
        bool isTriggerPair,
        bool allowInactive,
        in ColliderLifetimeToken registration,
        in ColliderLifetimeToken2D otherRegistration,
        bool shouldRaiseTrigger)
    {
        if (isColliding
            ? !registration.IsActive || !otherRegistration.IsActive
            : allowInactive
                ? !registration.IsCurrentLifetime || !otherRegistration.IsCurrentLifetime
                : !registration.IsActive)
        {
            return;
        }

        if (isColliding)
        {
            if (isTriggerPair)
            {
                if (shouldRaiseTrigger)
                {
                    if (isChanged)
                    {
                        OnMixedTriggerEnter?.Invoke(other);
                        if (!registration.IsActive || !otherRegistration.IsActive)
                            return;
                    }

                    OnMixedTriggerStay?.Invoke(other);
                }

                return;
            }

            if (isChanged)
            {
                OnMixedContactEnter?.Invoke(other);
                if (!registration.IsActive || !otherRegistration.IsActive)
                    return;
            }

            OnMixedContact?.Invoke(other);
            return;
        }

        if (!isChanged)
            return;

        if (isTriggerPair)
        {
            if (shouldRaiseTrigger)
                OnMixedTriggerExit?.Invoke(other);

            return;
        }

        OnMixedContactExit?.Invoke(other);
    }
}
