//=======================================================================
// LSCollider2D.Notifications.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

public abstract partial class LSCollider2D
{
    internal void NotifyContact(LSCollider2D other, bool isColliding, bool isChanged) =>
        NotifyContact(
            other,
            other.Body,
            isColliding,
            isChanged,
            allowInactive: false,
            new ColliderLifetimeToken2D(this),
            new ColliderLifetimeToken2D(other),
            IsTrigger || other.IsTrigger,
            ColliderTriggerEventPolicy.ShouldRaise(this, other));

    internal void NotifyContact(
        LSCollider2D other,
        SolidBody2D? otherBody,
        bool isColliding,
        bool isChanged,
        bool allowInactive,
        in ColliderLifetimeToken2D registration,
        in ColliderLifetimeToken2D otherRegistration,
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

    internal void NotifyMixedContact(LSCollider other, bool isColliding, bool isChanged, bool isTriggerPair) =>
        NotifyMixedContact(
            other,
            isColliding,
            isChanged,
            isTriggerPair,
            allowInactive: false,
            new ColliderLifetimeToken2D(this),
            new ColliderLifetimeToken(other),
            ColliderTriggerEventPolicy.ShouldRaise(other, this));

    internal void NotifyMixedContact(
        LSCollider other,
        bool isColliding,
        bool isChanged,
        bool isTriggerPair,
        bool allowInactive,
        in ColliderLifetimeToken2D registration,
        in ColliderLifetimeToken otherRegistration,
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
