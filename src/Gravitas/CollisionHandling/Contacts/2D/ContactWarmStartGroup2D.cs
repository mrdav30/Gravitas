//=======================================================================
// ContactWarmStartGroup2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;
using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Two inline warm-start rows belonging to one planar surface region.
/// </summary>
internal struct ContactWarmStartGroup2D
{
    internal ContactGroupKey Key;
    private ulong _contactId0;
    private ulong _contactId1;
    private ContactWarmStartImpulse _impulse0;
    private ContactWarmStartImpulse _impulse1;

    public int Count { get; private set; }

    public void Set(ulong contactId, Fixed64 normalImpulse, Fixed64 tangentImpulse)
    {
        ContactWarmStartImpulse impulse = new(normalImpulse, tangentImpulse);
        for (int i = 0; i < Count; i++)
        {
            if (GetContactId(i) != contactId)
                continue;

            SetImpulseUnchecked(i, impulse);
            return;
        }

        if (Count < ContactManifold2D.MaxContactsPerGroup)
        {
            SetContactIdUnchecked(Count, contactId);
            SetImpulseUnchecked(Count, impulse);
            Count++;
            return;
        }

        SetContactIdUnchecked(ContactManifold2D.MaxContactsPerGroup - 1, contactId);
        SetImpulseUnchecked(ContactManifold2D.MaxContactsPerGroup - 1, impulse);
    }

    public bool TryGet(ulong contactId, out ContactWarmStartImpulse impulse)
    {
        for (int i = 0; i < Count; i++)
        {
            if (GetContactId(i) != contactId)
                continue;

            impulse = GetImpulseUnchecked(i);
            return true;
        }

        impulse = default;
        return false;
    }

    public bool Remove(ulong contactId)
    {
        for (int i = 0; i < Count; i++)
        {
            if (GetContactId(i) != contactId)
                continue;

            Count--;
            for (int shift = i; shift < Count; shift++)
            {
                SetContactIdUnchecked(
                    shift,
                    GetContactId(shift + 1));
                SetImpulseUnchecked(
                    shift,
                    GetImpulseUnchecked(shift + 1));
            }

            SetContactIdUnchecked(Count, 0UL);
            SetImpulseUnchecked(Count, default);
            return true;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ulong GetContactId(int index) => index == 0 ? _contactId0 : _contactId1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ContactWarmStartImpulse GetImpulseUnchecked(int index) => index == 0 ? _impulse0 : _impulse1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetContactIdUnchecked(int index, ulong contactId)
    {
        if (index == 0)
        {
            _contactId0 = contactId;
            return;
        }

        _contactId1 = contactId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetImpulseUnchecked(int index, ContactWarmStartImpulse impulse)
    {
        if (index == 0)
        {
            _impulse0 = impulse;
            return;
        }

        _impulse1 = impulse;
    }
}
