//=======================================================================
// ContactWarmStartGroup.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;
using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Four cached point impulses belonging to one geometric surface group.
/// </summary>
internal struct ContactWarmStartGroup
{
    internal ContactGroupKey Key;
    private ulong _contactId0;
    private ulong _contactId1;
    private ulong _contactId2;
    private ulong _contactId3;
    private ContactWarmStartImpulse _impulse0;
    private ContactWarmStartImpulse _impulse1;
    private ContactWarmStartImpulse _impulse2;
    private ContactWarmStartImpulse _impulse3;

    public int Count { get; private set; }

    public void Set(
        ulong contactId,
        Vector3d normal,
        Fixed64 normalImpulse,
        Fixed64 tangentImpulse,
        Fixed64 secondaryTangentImpulse = default)
    {
        ContactWarmStartImpulse impulse = new(normal, normalImpulse, tangentImpulse, secondaryTangentImpulse);
        for (int i = 0; i < Count; i++)
        {
            if (GetContactId(i) != contactId)
                continue;

            SetImpulseUnchecked(i, impulse);
            return;
        }

        // Retain() removes vanished features before solving. A full stale cache
        // still stays bounded; insert by identity so replay is independent of discovery.
        int insertion = 0;
        while (insertion < Count && GetContactId(insertion) < contactId) insertion++;
        int last = Count < ContactManifold.MaxContactsPerGroup
            ? Count++ : ContactManifold.MaxContactsPerGroup - 1;
        if (insertion > last) insertion = last;
        for (int i = last; i > insertion; i--)
        {
            SetContactIdUnchecked(i, GetContactId(i - 1));
            SetImpulseUnchecked(i, GetImpulseUnchecked(i - 1));
        }
        SetContactIdUnchecked(insertion, contactId);
        SetImpulseUnchecked(insertion, impulse);
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
        for (int index = 0; index < Count; index++)
        {
            if (GetContactId(index) != contactId)
                continue;

            Count--;
            for (int shift = index; shift < Count; shift++)
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
    internal ulong GetContactId(int index) =>
        index switch
        {
            0 => _contactId0,
            1 => _contactId1,
            2 => _contactId2,
            _ => _contactId3
        };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ContactWarmStartImpulse GetImpulseUnchecked(int index) =>
        index switch
        {
            0 => _impulse0,
            1 => _impulse1,
            2 => _impulse2,
            _ => _impulse3
        };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetContactIdUnchecked(int index, ulong contactId)
    {
        switch (index)
        {
            case 0:
                _contactId0 = contactId;
                break;
            case 1:
                _contactId1 = contactId;
                break;
            case 2:
                _contactId2 = contactId;
                break;
            default:
                _contactId3 = contactId;
                break;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetImpulseUnchecked(int index, ContactWarmStartImpulse impulse)
    {
        switch (index)
        {
            case 0:
                _impulse0 = impulse;
                break;
            case 1:
                _impulse1 = impulse;
                break;
            case 2:
                _impulse2 = impulse;
                break;
            default:
                _impulse3 = impulse;
                break;
        }
    }
}
