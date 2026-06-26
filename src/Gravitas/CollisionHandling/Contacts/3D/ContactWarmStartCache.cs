//=======================================================================
// ContactWarmStartCache.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Fixed-size 3D warm-start cache keyed by stable manifold contact identity.
/// </summary>
internal struct ContactWarmStartCache
{
    private ulong _contactId0;
    private ulong _contactId1;
    private ulong _contactId2;
    private ulong _contactId3;
    private ContactWarmStartImpulse _impulse0;
    private ContactWarmStartImpulse _impulse1;
    private ContactWarmStartImpulse _impulse2;
    private ContactWarmStartImpulse _impulse3;

    public int Count { get; private set; }

    public void Clear()
    {
        Count = 0;
        _contactId0 = 0UL;
        _contactId1 = 0UL;
        _contactId2 = 0UL;
        _contactId3 = 0UL;
        _impulse0 = default;
        _impulse1 = default;
        _impulse2 = default;
        _impulse3 = default;
    }

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

        if (Count < ContactManifold.MaxContactCount)
        {
            SetContactIdUnchecked(Count, contactId);
            SetImpulseUnchecked(Count, impulse);
            Count++;
            return;
        }

        // Contact manifolds are already reduced to four stable contacts; replacement preserves bounded state if an older cache leaks through.
        SetContactIdUnchecked(ContactManifold.MaxContactCount - 1, contactId);
        SetImpulseUnchecked(ContactManifold.MaxContactCount - 1, impulse);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ulong GetContactId(int index) =>
        index switch
        {
            0 => _contactId0,
            1 => _contactId1,
            2 => _contactId2,
            _ => _contactId3
        };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ContactWarmStartImpulse GetImpulseUnchecked(int index) =>
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
