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


/// <summary>Retained pair-owned warm starts with independent four-point groups.</summary>
internal struct ContactWarmStartCache
{
    private ContactWarmStartGroup first;
    private SwiftCollections.SwiftList<ContactWarmStartGroup>? additional;
    internal int GroupCount { get; private set; }
    public int Count
    {
        get
        {
            int count = 0;
            for (int i = 0; i < GroupCount; i++) count += GetGroup(i).Count;
            return count;
        }
    }

    internal ContactWarmStartGroup GetGroup(int index) =>
        index == 0 ? first : additional!.InnerArray[index - 1];

    private void SetGroup(int index, in ContactWarmStartGroup value)
    {
        if (index == 0) first = value;
        else additional!.InnerArray[index - 1] = value;
    }

    public void Clear()
    {
        GroupCount = 0;
        first = default;
        additional?.FastClear();
    }

    public void Set(ulong contactId, Vector3d normal, Fixed64 normalImpulse,
        Fixed64 tangentImpulse, Fixed64 secondaryTangentImpulse = default) =>
        Set(default, contactId, normal, normalImpulse, tangentImpulse, secondaryTangentImpulse);

    internal void Set(in ContactGroupKey key, ulong contactId, Vector3d normal,
        Fixed64 normalImpulse, Fixed64 tangentImpulse, Fixed64 secondaryTangentImpulse)
    {
        int index = 0;
        while (index < GroupCount && GetGroup(index).Key.CompareTo(key) < 0) index++;
        if (index == GroupCount || GetGroup(index).Key.CompareTo(key) != 0)
        {
            if (GroupCount > 0)
            {
                additional ??= new SwiftCollections.SwiftList<ContactWarmStartGroup>();
                additional.Add(default);
                for (int i = GroupCount; i > index; i--) SetGroup(i, GetGroup(i - 1));
            }
            SetGroup(index, new ContactWarmStartGroup { Key = key });
            GroupCount++;
        }
        ref ContactWarmStartGroup current = ref (index == 0 ? ref first : ref additional!.InnerArray[index - 1]);
        current.Set(contactId, normal, normalImpulse, tangentImpulse, secondaryTangentImpulse);
    }

    public bool TryGet(ulong contactId, out ContactWarmStartImpulse impulse) =>
        TryGet(default, contactId, out impulse);

    internal bool TryGet(in ContactGroupKey key, ulong contactId, out ContactWarmStartImpulse impulse)
    {
        int index = Find(key);
        if (index >= 0)
            return index == 0 ? first.TryGet(contactId, out impulse)
                : additional!.InnerArray[index - 1].TryGet(contactId, out impulse);
        impulse = default;
        return false;
    }

    public bool Remove(ulong contactId) => Remove(default, contactId);

    internal bool Remove(in ContactGroupKey key, ulong contactId)
    {
        int index = Find(key);
        if (index < 0) return false;
        ref ContactWarmStartGroup current = ref (index == 0 ? ref first : ref additional!.InnerArray[index - 1]);
        if (!current.Remove(contactId)) return false;
        if (GetGroup(index).Count == 0) RemoveGroup(index);
        return true;
    }

    internal void Retain(ContactManifold manifold)
    {
        for (int group = GroupCount - 1; group >= 0; group--)
        {
            int current = manifold.FindGroup(GetGroup(group).Key);
            if (current < 0)
            {
                RemoveGroup(group);
                continue;
            }
            ref ContactWarmStartGroup cached = ref (group == 0 ? ref first : ref additional!.InnerArray[group - 1]);
            ref ContactGroup admitted = ref manifold.GetGroup(current);
            for (int point = cached.Count - 1; point >= 0; point--)
            {
                ulong id = cached.GetContactId(point);
                bool present = false;
                for (int i = 0; i < admitted.Count; i++) present |= admitted[i].ContactId == id;
                if (!present) cached.Remove(id);
            }
            if (cached.Count == 0) RemoveGroup(group);
        }
    }

    private int Find(in ContactGroupKey key)
    {
        int low = 0, high = GroupCount - 1;
        while (low <= high)
        {
            int middle = low + ((high - low) >> 1);
            int order = GetGroup(middle).Key.CompareTo(key);
            if (order == 0) return middle;
            if (order < 0) low = middle + 1;
            else high = middle - 1;
        }
        return -1;
    }

    private void RemoveGroup(int index)
    {
        for (int i = index; i < GroupCount - 1; i++) SetGroup(i, GetGroup(i + 1));
        if (GroupCount > 1) additional!.RemoveAt(additional.Count - 1);
        else first = default;
        GroupCount--;
    }

}
