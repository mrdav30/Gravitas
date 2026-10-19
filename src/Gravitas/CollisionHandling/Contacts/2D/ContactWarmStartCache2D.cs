//=======================================================================
// ContactWarmStartCache2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

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

/// <summary>Retained pair-owned warm starts with independent two-point groups.</summary>
internal struct ContactWarmStartCache2D
{
    private ContactWarmStartGroup2D first;
    private SwiftCollections.SwiftList<ContactWarmStartGroup2D>? additional;
    internal int GroupCount { get; private set; }
    public int Count
    {
        get
        {
            int count = 0;
            for (int i = 0; i < GroupCount; i++) count += i == 0 ? first.Count : additional!.InnerArray[i - 1].Count;
            return count;
        }
    }

    internal ContactWarmStartGroup2D GetGroup(int index) =>
        index == 0 ? first : additional!.InnerArray[index - 1];

    private void SetGroup(int index, in ContactWarmStartGroup2D value)
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

    internal void Set(in ContactGroupKey key, ulong contactId, Fixed64 normalImpulse, Fixed64 tangentImpulse)
    {
        int index = 0, order = -1;
        while (index < GroupCount)
        {
            // Key lookup must not copy the group's two retained impulse rows.
            ref readonly ContactWarmStartGroup2D candidate = ref (index == 0 ? ref first : ref additional!.InnerArray[index - 1]);
            order = candidate.Key.CompareTo(key);
            if (order >= 0) break;
            index++;
        }
        if (index == GroupCount || order != 0)
        {
            if (GroupCount > 0)
            {
                additional ??= new SwiftCollections.SwiftList<ContactWarmStartGroup2D>();
                additional.Add(default);
                for (int i = GroupCount; i > index; i--) SetGroup(i, GetGroup(i - 1));
            }
            SetGroup(index, new ContactWarmStartGroup2D { Key = key });
            GroupCount++;
        }
        ref ContactWarmStartGroup2D current = ref (index == 0 ? ref first : ref additional!.InnerArray[index - 1]);
        current.Set(contactId, normalImpulse, tangentImpulse);
    }

    internal bool TryGet(in ContactGroupKey key, ulong contactId, out ContactWarmStartImpulse impulse)
    {
        int index = Find(key);
        if (index >= 0)
            return index == 0 ? first.TryGet(contactId, out impulse)
                : additional!.InnerArray[index - 1].TryGet(contactId, out impulse);
        impulse = default;
        return false;
    }

    internal bool Remove(in ContactGroupKey key, ulong contactId)
    {
        int index = Find(key);
        if (index < 0) return false;
        ref ContactWarmStartGroup2D current = ref (index == 0 ? ref first : ref additional!.InnerArray[index - 1]);
        if (!current.Remove(contactId)) return false;
        if (current.Count == 0) RemoveGroup(index);
        return true;
    }

    internal void Retain(ContactManifold2D manifold)
    {
        for (int group = GroupCount - 1; group >= 0; group--)
        {
            ref ContactWarmStartGroup2D cached = ref (group == 0 ? ref first : ref additional!.InnerArray[group - 1]);
            int current = manifold.FindGroup(cached.Key);
            if (current < 0)
            {
                RemoveGroup(group);
                continue;
            }
            ref ContactGroup2D admitted = ref manifold.GetGroup(current);
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
            ref readonly ContactWarmStartGroup2D current = ref (middle == 0 ? ref first : ref additional!.InnerArray[middle - 1]);
            int order = current.Key.CompareTo(key);
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
