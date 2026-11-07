//=======================================================================
// ContactWarmStartCache.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;




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
            for (int i = 0; i < GroupCount; i++) count += i == 0 ? first.Count : additional!.InnerArray[i - 1].Count;
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
        int index = 0, order = -1;
        while (index < GroupCount)
        {
            // Key lookup must not copy the group's four retained impulse rows.
            ref readonly ContactWarmStartGroup candidate = ref (index == 0 ? ref first : ref additional!.InnerArray[index - 1]);
            order = candidate.Key.CompareTo(key);
            if (order >= 0) break;
            index++;
        }
        if (index == GroupCount || order != 0)
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
        if (current.Count == 0) RemoveGroup(index);
        return true;
    }

    internal void Retain(ContactManifold manifold)
    {
        for (int group = GroupCount - 1; group >= 0; group--)
        {
            ref ContactWarmStartGroup cached = ref (group == 0 ? ref first : ref additional!.InnerArray[group - 1]);
            int current = manifold.FindGroup(cached.Key);
            if (current < 0)
            {
                RemoveGroup(group);
                continue;
            }
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
            ref readonly ContactWarmStartGroup current = ref (middle == 0 ? ref first : ref additional!.InnerArray[middle - 1]);
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
