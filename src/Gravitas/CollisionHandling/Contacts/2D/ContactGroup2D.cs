//=======================================================================
// ContactGroup2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>Two inline samples belonging to one independent planar surface region.</summary>
internal struct ContactGroup2D
{
    internal ContactGroupKey Key;
    internal int Count;
    private ManifoldContact2D _contact0, _contact1;

    internal ManifoldContact2D this[int index] => GetContactUnchecked(index);

    internal int Add(in ManifoldContact2D contact)
    {
        ulong contactId = contact.ContactId;
        for (int i = 0; i < Count; i++)
        {
            ManifoldContact2D existing = GetContactUnchecked(i);
            if (existing.ContactId != contactId)
                continue;

            if (IsDeeper(contact, existing))
                SetContactUnchecked(i, contact);
            SortContactsById();
            return 0;
        }

        if (Count < ContactManifold2D.MaxContactsPerGroup)
        {
            SetContactUnchecked(Count, contact);
            Count++;
            SortContactsById();
            return 1;
        }

        int replaceIndex = FindShallowestReplacementIndex(contact);
        if (replaceIndex < 0)
            return 0;

        SetContactUnchecked(replaceIndex, contact);
        SortContactsById();
        return 0;
    }

    private int FindShallowestReplacementIndex(ManifoldContact2D candidate)
    {
        int replaceIndex = !IsDeeper(_contact1, _contact0)
            ? 1
            : 0;

        ManifoldContact2D shallowest = GetContactUnchecked(replaceIndex);
        if (IsDeeper(candidate, shallowest))
            return replaceIndex;

        if (HasEqualDepth(candidate, shallowest) && candidate.ContactId < shallowest.ContactId)
            return replaceIndex;

        return -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsDeeper(ManifoldContact2D candidate, ManifoldContact2D existing) =>
        candidate.Depth > existing.Depth
        || candidate.Depth == existing.Depth
        && candidate.DepthIsClamped
        && !existing.DepthIsClamped;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasEqualDepth(ManifoldContact2D left, ManifoldContact2D right) =>
        left.Depth == right.Depth
        && left.DepthIsClamped == right.DepthIsClamped;

    private void SortContactsById()
    {
        if (Count < 2 || _contact0.ContactId <= _contact1.ContactId)
            return;

        (_contact0, _contact1) = (_contact1, _contact0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ManifoldContact2D GetContactUnchecked(int index) =>
        index == 0 ? _contact0 : _contact1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetContactUnchecked(int index, ManifoldContact2D contact)
    {
        if (index == 0)
        {
            _contact0 = contact;
            return;
        }

        _contact1 = contact;
    }

}
