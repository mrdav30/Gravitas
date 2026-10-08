//=======================================================================
// ContactGroup.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>Four inline point samples belonging to one geometric region.</summary>
internal struct ContactGroup
{
    internal ContactGroupKey Key;
    internal int Count;
    private ManifoldContact _contact0, _contact1, _contact2, _contact3;

    internal int Add(in ManifoldContact contact)
    {
        for (int i = 0; i < Count; i++)
        {
            ManifoldContact existing = GetContactUnchecked(i);
            if (existing.ContactId != contact.ContactId)
                continue;
            if (IsDeeper(contact, existing))
                SetContactUnchecked(i, contact);
            SortContactsById();
            return 0;
        }
        if (Count < ContactManifold.MaxContactsPerGroup)
        {
            SetContactUnchecked(Count++, contact);
            SortContactsById();
            return 1;
        }
        int replacement = FindShallowestReplacementIndex(contact);
        if (replacement >= 0)
        {
            SetContactUnchecked(replacement, contact);
            SortContactsById();
        }
        return 0;
    }

    internal ManifoldContact this[int index] => GetContactUnchecked(index);

    private int FindShallowestReplacementIndex(ManifoldContact candidate)
    {
        int replaceIndex = 0;
        ManifoldContact shallowest = _contact0;

        for (int i = 1; i < Count; i++)
        {
            ManifoldContact contact = GetContactUnchecked(i);
            // Samples are already sorted by identity; a later equal-depth
            // sample loses the deterministic identity tie.
            if (IsDeeper(shallowest, contact) || HasEqualDepth(contact, shallowest))
            {
                shallowest = contact;
                replaceIndex = i;
            }
        }

        if (IsDeeper(candidate, shallowest))
            return replaceIndex;

        if (HasEqualDepth(candidate, shallowest) && candidate.ContactId < shallowest.ContactId)
            return replaceIndex;

        return -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsDeeper(ManifoldContact candidate, ManifoldContact existing) =>
        candidate.Depth > existing.Depth
        || candidate.Depth == existing.Depth
        && candidate.DepthIsClamped
        && !existing.DepthIsClamped;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasEqualDepth(ManifoldContact left, ManifoldContact right) =>
        left.Depth == right.Depth
        && left.DepthIsClamped == right.DepthIsClamped;

    private void SortContactsById()
    {
        for (int i = 1; i < Count; i++)
        {
            ManifoldContact contact = GetContactUnchecked(i);
            int j = i - 1;
            while (j >= 0 && GetContactUnchecked(j).ContactId > contact.ContactId)
            {
                SetContactUnchecked(j + 1, GetContactUnchecked(j));
                j--;
            }

            SetContactUnchecked(j + 1, contact);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ManifoldContact GetContactUnchecked(int index) =>
        index switch
        {
            0 => _contact0,
            1 => _contact1,
            2 => _contact2,
            _ => _contact3
        };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetContactUnchecked(int index, ManifoldContact contact)
    {
        switch (index)
        {
            case 0:
                _contact0 = contact;
                break;
            case 1:
                _contact1 = contact;
                break;
            case 2:
                _contact2 = contact;
                break;
            default:
                _contact3 = contact;
                break;
        }
    }

}
