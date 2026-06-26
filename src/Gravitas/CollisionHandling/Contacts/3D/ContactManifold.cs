//=======================================================================
// ContactManifold.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Fixed-capacity deterministic contact manifold owned by one collision pair.
/// </summary>
public sealed class ContactManifold : IEnumerable<ManifoldContact>
{
    public const int MaxContactCount = 4;

    private ManifoldContact _contact0;
    private ManifoldContact _contact1;
    private ManifoldContact _contact2;
    private ManifoldContact _contact3;
    private int _count;
    private int _lastUpdatedFrame = -1;

    /// <summary>
    /// Number of active contacts in this manifold.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _count;
    }

    /// <summary>
    /// Gets whether this manifold currently contains narrow-phase contact data.
    /// </summary>
    public bool HasContact
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _count > 0;
    }

    /// <summary>
    /// Simulation frame in which the active contacts were last rebuilt.
    /// </summary>
    public int LastUpdatedFrame
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _lastUpdatedFrame;
    }

    /// <summary>
    /// Deepest contact in the manifold. Ties use the lowest contact identity.
    /// </summary>
    public ManifoldContact PrimaryContact
    {
        get
        {
            SwiftThrowHelper.ThrowIfListIndexInvalid(0, _count);

            int bestIndex = 0;
            ManifoldContact best = _contact0;
            for (int i = 1; i < _count; i++)
            {
                ManifoldContact candidate = this[i];
                if (candidate.Depth > best.Depth
                    || candidate.Depth == best.Depth && candidate.ContactId < best.ContactId)
                {
                    best = candidate;
                    bestIndex = i;
                }
            }

            return this[bestIndex];
        }
    }

    public ManifoldContact this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            SwiftThrowHelper.ThrowIfListIndexInvalid(index, _count);
            return GetContactUnchecked(index);
        }
    }

    /// <summary>
    /// Clears contacts and records the frame for a new narrow-phase pass.
    /// </summary>
    public void BeginUpdate(int frame)
    {
        _count = 0;
        _lastUpdatedFrame = frame;
    }

    /// <summary>
    /// Clears all contact data.
    /// </summary>
    public void Reset()
    {
        _count = 0;
        _lastUpdatedFrame = -1;
        _contact0 = default;
        _contact1 = default;
        _contact2 = default;
        _contact3 = default;
    }

    /// <summary>
    /// Replaces the manifold with one contact.
    /// </summary>
    public void SetContact(Vector3d pointA, Vector3d pointB, Fixed64 depth, Vector3d normal)
    {
        _count = 0;
        AddContact(pointA, pointB, depth, normal);
    }

    /// <summary>
    /// Adds a contact, keeping the deepest four contacts and exposing them by stable contact identity.
    /// </summary>
    public void AddContact(Vector3d pointA, Vector3d pointB, Fixed64 depth, Vector3d normal)
    {
        ulong contactId = CreateContactId(pointA, pointB);
        var contact = new ManifoldContact(contactId, pointA, pointB, depth, normal);

        for (int i = 0; i < _count; i++)
        {
            ManifoldContact existing = GetContactUnchecked(i);
            if (existing.ContactId != contactId)
                continue;

            if (contact.Depth > existing.Depth)
                SetContactUnchecked(i, contact);
            SortContactsById();
            return;
        }

        if (_count < MaxContactCount)
        {
            SetContactUnchecked(_count, contact);
            _count++;
            SortContactsById();
            return;
        }

        int replaceIndex = FindShallowestReplacementIndex(contact);
        if (replaceIndex < 0)
            return;

        SetContactUnchecked(replaceIndex, contact);
        SortContactsById();
    }

    public void SetImmovableDirection(Vector3d direction)
    {
        for (int i = 0; i < _count; i++)
            SetContactUnchecked(i, GetContactUnchecked(i).WithImmovableDirection(direction));
    }

    public Enumerator GetEnumerator() => new(this);

    IEnumerator<ManifoldContact> IEnumerable<ManifoldContact>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    private int FindShallowestReplacementIndex(ManifoldContact candidate)
    {
        int replaceIndex = 0;
        ManifoldContact shallowest = _contact0;

        for (int i = 1; i < _count; i++)
        {
            ManifoldContact contact = GetContactUnchecked(i);
            if (contact.Depth < shallowest.Depth
                || contact.Depth == shallowest.Depth && contact.ContactId > shallowest.ContactId)
            {
                shallowest = contact;
                replaceIndex = i;
            }
        }

        if (candidate.Depth > shallowest.Depth)
            return replaceIndex;

        if (candidate.Depth == shallowest.Depth && candidate.ContactId < shallowest.ContactId)
            return replaceIndex;

        return -1;
    }

    private void SortContactsById()
    {
        for (int i = 1; i < _count; i++)
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

    private static ulong CreateContactId(Vector3d pointA, Vector3d pointB)
    {
        if (CompareVector(pointB, pointA) < 0)
            (pointA, pointB) = (pointB, pointA);

        ulong hash = 14695981039346656037UL;
        Mix(ref hash, pointA.X.m_rawValue);
        Mix(ref hash, pointA.Y.m_rawValue);
        Mix(ref hash, pointA.Z.m_rawValue);
        Mix(ref hash, pointB.X.m_rawValue);
        Mix(ref hash, pointB.Y.m_rawValue);
        Mix(ref hash, pointB.Z.m_rawValue);
        return hash;
    }

    private static int CompareVector(Vector3d left, Vector3d right)
    {
        int compare = left.X.m_rawValue.CompareTo(right.X.m_rawValue);
        if (compare != 0)
            return compare;

        compare = left.Y.m_rawValue.CompareTo(right.Y.m_rawValue);
        if (compare != 0)
            return compare;

        return left.Z.m_rawValue.CompareTo(right.Z.m_rawValue);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Mix(ref ulong hash, long value)
    {
        unchecked
        {
            hash ^= (ulong)value;
            hash *= 1099511628211UL;
        }
    }

    public struct Enumerator : IEnumerator<ManifoldContact>
    {
        private readonly ContactManifold _manifold;
        private int _index;

        internal Enumerator(ContactManifold manifold)
        {
            _manifold = manifold;
            _index = -1;
        }

        public ManifoldContact Current => _manifold[_index];

        object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            int next = _index + 1;
            if (next >= _manifold._count)
                return false;

            _index = next;
            return true;
        }

        public void Reset() => _index = -1;

        public void Dispose() { }
    }
}
