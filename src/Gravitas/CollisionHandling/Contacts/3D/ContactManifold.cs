//=======================================================================
// ContactManifold.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Fixed-capacity deterministic contact manifold owned by one collision pair.
/// </summary>
public sealed class ContactManifold : IEnumerable<ManifoldContact>
{
    /// <summary>Maximum number of contacts retained by a 3D manifold.</summary>
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

    /// <summary>Gets the contact at the specified deterministic order index.</summary>
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
    /// Replaces the manifold with one rigid-frame contact.
    /// </summary>
    public void SetContact(
        ContactAnchor anchorA,
        ContactAnchor anchorB,
        Fixed64 depth,
        Vector3d normal,
        bool depthIsClamped = false)
    {
        _count = 0;
        AddContact(anchorA, anchorB, depth, normal, depthIsClamped);
    }

    /// <summary>
    /// Adds a contact, keeping the deepest four contacts and exposing them by stable contact identity.
    /// </summary>
    public void AddContact(Vector3d pointA, Vector3d pointB, Fixed64 depth, Vector3d normal)
    {
        AddContact(
            ContactAnchor.FromWorldPoint(pointA),
            ContactAnchor.FromWorldPoint(pointB),
            depth,
            normal);
    }

    /// <summary>
    /// Adds a rigid-frame contact, keeping the deepest four contacts and
    /// exposing them by stable anchor identity.
    /// </summary>
    public void AddContact(
        ContactAnchor anchorA,
        ContactAnchor anchorB,
        Fixed64 depth,
        Vector3d normal,
        bool depthIsClamped = false)
    {
        AddContactCore(
            anchorA,
            anchorB,
            depth,
            normal,
            hasMaterialOverride: false,
            default,
            default,
            depthIsClamped,
            featureNamespaceA: 0,
            featureNamespaceB: 0);
    }

    internal void AddContact(
        Vector3d pointA,
        Vector3d pointB,
        Fixed64 depth,
        Vector3d normal,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped = false)
    {
        AddContactCore(
            ContactAnchor.FromWorldPoint(pointA),
            ContactAnchor.FromWorldPoint(pointB),
            depth,
            normal,
            hasMaterialOverride: true,
            materialA,
            materialB,
            depthIsClamped,
            featureNamespaceA: 0,
            featureNamespaceB: 0);
    }

    internal void AddContact(
        ContactAnchor anchorA,
        ContactAnchor anchorB,
        Fixed64 depth,
        Vector3d normal,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped = false,
        int featureNamespaceA = 0,
        int featureNamespaceB = 0)
    {
        AddContactCore(
            anchorA,
            anchorB,
            depth,
            normal,
            hasMaterialOverride: true,
            materialA,
            materialB,
            depthIsClamped,
            featureNamespaceA,
            featureNamespaceB);
    }

    private void AddContactCore(
        ContactAnchor anchorA,
        ContactAnchor anchorB,
        Fixed64 depth,
        Vector3d normal,
        bool hasMaterialOverride,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped,
        int featureNamespaceA,
        int featureNamespaceB)
    {
        ulong contactId = CreateContactId(
            anchorA,
            featureNamespaceA,
            anchorB,
            featureNamespaceB);
        var contact = new ManifoldContact(
            contactId,
            anchorA,
            anchorB,
            depth,
            normal,
            hasMaterialOverride,
            materialA,
            materialB,
            depthIsClamped,
            featureNamespaceA,
            featureNamespaceB);

        for (int i = 0; i < _count; i++)
        {
            ManifoldContact existing = GetContactUnchecked(i);
            if (existing.ContactId != contactId)
                continue;

            if (IsDeeper(contact, existing))
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

    /// <summary>Returns an allocation-free enumerator over the active contacts.</summary>
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
            if (contact.Depth <= shallowest.Depth)
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

    private static ulong CreateContactId(
        ContactAnchor anchorA,
        int featureNamespaceA,
        ContactAnchor anchorB,
        int featureNamespaceB)
    {
        if (CompareLocalFeature(
                featureNamespaceB,
                anchorB,
                featureNamespaceA,
                anchorA) < 0)
        {
            (anchorA, anchorB) = (anchorB, anchorA);
            (featureNamespaceA, featureNamespaceB) =
                (featureNamespaceB, featureNamespaceA);
        }

        ulong hash = 14695981039346656037UL;
        Mix(ref hash, featureNamespaceA);
        MixLocalFeature(ref hash, anchorA);
        Mix(ref hash, featureNamespaceB);
        MixLocalFeature(ref hash, anchorB);
        return hash;
    }

    private static int CompareLocalFeature(
        int leftNamespace,
        ContactAnchor left,
        int rightNamespace,
        ContactAnchor right)
    {
        int comparison = leftNamespace.CompareTo(rightNamespace);
        return comparison != 0
            ? comparison
            : left.CompareLocalFeature(right);
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

    private static void MixLocalFeature(
        ref ulong hash,
        ContactAnchor anchor)
    {
        Mix(
            ref hash,
            unchecked((long)anchor.GetLocalFeatureHash64()));
    }

    /// <summary>Enumerates the active contacts in deterministic order.</summary>
    public struct Enumerator : IEnumerator<ManifoldContact>
    {
        private readonly ContactManifold _manifold;
        private int _index;

        internal Enumerator(ContactManifold manifold)
        {
            _manifold = manifold;
            _index = -1;
        }

        /// <summary>Gets the current contact.</summary>
        public ManifoldContact Current => _manifold[_index];

        object IEnumerator.Current => Current;

        /// <summary>Advances to the next active contact.</summary>
        public bool MoveNext()
        {
            int next = _index + 1;
            if (next >= _manifold._count)
                return false;

            _index = next;
            return true;
        }

        /// <summary>Resets the enumerator to its initial position.</summary>
        public void Reset() => _index = -1;

        /// <summary>Releases enumerator resources.</summary>
        public void Dispose() { }
    }
}
