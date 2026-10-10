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
/// Pair-owned deterministic contact manifold with up to four samples per
/// geometric surface group and no fixed pair-wide group or contact limit.
/// </summary>
/// <remarks>
/// Inspection flattens canonical structural group order, then ascending contact
/// identity within each group. Index zero is not necessarily the deepest contact;
/// use <see cref="PrimaryContact"/> for that purpose. Overflow capacity is retained
/// across updates and resets; reaching a new high-water mark can allocate.
/// </remarks>
public sealed class ContactManifold : IEnumerable<ManifoldContact>
{
    /// <summary>Maximum number of point samples retained per geometric surface group.</summary>
    public const int MaxContactsPerGroup = 4;

    private ContactGroup _firstGroup;
    private SwiftCollections.SwiftList<ContactGroup>? _additionalGroups;
    private int _count;
    private long _lastUpdatedFrame = -1;

    /// <summary>Number of independent surface regions currently retained; this has no fixed pair-wide cap.</summary>
    public int GroupCount { get; private set; }

    /// <summary>Gets the first flattened point index of a group in canonical structural order.</summary>
    public int GetGroupStartIndex(int groupIndex)
    {
        SwiftThrowHelper.ThrowIfListIndexInvalid(groupIndex, GroupCount);
        int start = 0;
        for (int i = 0; i < groupIndex; i++)
            start += GetGroup(i).Count;
        return start;
    }

    /// <summary>Gets the number of retained samples in a group, from one through <see cref="MaxContactsPerGroup"/>.</summary>
    public int GetGroupContactCount(int groupIndex)
    {
        SwiftThrowHelper.ThrowIfListIndexInvalid(groupIndex, GroupCount);
        return GetGroup(groupIndex).Count;
    }

    internal ref ContactGroup GetGroup(int index)
    {
        if (index == 0) return ref _firstGroup;
        return ref _additionalGroups!.InnerArray[index - 1];
    }

    internal int FindGroup(in ContactGroupKey key)
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

    internal void ReserveGroups(int capacity)
    {
        SwiftThrowHelper.ThrowIfNegative(capacity, nameof(capacity));
        if (capacity <= 1) return;
        _additionalGroups ??= new SwiftCollections.SwiftList<ContactGroup>(capacity - 1);
        _additionalGroups.EnsureCapacity(capacity - 1);
    }

    /// <summary>
    /// Total number of active samples across all groups, without a fixed pair-wide cap.
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
    public long LastUpdatedFrame
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

            ManifoldContact best = _firstGroup[0];
            foreach (ManifoldContact candidate in this)
            {
                if (candidate.Depth > best.Depth
                    || candidate.Depth == best.Depth && candidate.ContactId < best.ContactId)
                {
                    best = candidate;
                }
            }

            return best;
        }
    }

    /// <summary>Gets a contact in flattened structural-group/ascending-contact-identity order.</summary>
    public ManifoldContact this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            SwiftThrowHelper.ThrowIfListIndexInvalid(index, _count);
            for (int group = 0; ; group++)
            {
                ref ContactGroup current = ref GetGroup(group);
                if (index < current.Count) return current[index];
                index -= current.Count;
            }
        }
    }

    /// <summary>
    /// Clears contacts and records the frame for a new narrow-phase pass.
    /// </summary>
    public void BeginUpdate(long frame)
    {
        ClearContacts();
        _lastUpdatedFrame = frame;
    }

    /// <summary>
    /// Clears all contact data.
    /// </summary>
    public void Reset()
    {
        ClearContacts();
        _lastUpdatedFrame = -1;
    }

    /// <summary>
    /// Replaces the manifold with one contact.
    /// </summary>
    public void SetContact(Vector3d pointA, Vector3d pointB, Fixed64 depth, Vector3d normal)
    {
        ClearContacts();
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
        ClearContacts();
        AddContact(anchorA, anchorB, depth, normal, depthIsClamped);
    }

    /// <summary>
    /// Adds a contact, keeping up to four samples in the default surface group and exposing them by stable contact identity.
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
    /// Adds a rigid-frame contact, keeping up to four samples in the default surface group and
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
        int featureNamespaceB = 0,
        ContactGroupKey group = default,
        int sampleIdentity = 0,
        ulong? contactIdentity = null)
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
            featureNamespaceB,
            group, sampleIdentity, contactIdentity);
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
        int featureNamespaceB,
        ContactGroupKey group = default,
        int sampleIdentity = 0,
        ulong? contactIdentity = null)
    {
        ulong contactId = contactIdentity ?? CreateContactId(
            anchorA,
            featureNamespaceA,
            anchorB,
            featureNamespaceB);
        // A continuous normal family can share paired anchors at zero depth.
        // Keep its reduced directions as distinct rows inside one feature group.
        if (sampleIdentity != 0) Mix(ref contactId, sampleIdentity);
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

        AddContact(group.Remap(featureNamespaceA, featureNamespaceB, reverse: false), contact);
    }

    internal void AddContact(in ContactGroupKey key, in ManifoldContact contact)
    {
        int group = 0;
        while (group < GroupCount && GetGroup(group).Key.CompareTo(key) < 0)
            group++;
        if (group == GroupCount || GetGroup(group).Key.CompareTo(key) != 0)
        {
            ReserveGroups(GroupCount + 1);
            if (GroupCount > 0)
            {
                _additionalGroups!.Add(default);
                for (int i = GroupCount; i > group; i--)
                    GetGroup(i) = GetGroup(i - 1);
            }
            GetGroup(group) = new ContactGroup { Key = key };
            GroupCount++;
        }
        _count += GetGroup(group).Add(contact);
    }

    private void ClearContacts()
    {
        _count = GroupCount = 0;
        _firstGroup = default;
        _additionalGroups?.FastClear();
    }

    /// <summary>Returns an allocation-free enumerator over the active contacts.</summary>
    public Enumerator GetEnumerator() => new(this);

    IEnumerator<ManifoldContact> IEnumerable<ManifoldContact>.GetEnumerator() => GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    internal static ulong RemapContactIdentity(ulong contactId, int namespaceA, int namespaceB)
    {
        // Group provenance carries the ordered owners. Identity remains
        // invariant when compound dispatch reverses the paired anchors.
        Mix(ref contactId, System.Math.Min(namespaceA, namespaceB));
        Mix(ref contactId, System.Math.Max(namespaceA, namespaceB));
        return contactId;
    }

    internal static ulong CreateContactId(
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
        private int _group, _point;
        private int _index;

        internal Enumerator(ContactManifold manifold)
        {
            _manifold = manifold;
            _group = _point = 0;
            _index = -1;
        }

        /// <summary>Gets the current contact.</summary>
        public ManifoldContact Current
        {
            get
            {
                SwiftThrowHelper.ThrowIfListIndexInvalid(_index, _manifold.Count);
                return _manifold.GetGroup(_group)[_point];
            }
        }

        object IEnumerator.Current => Current;

        /// <summary>Advances to the next active contact.</summary>
        public bool MoveNext()
        {
            int next = _index + 1;
            if (next >= _manifold._count)
                return false;

            if (_index >= 0 && ++_point >= _manifold.GetGroup(_group).Count)
            {
                _group++;
                _point = 0;
            }
            _index = next;
            return true;
        }

        /// <summary>Resets the enumerator to its initial position.</summary>
        public void Reset()
        {
            _group = _point = 0;
            _index = -1;
        }

        /// <summary>Releases enumerator resources.</summary>
        public void Dispose() { }
    }
}
