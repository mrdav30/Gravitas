//=======================================================================
// ContactManifold2D.cs
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
/// Pair-owned deterministic contact manifold with up to two samples per
/// geometric surface group and no fixed pair-wide group or contact limit.
/// </summary>
/// <remarks>
/// Inspection flattens canonical structural group order, then ascending contact
/// identity within each group. Index zero is not necessarily the deepest contact;
/// use <see cref="PrimaryContact"/> for that purpose. Overflow capacity is retained
/// across updates and resets; reaching a new high-water mark can allocate.
/// </remarks>
public sealed class ContactManifold2D : IEnumerable<ManifoldContact2D>
{
    /// <summary>Maximum number of point samples retained per geometric surface group.</summary>
    public const int MaxContactsPerGroup = 2;

    /// <summary>Maximum samples in the default group; independent groups have no pair-wide cap.</summary>
    public const int MaxContactCount = MaxContactsPerGroup;

    private ContactGroup2D _firstGroup;
    private SwiftCollections.SwiftList<ContactGroup2D>? _additionalGroups;
    private int _count;
    internal bool NeedsSurfaceRecovery;
    private long _lastUpdatedFrame = -1;

    /// <summary>Number of independent surface regions currently retained; this has no fixed pair-wide cap.</summary>
    public int GroupCount { get; private set; }

    /// <summary>Gets the first flattened point index of a group in canonical structural order.</summary>
    public int GetGroupStartIndex(int groupIndex)
    {
        ChronicleGuard.ThrowIfListIndexInvalid(groupIndex, GroupCount);
        int start = 0;
        for (int i = 0; i < groupIndex; i++)
            start += GetGroup(i).Count;
        return start;
    }

    /// <summary>Gets the number of retained samples in a group, from one through <see cref="MaxContactsPerGroup"/>.</summary>
    public int GetGroupContactCount(int groupIndex)
    {
        ChronicleGuard.ThrowIfListIndexInvalid(groupIndex, GroupCount);
        return GetGroup(groupIndex).Count;
    }

    internal ref ContactGroup2D GetGroup(int index)
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
        ChronicleGuard.ThrowIfNegative(capacity, nameof(capacity));
        if (capacity <= 1) return;
        _additionalGroups ??= new SwiftCollections.SwiftList<ContactGroup2D>(capacity - 1);
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
    public ManifoldContact2D PrimaryContact
    {
        get
        {
            ChronicleGuard.ThrowIfListIndexInvalid(0, _count);

            ManifoldContact2D best = _firstGroup[0];
            foreach (ManifoldContact2D candidate in this)
            {
                if (candidate.Depth > best.Depth
                    || candidate.Depth == best.Depth && (candidate.DepthIsClamped && !best.DepthIsClamped
                    || candidate.DepthIsClamped == best.DepthIsClamped && candidate.ContactId < best.ContactId))
                {
                    best = candidate;
                }
            }

            return best;
        }
    }

    /// <summary>Gets a contact in flattened structural-group/ascending-contact-identity order.</summary>
    public ManifoldContact2D this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            ChronicleGuard.ThrowIfListIndexInvalid(index, _count);
            for (int group = 0; ; group++)
            {
                ref ContactGroup2D current = ref GetGroup(group);
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
    public void SetContact(Vector2d pointA, Vector2d pointB, Fixed64 depth, Vector2d normal)
    {
        ClearContacts();
        AddContact(pointA, pointB, depth, normal);
    }

    /// <summary>
    /// Replaces the manifold with one rigid-frame contact.
    /// </summary>
    public void SetContact(
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
        bool depthIsClamped = false)
    {
        ClearContacts();
        AddContact(anchorA, anchorB, depth, normal, depthIsClamped);
    }

    /// <summary>
    /// Adds a contact, keeping up to two samples in the default surface group and exposing them by stable contact identity.
    /// </summary>
    public void AddContact(Vector2d pointA, Vector2d pointB, Fixed64 depth, Vector2d normal)
    {
        AddContact(
            ContactAnchor2D.FromWorldPoint(pointA),
            ContactAnchor2D.FromWorldPoint(pointB),
            depth,
            normal);
    }

    /// <summary>
    /// Adds a rigid-frame contact, keeping up to two samples in the default surface group and
    /// exposing them by stable anchor identity.
    /// </summary>
    public void AddContact(
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
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
        Vector2d pointA,
        Vector2d pointB,
        Fixed64 depth,
        Vector2d normal,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped = false)
    {
        AddContactCore(
            ContactAnchor2D.FromWorldPoint(pointA),
            ContactAnchor2D.FromWorldPoint(pointB),
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
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped = false,
        int featureNamespaceA = 0,
        int featureNamespaceB = 0,
        ContactGroupKey group = default)
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
            group);
    }

    private void AddContactCore(
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
        bool hasMaterialOverride,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped,
        int featureNamespaceA,
        int featureNamespaceB,
        ContactGroupKey group = default)
    {
        ulong contactId = CreateContactId(
            anchorA,
            featureNamespaceA,
            anchorB,
            featureNamespaceB);
        var contact = new ManifoldContact2D(
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

        // Ordinary compound parts share the established two-sample owner pool;
        // only declared structural groups retain independent constraints.
        AddContact(group.CompareTo(default) == 0 ? default
            : group.Remap(featureNamespaceA, featureNamespaceB, reverse: false), contact);
    }

    internal void AddContact(in ContactGroupKey key, in ManifoldContact2D contact)
    {
        if (GroupCount == 0)
        {
            // BeginUpdate already cleared the inline group; avoid clearing its
            // two large anchors again on every ordinary first contact.
            _firstGroup.Key = key;
            _count = _firstGroup.Add(contact);
            GroupCount = 1;
            return;
        }
        int group = 0;
        while (group < GroupCount && GetGroup(group).Key.CompareTo(key) < 0)
            group++;
        if (group == GroupCount || GetGroup(group).Key.CompareTo(key) != 0)
        {
            ReserveGroups(GroupCount + 1);
            _additionalGroups!.Add(default);
            for (int i = GroupCount; i > group; i--)
                GetGroup(i) = GetGroup(i - 1);
            GetGroup(group) = new ContactGroup2D { Key = key };
            GroupCount++;
        }
        _count += GetGroup(group).Add(contact);
    }

    internal void CopyFrom(ContactManifold2D source, long frame)
    {
        if (ReferenceEquals(this, source)) return;
        BeginUpdate(frame);
        ReserveGroups(source.GroupCount);
        for (int i = 0; i < source.GroupCount; i++)
        {
            if (i == 0) _firstGroup = source.GetGroup(i);
            else _additionalGroups!.Add(source.GetGroup(i));
        }
        GroupCount = source.GroupCount;
        _count = source._count;
        NeedsSurfaceRecovery = source.NeedsSurfaceRecovery;
    }

    internal bool HasSameContacts(ContactManifold2D source)
    {
        if (GroupCount != source.GroupCount || NeedsSurfaceRecovery != source.NeedsSurfaceRecovery) return false;
        for (int group = 0; group < GroupCount; group++)
        {
            ref ContactGroup2D previous = ref GetGroup(group);
            ref ContactGroup2D incoming = ref source.GetGroup(group);
            if (previous.Key.CompareTo(incoming.Key) != 0 || previous.Count != incoming.Count) return false;
            for (int index = 0; index < previous.Count; index++)
                if (!SameConstraint(previous[index], incoming[index])) return false;
        }
        return true;
    }

    private static bool SameConstraint(in ManifoldContact2D previous, in ManifoldContact2D incoming) =>
        previous.ContactId == incoming.ContactId && previous.FeatureNamespaceA == incoming.FeatureNamespaceA
        && previous.FeatureNamespaceB == incoming.FeatureNamespaceB && previous.Normal == incoming.Normal
        && previous.Depth == incoming.Depth && previous.DepthIsClamped == incoming.DepthIsClamped
        && previous.HasMaterialOverride == incoming.HasMaterialOverride
        && previous.MaterialA == incoming.MaterialA && previous.MaterialB == incoming.MaterialB
        && SamePoint(previous.AnchorA, incoming.AnchorA) && SamePoint(previous.AnchorB, incoming.AnchorB);

    private static bool SamePoint(in ContactAnchor2D previous, in ContactAnchor2D incoming)
    {
        ExactLever3D delta = previous.GetXZLeverFrom(incoming);
        return delta.XNumerator.IsZero && delta.ZNumerator.IsZero;
    }

    internal bool ReferencesSuppliers(SwiftCollections.SwiftHashSet<int> suppliers, bool first, int fallbackSource)
    {
        for (int group = 0; group < GroupCount; group++)
        {
            ContactGroupKey key = GetGroup(group).Key;
            int source = first ? key.SourceIdA : key.SourceIdB;
            if (suppliers.Contains(source == 0 ? fallbackSource : source)) return true;
        }
        return false;
    }

    private void ClearContacts()
    {
        _count = GroupCount = 0;
        NeedsSurfaceRecovery = false;
        _firstGroup = default;
        _additionalGroups?.FastClear();
    }

    /// <summary>Returns an allocation-free enumerator over the active contacts.</summary>
    public Enumerator GetEnumerator() => new(this);

    IEnumerator<ManifoldContact2D> IEnumerable<ManifoldContact2D>.GetEnumerator() => GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    private static ulong CreateContactId(
        ContactAnchor2D anchorA,
        int featureNamespaceA,
        ContactAnchor2D anchorB,
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
        ContactAnchor2D left,
        int rightNamespace,
        ContactAnchor2D right)
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
        ContactAnchor2D anchor)
    {
        Mix(
            ref hash,
            unchecked((long)anchor.GetLocalFeatureHash64()));
    }

    /// <summary>Enumerates the active contacts in deterministic order.</summary>
    public struct Enumerator : IEnumerator<ManifoldContact2D>
    {
        private readonly ContactManifold2D _manifold;
        private int _group, _point;
        private int _index;

        internal Enumerator(ContactManifold2D manifold)
        {
            _manifold = manifold;
            _group = _point = 0;
            _index = -1;
        }

        /// <summary>Gets the current contact.</summary>
        public ManifoldContact2D Current
        {
            get
            {
                ChronicleGuard.ThrowIfListIndexInvalid(_index, _manifold.Count);
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
