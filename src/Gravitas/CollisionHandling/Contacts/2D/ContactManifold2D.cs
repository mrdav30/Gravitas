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
/// Fixed-capacity deterministic pure 2D contact manifold owned by one collision pair.
/// </summary>
public sealed class ContactManifold2D : IEnumerable<ManifoldContact2D>
{
    /// <summary>Maximum number of contacts retained by a pure 2D manifold.</summary>
    public const int MaxContactCount = 2;

    private ManifoldContact2D _contact0;
    private ManifoldContact2D _contact1;
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
    public ManifoldContact2D PrimaryContact
    {
        get
        {
            SwiftThrowHelper.ThrowIfListIndexInvalid(0, _count);

            if (_count == 1)
                return _contact0;

            return IsDeeper(_contact1, _contact0)
                || HasEqualDepth(_contact1, _contact0) && _contact1.ContactId < _contact0.ContactId
                ? _contact1
                : _contact0;
        }
    }

    /// <summary>Gets the contact at the specified deterministic order index.</summary>
    public ManifoldContact2D this[int index]
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
    }

    /// <summary>
    /// Replaces the manifold with one contact.
    /// </summary>
    public void SetContact(Vector2d pointA, Vector2d pointB, Fixed64 depth, Vector2d normal)
    {
        _count = 0;
        AddContact(pointA, pointB, depth, normal);
    }

    /// <summary>
    /// Replaces the manifold with one canonical rigid-frame planar contact.
    /// </summary>
    public void SetContact(
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
        bool depthIsClamped = false)
    {
        _count = 0;
        AddContact(anchorA, anchorB, depth, normal, depthIsClamped);
    }

    /// <summary>
    /// Adds a contact, keeping the deepest two contacts and exposing them by stable contact identity.
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
    /// Adds a canonical rigid-frame contact, keeping the deepest two contacts and
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
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
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

        for (int i = 0; i < _count; i++)
        {
            ManifoldContact2D existing = GetContactUnchecked(i);
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

    IEnumerator<ManifoldContact2D> IEnumerable<ManifoldContact2D>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

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
        if (_count < 2 || _contact0.ContactId <= _contact1.ContactId)
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
        Mix(
            ref hash,
            unchecked((long)anchorA.GetLocalFeatureHash64()));
        Mix(ref hash, featureNamespaceB);
        Mix(
            ref hash,
            unchecked((long)anchorB.GetLocalFeatureHash64()));
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

    /// <summary>Enumerates the active contacts in deterministic order.</summary>
    public struct Enumerator : IEnumerator<ManifoldContact2D>
    {
        private readonly ContactManifold2D _manifold;
        private int _index;

        internal Enumerator(ContactManifold2D manifold)
        {
            _manifold = manifold;
            _index = -1;
        }

        /// <summary>Gets the current contact.</summary>
        public ManifoldContact2D Current => _manifold[_index];

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
