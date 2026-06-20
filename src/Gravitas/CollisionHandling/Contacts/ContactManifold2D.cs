using FixedMathSharp;
using SwiftCollections;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Fixed-capacity deterministic pure 2D contact manifold owned by one collision pair.
/// </summary>
public sealed class ContactManifold2D : IEnumerable<ManifoldContact2D>
{
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

            return _contact1.Depth > _contact0.Depth
                || _contact1.Depth == _contact0.Depth && _contact1.ContactId < _contact0.ContactId
                ? _contact1
                : _contact0;
        }
    }

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
    /// Adds a contact, keeping the deepest two contacts and exposing them by stable contact identity.
    /// </summary>
    public void AddContact(Vector2d pointA, Vector2d pointB, Fixed64 depth, Vector2d normal)
    {
        ulong contactId = CreateContactId(pointA, pointB);
        var contact = new ManifoldContact2D(contactId, pointA, pointB, depth, normal);

        for (int i = 0; i < _count; i++)
        {
            ManifoldContact2D existing = GetContactUnchecked(i);
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

    public Enumerator GetEnumerator() => new(this);

    IEnumerator<ManifoldContact2D> IEnumerable<ManifoldContact2D>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    private int FindShallowestReplacementIndex(ManifoldContact2D candidate)
    {
        int replaceIndex = _contact1.Depth < _contact0.Depth
            || _contact1.Depth == _contact0.Depth && _contact1.ContactId > _contact0.ContactId
            ? 1
            : 0;

        ManifoldContact2D shallowest = GetContactUnchecked(replaceIndex);
        if (candidate.Depth > shallowest.Depth)
            return replaceIndex;

        if (candidate.Depth == shallowest.Depth && candidate.ContactId < shallowest.ContactId)
            return replaceIndex;

        return -1;
    }

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

    private static ulong CreateContactId(Vector2d pointA, Vector2d pointB)
    {
        if (CompareVector(pointB, pointA) < 0)
            (pointA, pointB) = (pointB, pointA);

        ulong hash = 14695981039346656037UL;
        Mix(ref hash, pointA.X.m_rawValue);
        Mix(ref hash, pointA.Y.m_rawValue);
        Mix(ref hash, pointB.X.m_rawValue);
        Mix(ref hash, pointB.Y.m_rawValue);
        return hash;
    }

    private static int CompareVector(Vector2d left, Vector2d right)
    {
        int compare = left.X.m_rawValue.CompareTo(right.X.m_rawValue);
        if (compare != 0)
            return compare;

        return left.Y.m_rawValue.CompareTo(right.Y.m_rawValue);
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

    public struct Enumerator : IEnumerator<ManifoldContact2D>
    {
        private readonly ContactManifold2D _manifold;
        private int _index;

        internal Enumerator(ContactManifold2D manifold)
        {
            _manifold = manifold;
            _index = -1;
        }

        public ManifoldContact2D Current => _manifold[_index];

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
