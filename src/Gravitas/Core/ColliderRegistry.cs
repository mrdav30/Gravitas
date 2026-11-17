//=======================================================================
// ColliderRegistry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas;

/// <summary>
/// Tracks live colliders by ID and prepares them in registration order for replay.
/// </summary>
internal sealed partial class ColliderRegistry<TCollider>
    where TCollider : class, IPhysicsColliderRegistryItem
{
    private readonly SwiftSlotMap<TCollider> _byId;
    private readonly SwiftList<TCollider> _liveColliders;
    private readonly SwiftList<TCollider> _replayColliders;
    private int _firstRegistrationId = -1;
    private int _lastRegistrationId = -1;
    private int _nextReplayOrder;

    public ColliderRegistry(int capacity = SwiftSlotMap<TCollider>.DefaultCapacity)
    {
        _byId = new SwiftSlotMap<TCollider>(capacity);
        _liveColliders = new SwiftList<TCollider>(capacity);
        _replayColliders = new SwiftList<TCollider>(capacity);
    }

    public int Count => _liveColliders.Count;

    public int PeakCount => _byId.PeakCount;

    public int FirstRegistrationId => _firstRegistrationId;

    public TCollider this[int serviceIndex]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _liveColliders[serviceIndex];
    }

    public int Register(TCollider collider)
    {
        ChronicleGuard.ThrowIfNull(collider, nameof(collider));

        int id = _byId.Add(collider);
        int serviceIndex = _liveColliders.Count;
        collider.SetRegistryState(id, serviceIndex, _nextReplayOrder++);
        _liveColliders.Add(collider);

        collider.PreviousRegistrationId = _lastRegistrationId;
        collider.NextRegistrationId = -1;
        if (_lastRegistrationId < 0)
            _firstRegistrationId = id;
        else
            _byId[_lastRegistrationId].NextRegistrationId = id;
        _lastRegistrationId = id;

        return id;
    }

    public bool Remove(TCollider collider)
    {
        ChronicleGuard.ThrowIfNull(collider, nameof(collider));

        int id = collider.Id;
        if (!TryGetById(id, out TCollider? registered) || !ReferenceEquals(registered, collider))
            return false;

        int previous = collider.PreviousRegistrationId;
        int next = collider.NextRegistrationId;
        if (previous < 0)
            _firstRegistrationId = next;
        else
            _byId[previous].NextRegistrationId = next;
        if (next < 0)
            _lastRegistrationId = previous;
        else
            _byId[next].PreviousRegistrationId = previous;

        RemoveLiveCollider(collider);
        _byId.RemoveAt(id);
        collider.ClearRegistryState();
        return true;
    }

    public void Clear()
    {
        for (int i = 0; i < _liveColliders.Count; i++)
            _liveColliders[i].ClearRegistryState();

        _liveColliders.Clear();
        // Replay gathers can leave a larger previous snapshot beyond Count.
        _replayColliders.Clear(clearRetainedReferences: true);
        _byId.Clear();
        _firstRegistrationId = -1;
        _lastRegistrationId = -1;
        _nextReplayOrder = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetById(int id, out TCollider? collider)
    {
        if (id < 0 || !_byId.TryGetValue(id, out TCollider? value))
        {
            collider = null;
            return false;
        }

        collider = value;
        return true;
    }

    public bool TryGetByServiceIndex(int serviceIndex, out TCollider? collider)
    {
        if (serviceIndex < 0 || serviceIndex >= _liveColliders.Count)
        {
            collider = null;
            return false;
        }

        collider = _liveColliders[serviceIndex];
        return true;
    }

    public SwiftList<TCollider> PrepareReplayColliders()
    {
        _replayColliders.FastClear();
        _replayColliders.EnsureCapacity(_liveColliders.Count);
        for (int id = _firstRegistrationId; id >= 0;)
        {
            TCollider collider = GetAtRegistration(id, out id);
            collider.SetRegistryReplayOrdinal(_replayColliders.Count);
            _replayColliders.Add(collider);
        }

        return _replayColliders;
    }

    public TCollider GetAtRegistration(int id, out int nextId)
    {
        TCollider collider = _byId[id];
        nextId = collider.NextRegistrationId;
        return collider;
    }

    private void RemoveLiveCollider(TCollider collider)
    {
        int index = collider.ServiceIndex;
        int lastIndex = _liveColliders.Count - 1;
        if (index != lastIndex)
        {
            TCollider moved = _liveColliders[lastIndex];
            _liveColliders[index] = moved;
            moved.SetRegistryServiceIndex(index);
        }

        _liveColliders.RemoveAt(lastIndex);
    }


}
