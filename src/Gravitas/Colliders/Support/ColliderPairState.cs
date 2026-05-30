using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal struct ColliderPairState<TPair>
    where TPair : class
{
    private SwiftDictionary<int, TPair>? _collisionPairs;
    private SwiftHashSet<int>? _collisionPairHolders;

    public int CollisionPairCount => _collisionPairs?.Count ?? 0;

    public int CollisionPairHolderCount => _collisionPairHolders?.Count ?? 0;

    public SwiftDictionary<int, TPair>? CollisionPairs => _collisionPairs;

    public SwiftHashSet<int>? CollisionPairHolders => _collisionPairHolders;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetCollisionPair(int otherId, out TPair? collisionPair)
    {
        if (_collisionPairs == null)
        {
            collisionPair = null;
            return false;
        }

        return _collisionPairs.TryGetValue(otherId, out collisionPair);
    }

    public bool TryAddCollisionPair(int otherId, TPair collisionPair)
    {
        _collisionPairs ??= new();
        return _collisionPairs.Add(otherId, collisionPair);
    }

    public bool TryRemoveCollisionPair(int otherId, out TPair? collisionPair)
    {
        if (_collisionPairs == null)
        {
            collisionPair = null;
            return false;
        }

        if (!_collisionPairs.TryGetValue(otherId, out collisionPair))
            return false;

        _collisionPairs.Remove(otherId);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryAddCollisionPairHolder(int otherId)
    {
        _collisionPairHolders ??= new();
        return _collisionPairHolders.Add(otherId);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryRemoveCollisionPairHolder(int otherId) =>
        _collisionPairHolders?.Remove(otherId) == true;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearCollisionPairs() => _collisionPairs?.Clear();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearCollisionPairHolders() => _collisionPairHolders?.Clear();
}
