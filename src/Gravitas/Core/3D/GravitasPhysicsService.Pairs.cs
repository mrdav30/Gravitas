//=======================================================================
// GravitasPhysicsService.Pairs.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas;

internal readonly struct CollisionPairLifetimeToken
{
    internal CollisionPairLifetimeToken(CollisionPair pair)
    {
        Pair = pair;
        LifetimeVersion = pair.LifetimeVersion;
        ColliderA = new ColliderLifetimeToken(pair.ColliderA);
        ColliderB = new ColliderLifetimeToken(pair.ColliderB);
    }

    internal CollisionPair Pair { get; }

    internal long LifetimeVersion { get; }

    private ColliderLifetimeToken ColliderA { get; }

    private ColliderLifetimeToken ColliderB { get; }

    internal bool IsCurrentLifetime => Pair.LifetimeVersion == LifetimeVersion
        && ColliderA.IsCurrentLifetime
        && ColliderB.IsCurrentLifetime;
}

public sealed partial class GravitasPhysicsService
{
    internal CollisionPair? GetCollisionPair(int id1, int id2)
    {
        if (!TryGetColliderById(id1, out LSCollider? collider1)
            || !TryGetColliderById(id2, out LSCollider? collider2))
        {
            return null;
        }

        if (collider1!.Id > collider2!.Id)
            (collider2, collider1) = (collider1, collider2);

        if (!RequireCollisionPair(collider1, collider2))
            return null;

        if (!collider1.TryGetCollisionPair(collider2.Id, out CollisionPair? pair)
            && !collider2.TryGetCollisionPair(collider1.Id, out pair))
        {
            pair = CreatePair(collider1, collider2);
            pair.ColliderA.TryAddCollisionPair(pair.ColliderB.Id, pair);
            pair.ColliderB.TryAddCollisionPairHolder(pair.ColliderA.Id);
        }

        return pair;
    }

    internal bool RequireCollisionPair(LSCollider collider1, LSCollider collider2)
    {
        return collider1.IsActive && collider2.IsActive
            && !collider1.IsDeactivationInProgress && !collider2.IsDeactivationInProgress
            && collider1.Shape != ColliderType.None && collider2.Shape != ColliderType.None
            && (collider1.Body != null || collider2.Body != null)
            && !IsLayerCollisionDisabled(collider1.Layer, collider2.Layer)
            && ColliderCollisionFilter.AllowsPhysicalPair(collider1, collider2)
            && !_context.Constraints3D.ShouldExcludeLinkedCollision(collider1, collider2)
            && !collider1.IsSibling(collider2);
    }

    internal bool IsLayerCollisionDisabled(PhysicsLayer layer1, PhysicsLayer layer2)
    {
        bool[,] matrix = _context.Settings.CollisionMatrix;
        int layerIndex1 = layer1.Index;
        int layerIndex2 = layer2.Index;
        if (layerIndex1 >= matrix.GetLength(0) || layerIndex2 >= matrix.GetLength(1))
            return false;

        return !matrix[layerIndex1, layerIndex2];
    }

    internal void PoolForDeactivation(CollisionPair pair)
    {
        SwiftThrowHelper.ThrowIfNull(pair, nameof(pair));
        lock (_activeCollisionPairs)
            _activeCollisionPairs.Enqueue(new CollisionPairLifetimeToken(pair));
    }

    internal void FullDeactivateCollisionPair(CollisionPair pair)
    {
        RemovePairReferences(pair);
        DeactivateAndPoolPair(pair);
    }

    internal void DeactivateAndPoolPair(CollisionPair pair)
    {
        if (!pair.Active)
            return;

        pair.Deactivate();
        if (_context.Settings.PoolingEnabled && !pair.IsNotificationInProgress)
            _cachedCollisionPairs.Push(pair);
    }

    internal void RemovePairReferences(CollisionPair pair)
    {
        pair.ColliderA.TryRemoveCollisionPair(pair.Id2);
        pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
    }

    internal void RemovePairsForCollider(LSCollider collider)
    {
        int deactivationStart = _pairsPendingDeactivation.Count;
        SwiftDictionary<int, CollisionPair>? collisionPairs = collider.CollisionPairs;
        if (collisionPairs != null)
        {
            foreach (var pairEntry in collisionPairs)
                _pairsPendingDeactivation.Add(new CollisionPairLifetimeToken(pairEntry.Value));
        }

        SwiftHashSet<int>? collisionPairHolders = collider.CollisionPairHolders;
        if (collisionPairHolders != null)
        {
            foreach (int holderId in collisionPairHolders)
            {
                if (!TryGetColliderById(holderId, out LSCollider? holder))
                    continue;

                if (holder!.TryGetCollisionPair(collider.Id, out CollisionPair? pair))
                    _pairsPendingDeactivation.Add(new CollisionPairLifetimeToken(pair!));
            }
        }

        int deactivationEnd = _pairsPendingDeactivation.Count;
        try
        {
            for (int i = deactivationStart; i < deactivationEnd; i++)
            {
                CollisionPairLifetimeToken token = _pairsPendingDeactivation[i];
                if (token.IsCurrentLifetime)
                    FullDeactivateCollisionPair(token.Pair);
            }

            collider.ClearCollisionPairState();
            collider.ClearRuntimeRelationships();
        }
        finally
        {
            if (deactivationStart == 0)
                _pairsPendingDeactivation.FastClear();
        }
    }

    private CollisionPair CreatePair(LSCollider collider1, LSCollider collider2)
    {
        if (!_context.Settings.PoolingEnabled || _cachedCollisionPairs.Count <= 0)
            return new CollisionPair(collider1, collider2);

        CollisionPair pair = _cachedCollisionPairs.Pop();
        pair.Initialize(collider1, collider2);
        return pair;
    }

    private void ProcessActiveCollisionPairs()
    {
        int snapshotStart = _activeCollisionPairSnapshot.Count;
        int collisionCounter = _activeCollisionPairs.Count;
        for (int i = 0; i < collisionCounter; i++)
            _activeCollisionPairSnapshot.Add(_activeCollisionPairs.Dequeue());

        int snapshotEnd = _activeCollisionPairSnapshot.Count;
        int processingIndex = snapshotStart;
        try
        {
            for (; processingIndex < snapshotEnd; processingIndex++)
            {
                CollisionPairLifetimeToken token = _activeCollisionPairSnapshot[processingIndex];
                CollisionPair instancePair = token.Pair;
                if (!token.IsCurrentLifetime || !instancePair.Active)
                    continue;

                if (!RequireCollisionPair(instancePair.ColliderA, instancePair.ColliderB))
                {
                    FullDeactivateCollisionPair(instancePair);
                    continue;
                }

                bool preservedSleepingContact = instancePair.TryPreserveSleepingRestingContact();
                int passedFrames = _context.FrameCount - instancePair.LastCollidedFrame;
                if (!preservedSleepingContact && passedFrames >= InactiveFrameThreshold)
                    FullDeactivateCollisionPair(instancePair);
                else
                {
                    if (instancePair.CullCounter <= 0)
                        instancePair.NotifyCollidersOfContact();
                    if (token.IsCurrentLifetime && instancePair.Active)
                        _activeCollisionPairs.Enqueue(token);
                }
            }
        }
        finally
        {
            for (; processingIndex < snapshotEnd; processingIndex++)
            {
                CollisionPairLifetimeToken token = _activeCollisionPairSnapshot[processingIndex];
                if (token.IsCurrentLifetime && token.Pair.Active)
                    _activeCollisionPairs.Enqueue(token);
            }

            if (snapshotStart == 0)
                _activeCollisionPairSnapshot.FastClear();
        }
    }

}
