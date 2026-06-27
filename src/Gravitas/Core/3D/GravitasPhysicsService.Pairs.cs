//=======================================================================
// GravitasPhysicsService.Pairs.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;

namespace Gravitas;

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
            && collider1.Shape != ColliderType.None && collider2.Shape != ColliderType.None
            && (collider1.Body != null || collider2.Body != null)
            && !IsLayerCollisionDisabled(collider1.Layer, collider2.Layer)
            && ColliderCollisionFilter.AllowsPhysicalPair(collider1, collider2)
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
            _activeCollisionPairs.Enqueue(pair);
    }

    internal void FullDeactivateCollisionPair(CollisionPair pair)
    {
        if (!pair.Active)
            return;

        TryRemovePairReferences(pair);
        DeactivateAndPoolPair(pair);
    }

    internal void DeactivateAndPoolPair(CollisionPair pair)
    {
        if (!pair.Active)
            return;

        pair.Deactivate();
        if (_context.Settings.PoolingEnabled)
            _cachedCollisionPairs.Push(pair);
    }

    internal bool TryRemovePairReferences(CollisionPair pair)
    {
        return pair.ColliderA.TryRemoveCollisionPair(pair.Id2)
            && pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
    }

    private CollisionPair CreatePair(LSCollider collider1, LSCollider collider2)
    {
        if (_cachedCollisionPairs.Count <= 0)
            return new CollisionPair(collider1, collider2);

        CollisionPair pair = _cachedCollisionPairs.Pop();
        pair.Initialize(collider1, collider2);
        return pair;
    }

    private void ProcessActiveCollisionPairs()
    {
        int collisionCounter = _activeCollisionPairs.Count;
        while (collisionCounter > 0)
        {
            CollisionPair instancePair = _activeCollisionPairs.Dequeue();
            if (instancePair == null || !instancePair.Active)
            {
                collisionCounter--;
                continue;
            }

            if (!RequireCollisionPair(instancePair.ColliderA, instancePair.ColliderB))
            {
                FullDeactivateCollisionPair(instancePair);
                collisionCounter--;
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
                _activeCollisionPairs.Enqueue(instancePair);
            }

            collisionCounter--;
        }
    }

}
