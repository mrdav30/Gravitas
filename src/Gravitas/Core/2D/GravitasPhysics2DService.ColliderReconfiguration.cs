// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas;

/// <content>Retires exact pair lifetimes after accepted native 2D posture changes.</content>
public sealed partial class GravitasPhysics2DService
{
    internal void InvalidatePairsForColliderReconfiguration(
        LSCollider2D collider, ref SwiftList<Exception>? notificationExceptions)
    {
        // Invocation-local snapshots survive nested transactions and callbacks
        // that remove, recycle, or recreate a pair under the same collider IDs.
        var snapshot = new SwiftList<(CollisionPair2D Pair, long Lifetime)>();
        if (collider.CollisionPairs != null)
        {
            foreach (var entry in collider.CollisionPairs)
                snapshot.Add((entry.Value, entry.Value.LifetimeVersion));
        }
        if (collider.CollisionPairHolders != null)
        {
            foreach (int holderId in collider.CollisionPairHolders)
            {
                if (TryGetColliderById(holderId, out LSCollider2D? holder)
                    && holder!.TryGetCollisionPair(collider.Id, out CollisionPair2D? pair))
                    snapshot.Add((pair!, pair!.LifetimeVersion));
            }
        }

        for (int i = 0; i < snapshot.Count; i++)
        {
            var token = snapshot[i];
            CollisionPair2D pair = token.Pair;
            ulong key = CreatePairKey(pair.Id1, pair.Id2);
            if (pair.LifetimeVersion != token.Lifetime
                || !_pairs.TryGetValue(key, out CollisionPair2D current)
                || !ReferenceEquals(current, pair))
                continue;

            RemovePairReferences(pair);
            _pairs.Remove(key);
            try
            {
                pair.MarkSeparated();
            }
            catch (Exception exception)
            {
                CollisionNotificationExceptions.Capture(ref notificationExceptions, exception);
            }
            finally
            {
                if (pair.LifetimeVersion == token.Lifetime)
                    RecyclePair(pair);
            }
        }
        // Do not clear whole pair tables or hierarchy: callbacks may have
        // published a new registration or fresh pair ownership on this shell.
    }
}
