//=======================================================================
// GravitasPhysicsService.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

public sealed partial class GravitasPhysicsService
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        SwiftList<LSCollider> replayColliders = PrepareReplayColliders();

        writer.WriteSection("physics.3d", 2);
        writer.WriteBool(SimulatePhysics);
        writer.WriteInt32(BodyCount);
        writer.WriteInt32(ColliderCount);
        writer.WriteInt32(_dynamicBodies.PeakCount);
        writer.WriteInt32(replayColliders.Count);

        for (int i = 0; i < replayColliders.Count; i++)
        {
            LSCollider collider = replayColliders[i];
            collider.ContributeReplayHash(ref writer, mode);
            if (collider.Body == null)
                writer.WriteBool(false);
            else
            {
                writer.WriteBool(true);
                collider.Body.ContributeReplayHash(ref writer, mode);
            }
        }

        writer.WriteSection("physics.3d.pairs", 2);
        for (int i = 0; i < replayColliders.Count; i++)
        {
            LSCollider collider = replayColliders[i];

            for (int j = i + 1; j < replayColliders.Count; j++)
            {
                LSCollider other = replayColliders[j];
                if ((!collider.TryGetCollisionPair(other.Id, out CollisionPair? pair)
                        && !other.TryGetCollisionPair(collider.Id, out pair))
                    || pair == null)
                {
                    continue;
                }

                pair.ContributeReplayHash(ref writer, mode);
            }
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("physics.3d.caches", 3);
        writer.WriteInt32(PeakColliderCount);
        writer.WriteInt32(_cachedCollisionPairs.Count);
        writer.WriteInt32(_activeCollisionPairs.Count);
        writer.WriteInt32(_continuousCollisionPreparedToken);
        writer.WriteInt32(_continuousCollisionCandidates.Count);
        writer.WriteInt32(_processedContinuousCollisionBodyIds.Count);
        writer.WriteInt32(_queuedContinuousCollisionHandoffIds.Count);
        writer.WriteInt32(_continuousCollisionHandoffQueue.Count);
        for (int i = 0; i < _continuousCollisionHandoffQueue.Count; i++)
            writer.WriteInt32(_continuousCollisionHandoffQueue[i]);
        writer.WriteInt32(LastContinuousCollisionIslandCount);
        writer.WriteInt32(LastContinuousCollisionIslandIterationCount);
        writer.WriteBool(LastContinuousCollisionIslandLimitReached);
    }
}
