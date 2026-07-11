//=======================================================================
// GravitasPhysics2DService.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        SwiftList<LSCollider2D> replayColliders = PrepareReplayColliders();

        writer.WriteSection("physics.2d", 2);
        writer.WriteBool(SimulatePhysics);
        writer.WriteInt32(BodyCount);
        writer.WriteInt32(ColliderCount);
        writer.WriteInt32(_dynamicBodies.PeakCount);
        writer.WriteInt32(replayColliders.Count);

        for (int i = 0; i < replayColliders.Count; i++)
        {
            LSCollider2D collider = replayColliders[i];
            collider.ContributeReplayHash(ref writer, mode);
            if (collider.Body == null)
                writer.WriteBool(false);
            else
            {
                writer.WriteBool(true);
                collider.Body.ContributeReplayHash(ref writer, mode);
            }
        }

        writer.WriteSection("physics.2d.pairs", 2);
        for (int i = 0; i < replayColliders.Count; i++)
        {
            LSCollider2D collider = replayColliders[i];

            for (int j = i + 1; j < replayColliders.Count; j++)
            {
                LSCollider2D other = replayColliders[j];
                if (!TryGetReplayHashPair(collider, other, out CollisionPair2D? pair))
                {
                    continue;
                }

                pair!.ContributeReplayHash(ref writer, mode);
            }
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("physics.2d.caches", 2);
        writer.WriteInt32(_colliders.PeakCount);
        writer.WriteInt32(_processedPairKeys.Count);
        writer.WriteInt32(_pairs.Count);
        writer.WriteInt32(_pairsToRemove.Count);
        writer.WriteInt32(_cachedPairs.Count);
        writer.WriteInt32(_discreteResponsePairs.Count);
        writer.WriteInt32(_continuousCollisionPreparedToken);
        writer.WriteBool(_continuousCollisionPreparedMixedIndex);
        writer.WriteInt32(_planarContinuousCollisionCandidates.Count);
        writer.WriteInt32(_mixedContinuousCollisionCandidates.Count);
        writer.WriteInt32(_processedContinuousCollisionBodies.Count);
        writer.WriteInt32(_queuedContinuousCollisionHandoffBodies.Count);
        writer.WriteInt32(_continuousCollisionHandoffQueue.Count);
        for (int i = 0; i < _continuousCollisionHandoffQueue.Count; i++)
            writer.WriteInt32(_continuousCollisionHandoffQueue[i].DynamicId);
        writer.WriteInt32(LastBroadPhaseCandidateCount);
        writer.WriteInt32(LastContinuousCollisionIslandCount);
        writer.WriteInt32(LastContinuousCollisionIslandIterationCount);
        writer.WriteBool(LastContinuousCollisionIslandLimitReached);
    }

    private static bool TryGetReplayHashPair(
        LSCollider2D first,
        LSCollider2D second,
        out CollisionPair2D? pair) =>
        first.TryGetCollisionPair(second.Id, out pair)
        || second.TryGetCollisionPair(first.Id, out pair);
}
