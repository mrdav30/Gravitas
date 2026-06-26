//=======================================================================
// GravitasPhysics2DService.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("physics.2d", 1);
        writer.WriteBool(SimulatePhysics);
        writer.WriteInt32(BodyCount);
        writer.WriteInt32(ColliderCount);
        writer.WriteInt32(_dynamicBodies.PeakCount);
        writer.WriteInt32(_nextColliderId);

        for (int colliderId = 1; colliderId < _nextColliderId; colliderId++)
        {
            bool hasCollider = TryGetColliderById(colliderId, out LSCollider2D? collider);
            writer.WriteBool(hasCollider);
            if (!hasCollider)
                continue;

            collider!.ContributeReplayHash(ref writer, mode);
            if (collider.Body == null)
                writer.WriteBool(false);
            else
            {
                writer.WriteBool(true);
                collider.Body.ContributeReplayHash(ref writer, mode);
            }
        }

        writer.WriteSection("physics.2d.pairs", 1);
        for (int colliderId = 1; colliderId < _nextColliderId; colliderId++)
        {
            if (!TryGetColliderById(colliderId, out LSCollider2D? collider))
                continue;

            for (int otherId = colliderId + 1; otherId < _nextColliderId; otherId++)
            {
                if (collider!.TryGetCollisionPair(otherId, out CollisionPair2D? pair) && pair != null)
                    pair.ContributeReplayHash(ref writer, mode);
            }
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("physics.2d.caches", 1);
        writer.WriteInt32(_processedPairKeys.Count);
        writer.WriteInt32(_pairs.Count);
        writer.WriteInt32(_pairsToRemove.Count);
        writer.WriteInt32(_cachedPairs.Count);
        writer.WriteInt32(_discreteResponsePairs.Count);
        writer.WriteInt32(_continuousCollisionPreparedToken);
        writer.WriteBool(_continuousCollisionPreparedMixedIndex);
        writer.WriteInt32(_planarContinuousCollisionCandidates.Count);
        writer.WriteInt32(_mixedContinuousCollisionCandidates.Count);
        writer.WriteInt32(_processedContinuousCollisionBodyIds.Count);
        writer.WriteInt32(_queuedContinuousCollisionHandoffIds.Count);
        writer.WriteInt32(_continuousCollisionHandoffQueue.Count);
        for (int i = 0; i < _continuousCollisionHandoffQueue.Count; i++)
            writer.WriteInt32(_continuousCollisionHandoffQueue[i]);
        writer.WriteInt32(LastBroadPhaseCandidateCount);
        writer.WriteInt32(LastContinuousCollisionIslandCount);
        writer.WriteInt32(LastContinuousCollisionIslandIterationCount);
        writer.WriteBool(LastContinuousCollisionIslandLimitReached);
    }
}
