//=======================================================================
// GravitasPhysicsService.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas;

public sealed partial class GravitasPhysicsService
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("physics.3d", 1);
        writer.WriteBool(SimulatePhysics);
        writer.WriteInt32(PeakColliderCount);
        writer.WriteInt32(AssimilatedBodyCount);
        writer.WriteInt32(AssimilatedColliderCount);
        writer.WriteInt32(_dynamicBodies.PeakCount);

        for (int colliderId = 1; colliderId <= PeakColliderCount; colliderId++)
        {
            bool hasCollider = TryGetColliderById(colliderId, out LSCollider? collider);
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

        writer.WriteSection("physics.3d.pairs", 1);
        for (int colliderId = 1; colliderId <= PeakColliderCount; colliderId++)
        {
            if (!TryGetColliderById(colliderId, out LSCollider? collider))
                continue;

            for (int otherId = colliderId + 1; otherId <= PeakColliderCount; otherId++)
            {
                if (collider!.TryGetCollisionPair(otherId, out CollisionPair? pair) && pair != null)
                    pair.ContributeReplayHash(ref writer, mode);
            }
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("physics.3d.caches", 1);
        writer.WriteInt32(_cachedColliderIds.Count);
        foreach (int cachedColliderId in _cachedColliderIds)
            writer.WriteInt32(cachedColliderId);
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
