//=======================================================================
// GravitasMixedCollisionService.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

internal sealed partial class GravitasMixedCollisionService
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        SwiftList<LSCollider> replay3DColliders = _context.Physics.PrepareReplayColliders();
        SwiftList<LSCollider2D> replay2DColliders = _context.Physics2D.PrepareReplayColliders();

        writer.WriteSection("physics.mixed", 2);
        writer.WriteUInt32(Version);
        writer.WriteInt32(ActivePairCount);

        for (int i = 0; i < replay3DColliders.Count; i++)
        {
            int collider3DId = replay3DColliders[i].Id;
            for (int j = 0; j < replay2DColliders.Count; j++)
            {
                int collider2DId = replay2DColliders[j].Id;
                ulong key = MixedColliderKey.CreateKey(collider3DId, collider2DId);
                if (_pairs.TryGetValue(key, out CollisionPairMixed? pair) && pair != null)
                    pair.ContributeReplayHash(ref writer, mode);
            }
        }

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("physics.mixed.caches", 1);
        writer.WriteInt32(ActivePartitionCount);
        writer.WriteInt32(InactivePartitionCount);
        writer.WriteInt32(RetainedPartitionCount);
        writer.WriteInt32(PooledPairCount);
        writer.WriteInt32(LastBroadPhaseCandidateCount);
        writer.WriteInt32(SimulateCount);
        writer.WriteInt32(LateSimulateCount);
        writer.WriteInt32(VisualizeCount);
        writer.WriteInt32(_retainedPartitionRetirementCursor);
        writer.WriteInt32(_cached3DQueryRefreshFrame);
        writer.WriteInt32(_cached3DQueryRefreshLateToken);
        writer.WriteInt32(_cached2DQueryRefreshFrame);
        writer.WriteInt32(_cached2DQueryRefreshLateToken);
    }
}
