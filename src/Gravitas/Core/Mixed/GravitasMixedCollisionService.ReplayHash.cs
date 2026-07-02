//=======================================================================
// GravitasMixedCollisionService.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas;

internal sealed partial class GravitasMixedCollisionService
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("physics.mixed", 1);
        writer.WriteUInt32(Version);
        writer.WriteInt32(ActivePairCount);

        int peak3D = _context.Physics.PeakColliderCount;
        for (int collider3DId = 1; collider3DId <= peak3D; collider3DId++)
        {
            if (!_context.Physics.TryGetColliderById(collider3DId, out LSCollider? _))
                continue;

            for (int collider2DId = 1; collider2DId < _context.Physics2D.NextColliderIdForReplayHash; collider2DId++)
            {
                if (!_context.Physics2D.TryGetColliderById(collider2DId, out LSCollider2D? _))
                    continue;

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
