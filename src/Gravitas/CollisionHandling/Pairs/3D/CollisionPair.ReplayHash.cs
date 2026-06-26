//=======================================================================
// CollisionPair.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

public partial class CollisionPair
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.3d", 1);
        writer.WriteBool(Active);
        writer.WriteInt32(Id1);
        writer.WriteInt32(Id2);
        writer.WriteEnum(CollisionType);
        writer.WriteUInt32(PartitionVersion);
        writer.WriteInt32(PairVersion);
        writer.WriteInt32(LastFrame);
        writer.WriteInt32(LastCollidedFrame);
        writer.WriteBool(_doPhysics);
        writer.WriteBool(_preventCulling);
        writer.WriteInt32(CullCounter);
        writer.WriteBool(_preventDistanceCull);
        writer.WriteBool(_isColliding);
        writer.WriteBool(_isCollidingChanged);
        writer.WriteFixed64(_fastCollideDistance);
        writer.WriteFixed64(_fastDistance);
        writer.WriteFixed64(_fastDistanceOffset);
        writer.WriteUInt32(_lastColliderABroadPhaseVersion);
        writer.WriteUInt32(_lastColliderBBroadPhaseVersion);
        ContributeManifoldReplayHash(ref writer, Manifold);
        ContributeWarmStartReplayHash(ref writer, _warmStart);

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("pair.3d.caches", 1);
        writer.WriteBool(_isPooledForDeactivation);
    }

    private static void ContributeManifoldReplayHash(
        ref GravitasReplayHashWriter writer,
        ContactManifold manifold)
    {
        writer.WriteSection("manifold.3d", 1);
        writer.WriteInt32(manifold.LastUpdatedFrame);
        writer.WriteInt32(manifold.Count);
        for (int i = 0; i < manifold.Count; i++)
        {
            ManifoldContact contact = manifold[i];
            writer.WriteUInt64(contact.ContactId);
            writer.WriteVector3d(contact.PointA);
            writer.WriteVector3d(contact.PointB);
            writer.WriteFixed64(contact.Depth);
            writer.WriteVector3d(contact.Normal);
            writer.WriteVector3d(contact.ImmovableCollisionDirection);
        }
    }

    private static void ContributeWarmStartReplayHash(
        ref GravitasReplayHashWriter writer,
        ContactWarmStartCache warmStart)
    {
        writer.WriteSection("warm-start.3d", 1);
        writer.WriteInt32(warmStart.Count);
        for (int i = 0; i < warmStart.Count; i++)
        {
            writer.WriteUInt64(warmStart.GetContactIdForReplayHash(i));
            ContactWarmStartImpulse impulse = warmStart.GetImpulseForReplayHash(i);
            writer.WriteVector3d(impulse.Normal);
            writer.WriteFixed64(impulse.NormalImpulse);
            writer.WriteFixed64(impulse.TangentImpulse);
            writer.WriteFixed64(impulse.SecondaryTangentImpulse);
        }
    }
}
