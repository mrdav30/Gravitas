//=======================================================================
// CollisionPair2D.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;

namespace Gravitas;

internal sealed partial class CollisionPair2D
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.2d", 1);
        writer.WriteInt32(Id1);
        writer.WriteInt32(Id2);
        writer.WriteEnum(CollisionType);
        writer.WriteInt32(LastFrame);
        writer.WriteBool(_isColliding);
        ContributeManifoldReplayHash(ref writer, Manifold);
        ContributeWarmStartReplayHash(ref writer, _warmStart);
    }

    private static void ContributeManifoldReplayHash(
        ref GravitasReplayHashWriter writer,
        ContactManifold2D manifold)
    {
        writer.WriteSection("manifold.2d", 1);
        writer.WriteInt32(manifold.LastUpdatedFrame);
        writer.WriteInt32(manifold.Count);
        for (int i = 0; i < manifold.Count; i++)
        {
            ManifoldContact2D contact = manifold[i];
            writer.WriteUInt64(contact.ContactId);
            writer.WriteVector2d(contact.PointA);
            writer.WriteVector2d(contact.PointB);
            writer.WriteFixed64(contact.Depth);
            writer.WriteVector2d(contact.Normal);
        }
    }

    private static void ContributeWarmStartReplayHash(
        ref GravitasReplayHashWriter writer,
        ContactWarmStartCache2D warmStart)
    {
        writer.WriteSection("warm-start.2d", 1);
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
