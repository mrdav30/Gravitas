//=======================================================================
// CollisionPairMixed.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler.Hashing;

namespace Gravitas.CollisionHandling;

internal sealed partial class CollisionPairMixed
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.mixed", 9);
        writer.WriteInt32(Collider3D.ReplayOrdinal);
        writer.WriteInt32(Collider2D.ReplayOrdinal);
        writer.WriteUInt64(MixedColliderKey.CreateKey(Collider3D.ReplayOrdinal, Collider2D.ReplayOrdinal));
        writer.WriteInt64(LastFrame);
        writer.WriteBool(_isColliding);
        writer.WriteBool(_isTriggerPair);
        // Only an existing contact depends on these stamps; loading may republish noncontact geometry.
        writer.WriteBool(!_isColliding || _lastCollider3DContactDependencyVersion == Collider3D.ContactDependencyVersion);
        writer.WriteBool(!_isColliding || _lastCollider2DContactDependencyVersion == Collider2D.ContactDependencyVersion);
        writer.WriteBool(!_isColliding || !_hasManifoldCache || HasCurrentManifoldCache);
        CollisionPair.ContributeManifoldReplayHash(ref writer, Context, Manifold, "manifold.mixed");
        // Per-step impulses are rebuilt before the next solve and cannot
        // affect future authoritative state, unlike cross-frame 3D warm starts.
    }
}
