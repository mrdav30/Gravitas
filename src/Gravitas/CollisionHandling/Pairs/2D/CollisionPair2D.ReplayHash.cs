//=======================================================================
// CollisionPair2D.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler.Hashing;
using FixedMathSharp.Chronicler;
using Gravitas.CollisionHandling;
using Gravitas.Materials;

namespace Gravitas;

internal sealed partial class CollisionPair2D
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.2d", 4);
        writer.WriteInt32(ColliderA.ReplayOrdinal);
        writer.WriteInt32(ColliderB.ReplayOrdinal);
        writer.WriteEnum(CollisionType);
        writer.WriteInt64(LastFrame);
        writer.WriteBool(_isColliding);
        // Only an existing contact depends on these stamps; loading may republish noncontact geometry.
        writer.WriteBool(!_isColliding || _lastColliderAContactDependencyVersion == ColliderA.ContactDependencyVersion);
        writer.WriteBool(!_isColliding || _lastColliderBContactDependencyVersion == ColliderB.ContactDependencyVersion);
        writer.WriteBool(_wasTriggerPair);
        ContributeManifoldReplayHash(ref writer, ColliderA.Context, Manifold);
        ContributeWarmStartReplayHash(ref writer, _warmStart);
    }

    private static void ContributeManifoldReplayHash(
        ref ChronicleHashWriter writer,
        GravitasWorldContext context,
        ContactManifold2D manifold)
    {
        writer.WriteSection("manifold.2d", 6);
        writer.WriteInt64(manifold.LastUpdatedFrame);
        writer.WriteInt32(manifold.Count);
        writer.WriteInt32(manifold.GroupCount);
        for (int g = 0; g < manifold.GroupCount; g++)
        {
            ref ContactGroup2D group = ref manifold.GetGroup(g);
            WriteGroupKey(ref writer, context, group.Key);
            writer.WriteInt32(group.Count);
            for (int i = 0; i < group.Count; i++)
            {
                ManifoldContact2D contact = group[i];
                writer.WriteUInt64(contact.ContactId);
                writer.WriteInt32(contact.FeatureNamespaceA);
                writer.WriteVector2d(contact.AnchorA.Origin);
                writer.WriteVector2d(contact.AnchorA.LocalPoint);
                writer.WriteVector2d(contact.AnchorA.LocalDisplacement);
                writer.WriteUInt64(contact.AnchorA.GetLocalFeatureHash64());
                writer.WriteFixed64(contact.AnchorA.Rotation);
                writer.WriteInt32(contact.FeatureNamespaceB);
                writer.WriteVector2d(contact.AnchorB.Origin);
                writer.WriteVector2d(contact.AnchorB.LocalPoint);
                writer.WriteVector2d(contact.AnchorB.LocalDisplacement);
                writer.WriteUInt64(contact.AnchorB.GetLocalFeatureHash64());
                writer.WriteFixed64(contact.AnchorB.Rotation);
                writer.WriteFixed64(contact.Depth);
                writer.WriteBool(contact.DepthIsClamped);
                writer.WriteVector2d(contact.Normal);
                writer.WriteBool(contact.HasMaterialOverride);
                if (contact.HasMaterialOverride)
                {
                    WriteMaterial(ref writer, contact.MaterialA);
                    WriteMaterial(ref writer, contact.MaterialB);
                }
            }
        }
    }

    private static void WriteMaterial(ref ChronicleHashWriter writer, PhysicsMaterial material)
    {
        writer.WriteFixed64(material.StaticFriction);
        writer.WriteFixed64(material.DynamicFriction);
        writer.WriteFixed64(material.Restitution);
        writer.WriteEnum(material.FrictionCombine);
        writer.WriteEnum(material.RestitutionCombine);
    }

    private void ContributeWarmStartReplayHash(
        ref ChronicleHashWriter writer,
        ContactWarmStartCache2D warmStart)
    {
        writer.WriteSection("warm-start.2d", 2);
        writer.WriteInt32(warmStart.Count);
        writer.WriteInt32(warmStart.GroupCount);
        for (int g = 0; g < warmStart.GroupCount; g++)
        {
            ContactWarmStartGroup2D group = warmStart.GetGroup(g);
            WriteGroupKey(ref writer, ColliderA.Context, group.Key);
            writer.WriteInt32(group.Count);
            for (int point = 0; point < group.Count; point++)
            {
                writer.WriteUInt64(group.GetContactId(point));
                ContactWarmStartImpulse impulse = group.GetImpulseUnchecked(point);
                writer.WriteVector3d(impulse.Normal);
                writer.WriteFixed64(impulse.NormalImpulse);
                writer.WriteFixed64(impulse.TangentImpulse);
                writer.WriteFixed64(impulse.SecondaryTangentImpulse);
            }
        }
    }

    private static void WriteGroupKey(ref ChronicleHashWriter writer,
        GravitasWorldContext context, in ContactGroupKey group)
    {
        writer.WriteInt32(group.NamespaceA);
        writer.WriteInt32(group.NamespaceB);
        writer.WriteInt32(group.SurfaceA);
        writer.WriteInt32(group.SurfaceB);
        writer.WriteInt32(group.Region);
        WriteSource(ref writer, context, group.SourceIdA);
        WriteSource(ref writer, context, group.SourceIdB);
    }

    private static void WriteSource(ref ChronicleHashWriter writer,
        GravitasWorldContext context, int source) =>
        writer.WriteInt32(source == 0 ? 0
            : context.Physics2D.TryGetColliderById(source - 1, out var collider)
                ? collider!.ReplayOrdinal + 1 : -1);
}
