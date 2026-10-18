//=======================================================================
// CollisionPair.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler.Hashing;
using FixedMathSharp.Chronicler;
using Gravitas.Materials;

namespace Gravitas.CollisionHandling;

public partial class CollisionPair
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.3d", 4);
        // Only an existing contact depends on these stamps; loading may republish noncontact geometry.
        writer.WriteBool(!_isColliding || _lastColliderAContactDependencyVersion == ColliderA.ContactDependencyVersion);
        writer.WriteBool(!_isColliding || _lastColliderBContactDependencyVersion == ColliderB.ContactDependencyVersion);
        writer.WriteBool(Active);
        writer.WriteInt32(ColliderA.ReplayOrdinal);
        writer.WriteInt32(ColliderB.ReplayOrdinal);
        writer.WriteEnum(CollisionType);
        writer.WriteUInt32(PartitionVersion);
        writer.WriteInt32(PairVersion);
        writer.WriteInt64(LastFrame);
        writer.WriteInt64(LastCollidedFrame);
        writer.WriteBool(_doPhysics);
        writer.WriteInt32(CullCounter);
        writer.WriteBool(_preventDistanceCull);
        writer.WriteBool(_isColliding);
        writer.WriteBool(_isCollidingChanged);
        writer.WriteFixed64(_fastCollideDistance);
        writer.WriteFixed64(_fastDistance);
        writer.WriteFixed64(_fastDistanceOffset);
        // An unculled noncontact refreshes both stamps before using them; other pairs depend on freshness.
        bool usesBroadPhaseCache = _isColliding || CullCounter > 0;
        writer.WriteBool(!usesBroadPhaseCache || _lastColliderABroadPhaseVersion == ColliderA.BroadPhaseVersion);
        writer.WriteBool(!usesBroadPhaseCache || _lastColliderBBroadPhaseVersion == ColliderB.BroadPhaseVersion);
        ContributeManifoldReplayHash(ref writer, Context, Manifold);
        ContributeWarmStartReplayHash(ref writer, _warmStart);

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("pair.3d.caches", 1);
        writer.WriteBool(_isPooledForDeactivation);
    }

    internal static void ContributeManifoldReplayHash(
        ref ChronicleHashWriter writer,
        GravitasWorldContext context,
        ContactManifold manifold,
        string sectionName = "manifold.3d")
    {
        writer.WriteSection(sectionName, 9);
        writer.WriteInt64(manifold.LastUpdatedFrame);
        writer.WriteInt32(manifold.Count);
        writer.WriteInt32(manifold.GroupCount);
        for (int groupIndex = 0; groupIndex < manifold.GroupCount; groupIndex++)
        {
            ref ContactGroup group = ref manifold.GetGroup(groupIndex);
            WriteGroupKey(ref writer, context, group.Key);
            writer.WriteInt32(group.Count);
            for (int i = 0; i < group.Count; i++)
            {
                ManifoldContact contact = group[i];
                writer.WriteUInt64(contact.ContactId);
                writer.WriteInt32(contact.FeatureNamespaceA);
                writer.WriteVector3d(contact.AnchorA.Origin);
                writer.WriteQuaternion(contact.AnchorA.Rotation);
                writer.WriteVector3d(contact.AnchorA.LocalPoint);
                writer.WriteVector3d(contact.AnchorA.LocalDisplacement);
                writer.WriteUInt64(contact.AnchorA.GetLocalFeatureHash64());
                writer.WriteInt32(contact.FeatureNamespaceB);
                writer.WriteVector3d(contact.AnchorB.Origin);
                writer.WriteQuaternion(contact.AnchorB.Rotation);
                writer.WriteVector3d(contact.AnchorB.LocalPoint);
                writer.WriteVector3d(contact.AnchorB.LocalDisplacement);
                writer.WriteUInt64(contact.AnchorB.GetLocalFeatureHash64());
                writer.WriteFixed64(contact.Depth);
                writer.WriteBool(contact.DepthIsClamped);
                writer.WriteVector3d(contact.Normal);
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
        ContactWarmStartCache warmStart)
    {
        writer.WriteSection("warm-start.3d", 3);
        writer.WriteInt32(warmStart.Count);
        writer.WriteInt32(warmStart.GroupCount);
        for (int groupIndex = 0; groupIndex < warmStart.GroupCount; groupIndex++)
        {
            ContactWarmStartGroup group = warmStart.GetGroup(groupIndex);
            WriteGroupKey(ref writer, Context, group.Key);
            writer.WriteInt32(group.Count);
            for (int i = 0; i < group.Count; i++)
            {
                writer.WriteUInt64(group.GetContactId(i));
                ContactWarmStartImpulse impulse = group.GetImpulseUnchecked(i);
                writer.WriteVector3d(impulse.Normal);
                writer.WriteFixed64(impulse.NormalImpulse);
                writer.WriteFixed64(impulse.TangentImpulse);
                writer.WriteFixed64(impulse.SecondaryTangentImpulse);
            }
        }
    }

    private static void WriteGroupKey(ref ChronicleHashWriter writer, GravitasWorldContext context, in ContactGroupKey group)
    {
        writer.WriteInt32(group.NamespaceA);
        writer.WriteInt32(group.NamespaceB);
        writer.WriteInt32(group.SurfaceA);
        writer.WriteInt32(group.SurfaceB);
        writer.WriteInt32(group.Region);
        WriteSource(ref writer, context, group.SourceIdA);
        WriteSource(ref writer, context, group.SourceIdB);
    }

    private static void WriteSource(ref ChronicleHashWriter writer, GravitasWorldContext context, int source)
    {
        // Runtime identity is available before hashing assigns live replay ordinals.
        writer.WriteInt32(source == 0 ? 0 : context.Physics.TryGetColliderById(source - 1, out var collider)
            ? collider!.ReplayOrdinal + 1 : -1);
    }
}
