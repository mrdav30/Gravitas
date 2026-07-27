//=======================================================================
// CollisionPairMixed.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp.Chronicler;
using Gravitas.Materials;

namespace Gravitas.CollisionHandling;

internal sealed partial class CollisionPairMixed
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.mixed", 5);
        writer.WriteInt32(Collider3D.ReplayOrdinal);
        writer.WriteInt32(Collider2D.ReplayOrdinal);
        writer.WriteUInt64(MixedColliderKey.CreateKey(Collider3D.ReplayOrdinal, Collider2D.ReplayOrdinal));
        writer.WriteInt32(LastFrame);
        writer.WriteBool(_isColliding);
        writer.WriteBool(_isTriggerPair);
        writer.WriteBool(Contact.HasContact);
        if (!Contact.HasContact)
            return;

        writer.WriteVector3d(Contact.Anchor3D.Origin);
        writer.WriteQuaternion(Contact.Anchor3D.Rotation);
        writer.WriteVector3d(Contact.Anchor3D.LocalPoint);
        writer.WriteVector3d(Contact.Anchor3D.LocalDisplacement);
        writer.WriteUInt64(Contact.Anchor3D.GetLocalFeatureHash64());
        writer.WriteVector3d(Contact.Anchor2D.Origin);
        writer.WriteQuaternion(Contact.Anchor2D.Rotation);
        writer.WriteVector3d(Contact.Anchor2D.LocalPoint);
        writer.WriteVector3d(Contact.Anchor2D.LocalDisplacement);
        writer.WriteUInt64(Contact.Anchor2D.GetLocalFeatureHash64());
        writer.WriteVector3d(Contact.Normal3DTo2D);
        writer.WriteFixed64(Contact.Depth);
        writer.WriteBool(Contact.DepthIsClamped);
        writer.WriteBool(Contact.HasMaterialOverride);
        if (!Contact.HasMaterialOverride)
            return;

        WriteMaterial(ref writer, Contact.Material3D);
        WriteMaterial(ref writer, Contact.Material2D);
    }

    private static void WriteMaterial(ref ChronicleHashWriter writer, PhysicsMaterial material)
    {
        writer.WriteFixed64(material.StaticFriction);
        writer.WriteFixed64(material.DynamicFriction);
        writer.WriteFixed64(material.Restitution);
        writer.WriteEnum(material.FrictionCombine);
        writer.WriteEnum(material.RestitutionCombine);
    }
}
