//=======================================================================
// CollisionPairMixed.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Materials;

namespace Gravitas.CollisionHandling;

internal sealed partial class CollisionPairMixed
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("pair.mixed", 1);
        writer.WriteInt32(Collider3DId);
        writer.WriteInt32(Collider2DId);
        writer.WriteUInt64(Key);
        writer.WriteInt32(LastFrame);
        writer.WriteBool(_isColliding);
        writer.WriteBool(_isTriggerPair);
        writer.WriteBool(Contact.HasContact);
        if (!Contact.HasContact)
            return;

        writer.WriteVector3d(Contact.Point3D);
        writer.WriteVector3d(Contact.Point2D);
        writer.WriteVector3d(Contact.Normal3DTo2D);
        writer.WriteFixed64(Contact.Depth);
        writer.WriteBool(Contact.HasMaterialOverride);
        if (!Contact.HasMaterialOverride)
            return;

        WriteMaterial(ref writer, Contact.Material3D);
        WriteMaterial(ref writer, Contact.Material2D);
    }

    private static void WriteMaterial(ref GravitasReplayHashWriter writer, PhysicsMaterial material)
    {
        writer.WriteFixed64(material.StaticFriction);
        writer.WriteFixed64(material.DynamicFriction);
        writer.WriteFixed64(material.Restitution);
        writer.WriteEnum(material.FrictionCombine);
        writer.WriteEnum(material.RestitutionCombine);
    }
}
