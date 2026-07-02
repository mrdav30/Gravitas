//=======================================================================
// CollisionPair2D.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Chronicler;
using Chronicler;
using Gravitas.CollisionHandling;
using Gravitas.Materials;

namespace Gravitas;

internal sealed partial class CollisionPair2D
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
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
        ref ChronicleHashWriter writer,
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
            writer.WriteBool(contact.HasMaterialOverride);
            if (contact.HasMaterialOverride)
            {
                WriteMaterial(ref writer, contact.MaterialA);
                WriteMaterial(ref writer, contact.MaterialB);
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

    private static void ContributeWarmStartReplayHash(
        ref ChronicleHashWriter writer,
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
