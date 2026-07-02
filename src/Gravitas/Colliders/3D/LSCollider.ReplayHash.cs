//=======================================================================
// LSCollider.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Materials;
using System;

namespace Gravitas.Colliders;

public abstract partial class LSCollider
{
    internal void ContributeReplayHash(
        ref GravitasReplayHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("collider.3d", 2);
        writer.WriteInt32(_id);
        writer.WriteBool(_active);
        writer.WriteBool(_isTrigger);
        writer.WritePhysicsLayer(_layer);
        writer.WritePhysicsLayerMask(_ignoredCollisionLayers);
        WriteMaterial(ref writer, _material);
        writer.WriteBool(_preventCulling);
        writer.WriteEnum(Shape);
        writer.WriteInt32(Priority);
        writer.WriteVector3d(_offset);
        writer.WriteFixed64(_radius);
        writer.WriteVector3d(_size);
        writer.WriteVector3d(Position);
        writer.WriteQuaternion(Rotation);
        writer.WriteVector3d(LocalScale);
        writer.WriteVector3d(Center);
        writer.WriteVector3d(BoundsMin);
        writer.WriteVector3d(BoundsMax);
        writer.WriteInt32(HierarchyChildCount);
        writer.WriteUInt64(HierarchyKey.Packed);
        writer.WriteUInt64(ParentKey.Packed);
        writer.WriteUInt64(TopParentKey.Packed);

        ContributeShapeReplayHash(ref writer);

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("collider.3d.caches", 1);
        writer.WriteUInt32(RuntimeShapeVersion);
        writer.WriteBool(IsPartitioned);
        writer.WriteBool(IsMixedPartitioned);
        writer.WriteInt32(PartitionKind);
        writer.WriteInt32(MixedPartitionKind);
        writer.WriteUInt32(BroadPhaseVersion);
        writer.WriteUInt32(RaycastVersion);
        writer.WriteUInt32(CircleQueryVersion);
        writer.WriteInt32(CollisionPairCount);
        writer.WriteInt32(CollisionPairHolderCount);
    }

    private void ContributeShapeReplayHash(ref GravitasReplayHashWriter writer)
    {
        writer.WriteSection("collider.3d.shape", 1);
        switch (this)
        {
            case LSMeshCollider mesh:
                writer.WriteEnum(mesh.Mode);
                writer.WriteEnum(mesh.InertiaPolicy);
                writer.WriteInt32(mesh.Mesh.VertexCount);
                for (int i = 0; i < mesh.Mesh.LocalVertices.Length; i++)
                    writer.WriteVector3d(mesh.Mesh.LocalVertices[i]);
                writer.WriteInt32(mesh.Mesh.Triangles.Length);
                for (int i = 0; i < mesh.Mesh.Triangles.Length; i++)
                    writer.WriteInt32(mesh.Mesh.Triangles[i]);
                break;

            case LSCompoundCollider compound:
                writer.WriteInt32(compound.PartCount);
                ReadOnlySpan<CompoundColliderPart> parts = compound.Parts;
                for (int i = 0; i < parts.Length; i++)
                    ContributeCompoundPartReplayHash(ref writer, parts[i]);
                break;

            default:
                writer.WriteFixed64(Area);
                break;
        }
    }

    private static void ContributeCompoundPartReplayHash(
        ref GravitasReplayHashWriter writer,
        CompoundColliderPart part)
    {
        writer.WriteVector3d(part.LocalOffset);
        writer.WriteQuaternion(part.LocalRotation);
        writer.WriteVector3d(part.LocalScale);
        writer.WriteBool(part.HasMaterial);
        if (part.TryGetMaterial(out PhysicsMaterial material))
            WriteMaterial(ref writer, material);
        ContributeShapeDefinitionReplayHash(ref writer, part.Shape);
    }

    private static void ContributeShapeDefinitionReplayHash(
        ref GravitasReplayHashWriter writer,
        ColliderShapeDefinition definition)
    {
        writer.WriteEnum(definition.Kind);
        writer.WriteBool(definition.HasMaterial);
        if (definition.HasMaterial)
            WriteMaterial(ref writer, definition.Material);
        writer.WriteFixed64(definition.Radius);
        writer.WriteFixed64(definition.Height);
        writer.WriteVector3d(definition.Size);
        writer.WriteEnum(definition.MeshInertiaPolicy);
        writer.WriteInt32(definition.MeshVertexCount);
        for (int i = 0; i < definition.MeshVertexCount; i++)
            writer.WriteVector3d(definition.GetMeshVertex(i));
        writer.WriteInt32(definition.MeshTriangleIndexCount);
        for (int i = 0; i < definition.MeshTriangleIndexCount; i++)
            writer.WriteInt32(definition.GetMeshTriangleIndex(i));
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
