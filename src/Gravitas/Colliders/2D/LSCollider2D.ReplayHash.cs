//=======================================================================
// LSCollider2D.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp.Chronicler;
using Gravitas.Materials;
using System;

namespace Gravitas.Colliders;

public abstract partial class LSCollider2D
{
    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("collider.2d", 3);
        writer.WriteInt32(_replayOrdinal);
        writer.WriteBool(_isActive);
        writer.WriteBool(_isTrigger);
        writer.WritePhysicsLayer(_layer);
        writer.WritePhysicsLayerMask(_ignoredCollisionLayers);
        WriteMaterial(ref writer, _material);
        writer.WriteEnum(Shape);
        writer.WriteInt32(Priority);
        writer.WriteVector2d(_localOffset);
        writer.WriteBool(_mixedHalfThicknessOverride.HasValue);
        if (_mixedHalfThicknessOverride.HasValue)
            writer.WriteFixed64(_mixedHalfThicknessOverride.Value);
        writer.WriteVector2d(Position);
        writer.WriteFixed64(Rotation);
        writer.WriteVector2d(LocalScale);
        writer.WriteVector2d(Center);
        writer.WriteFixed64(MinX);
        writer.WriteFixed64(MinY);
        writer.WriteFixed64(MaxX);
        writer.WriteFixed64(MaxY);
        writer.WriteVector3d(_mixedBounds3D.Min);
        writer.WriteVector3d(_mixedBounds3D.Max);
        writer.WriteFixed64(_mixedHalfThickness);
        writer.WriteFixed64(_mixedSlabCenterY);
        writer.WriteInt32(HierarchyChildCount);
        WriteReplayHierarchyKey(ref writer, HierarchyKey);
        WriteReplayHierarchyKey(ref writer, ParentKey);
        WriteReplayHierarchyKey(ref writer, TopParentKey);

        ContributeShapeReplayHash(ref writer);

        if (mode != GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            return;

        writer.WriteSection("collider.2d.caches", 1);
        writer.WriteUInt32(RuntimeShapeVersion);
        writer.WriteUInt32(_shapeVersion);
        writer.WriteInt32(_serviceIndex);
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

    private void ContributeShapeReplayHash(ref ChronicleHashWriter writer)
    {
        writer.WriteSection("collider.2d.shape", 1);
        switch (this)
        {
            case LSCircleCollider2D circle:
                writer.WriteFixed64(circle.Radius);
                writer.WriteFixed64(circle.ScaledRadius);
                break;

            case LSAABBoxCollider2D box:
                writer.WriteVector2d(box.Size);
                writer.WriteVector2d(box.ScaledSize);
                break;

            case LSCapsuleCollider2D capsule:
                writer.WriteFixed64(capsule.Radius);
                writer.WriteFixed64(capsule.Height);
                writer.WriteFixed64(capsule.ScaledRadius);
                writer.WriteFixed64(capsule.ScaledHeight);
                writer.WriteVector2d(capsule.SegmentStart);
                writer.WriteVector2d(capsule.SegmentEnd);
                break;

            case LSCompoundCollider2D compound:
                writer.WriteInt32(compound.PartCount);
                ReadOnlySpan<CompoundColliderPart2D> parts = compound.Parts;
                for (int i = 0; i < parts.Length; i++)
                    ContributeCompoundPartReplayHash(ref writer, parts[i]);
                break;
        }

        writer.WriteInt32(VertexCount);
        for (int i = 0; i < VertexCount; i++)
            writer.WriteVector2d(GetVertexUnchecked(i));
    }

    private void WriteReplayHierarchyKey(ref ChronicleHashWriter writer, ColliderHierarchyKey key)
    {
        if (!TryResolveReplayHierarchyOrdinal(key, out int replayOrdinal))
        {
            writer.WriteUInt64(0UL);
            return;
        }

        writer.WriteUInt64(((ulong)key.Dimension << 32) | (uint)replayOrdinal);
    }

    private bool TryResolveReplayHierarchyOrdinal(ColliderHierarchyKey key, out int replayOrdinal)
    {
        replayOrdinal = -1;
        if (!key.IsValid || _context == null)
            return false;

        if (key.Is2D)
        {
            _context.Physics2D.TryGetColliderById(key.Id, out LSCollider2D? collider);
            replayOrdinal = collider!.ReplayOrdinal;
            return replayOrdinal >= 0;
        }

        _context.Physics.TryGetColliderById(key.Id, out LSCollider? collider3D);
        replayOrdinal = collider3D!.ReplayOrdinal;
        return replayOrdinal >= 0;
    }

    private static void ContributeCompoundPartReplayHash(
        ref ChronicleHashWriter writer,
        CompoundColliderPart2D part)
    {
        writer.WriteVector2d(part.LocalOffset);
        writer.WriteFixed64(part.LocalRotation);
        writer.WriteVector2d(part.LocalScale);
        writer.WriteBool(part.HasMaterial);
        if (part.TryGetMaterial(out PhysicsMaterial material))
            WriteMaterial(ref writer, material);
        ContributeShapeDefinitionReplayHash(ref writer, part.Shape);
    }

    private static void ContributeShapeDefinitionReplayHash(
        ref ChronicleHashWriter writer,
        ColliderShapeDefinition2D definition)
    {
        writer.WriteEnum(definition.Kind);
        writer.WriteBool(definition.HasMaterial);
        if (definition.HasMaterial)
            WriteMaterial(ref writer, definition.Material);
        writer.WriteFixed64(definition.Radius);
        writer.WriteVector2d(definition.Size);
        writer.WriteInt32(definition.PolygonVertexCount);
        for (int i = 0; i < definition.PolygonVertexCount; i++)
            writer.WriteVector2d(definition.GetPolygonVertex(i));
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
