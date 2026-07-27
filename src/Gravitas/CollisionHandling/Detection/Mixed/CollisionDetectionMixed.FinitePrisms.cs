//=======================================================================
// CollisionDetectionMixed.FinitePrisms.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetectionMixed
{
    private static bool TryCuboidEmbedded2D(
        LSCuboidCollider cuboid,
        LSCollider2D embedded,
        out MixedContact contact)
    {
        if (embedded is LSCircleCollider2D circle)
        {
            return TryGetCuboidRelationContact(
                cuboid.OrientedBox.TryGetCircleSlabContact(
                    GetEmbeddedCenter3D(circle),
                    circle.Rotation,
                    circle.MixedHalfThickness,
                    circle.ScaledRadius,
                    out FixedContactAnchors anchors),
                anchors,
                out contact);
        }

        if (embedded is LSCapsuleCollider2D capsule)
        {
            return TryGetCuboidRelationContact(
                cuboid.OrientedBox.TryGetCenteredCapsuleSlabContact(
                    GetEmbeddedCenter3D(capsule),
                    capsule.Rotation,
                    Vector2d.Forward,
                    capsule.AxisLength,
                    capsule.ScaledRadius,
                    capsule.MixedHalfThickness,
                    out FixedContactAnchors anchors),
                anchors,
                out contact);
        }

        if (embedded is LSPolygonCollider2D polygon)
        {
            return TryGetCuboidPrismContact(
                cuboid,
                embedded,
                polygon.ScaledLocalVertices,
                out contact);
        }

        Vector2d halfExtents =
            ((LSAABBoxCollider2D)embedded).ScaledHalfExtents;
        Span<Vector2d> centerRelativeVertices = stackalloc Vector2d[4];
        centerRelativeVertices[0] = new Vector2d(-halfExtents.X, -halfExtents.Y);
        centerRelativeVertices[1] = new Vector2d(halfExtents.X, -halfExtents.Y);
        centerRelativeVertices[2] = new Vector2d(halfExtents.X, halfExtents.Y);
        centerRelativeVertices[3] = new Vector2d(-halfExtents.X, halfExtents.Y);
        return TryGetCuboidPrismContact(
            cuboid,
            embedded,
            centerRelativeVertices,
            out contact);
    }

    private static bool TryGetCuboidPrismContact(
        LSCuboidCollider cuboid,
        LSCollider2D embedded,
        ReadOnlySpan<Vector2d> centerRelativeVertices,
        out MixedContact contact)
    {
        Vector3d embeddedCenter = GetEmbeddedCenter3D(embedded);
        return TryGetCuboidRelationContact(
            cuboid.OrientedBox.TryGetConvexPrismContact(
                embeddedCenter,
                embedded.ConvexRotation,
                centerRelativeVertices,
                embedded.MixedHalfThickness,
                out FixedContactAnchors anchors),
            anchors,
            out contact);
    }

    private static bool TryGetCuboidRelationContact(
        bool collided,
        in FixedContactAnchors anchors,
        out MixedContact contact)
    {
        if (!collided)
            return NoContact(out contact);

        contact = new MixedContact(
            new ContactAnchor(anchors.FirstAnchor),
            new ContactAnchor(anchors.SecondAnchor),
            anchors.Normal,
            anchors.Depth,
            anchors.DepthIsClamped);
        return true;
    }

    private static bool TryGetTriangleConvexPrismContact(
        LSMeshCollider mesh,
        int triangleIndex,
        LSCollider2D embedded,
        out MixedContact contact)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        Span<Vector2d> boxOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> prismOffsets = GetConvexPrismOffsets(
            embedded,
            boxOffsets);
        bool collided = FixedConvexPrismRelations.TryGetTriangleContact(
            mesh.Mesh.Origin,
            mesh.Mesh.Rotation,
            new FixedTriangle(first, second, third),
            GetEmbeddedCenter3D(embedded),
            embedded.ConvexRotation,
            prismOffsets,
            embedded.MixedHalfThickness,
            out FixedContactAnchors relation);
        return TryBuildFinitePrismContact(
            collided,
            relation,
            out contact);
    }

    private static bool TryGetSphereConvexPrismContact(
        LSSphereCollider sphere,
        LSCollider2D embedded,
        out MixedContact contact)
    {
        Span<Vector2d> boxOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = GetConvexPrismOffsets(
            embedded,
            boxOffsets);
        bool collided = FixedConvexPrismRelations.TryGetSphereContact(
            sphere.Center,
            sphere.ScaledRadius,
            GetEmbeddedCenter3D(embedded),
            embedded.ConvexRotation,
            offsets,
            embedded.MixedHalfThickness,
            out FixedContactAnchors relation);
        return TryBuildFinitePrismContact(
            collided,
            relation,
            out contact);
    }

    private static bool TryGetCapsuleConvexPrismContact(
        LSCapsuleCollider capsule,
        LSCollider2D embedded,
        out MixedContact contact)
    {
        Span<Vector2d> boxOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = GetConvexPrismOffsets(
            embedded,
            boxOffsets);
        bool collided =
            FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
                capsule.Center,
                capsule.Rotation,
                Vector3d.Up,
                capsule.AxisLength,
                capsule.ScaledRadius,
                GetEmbeddedCenter3D(embedded),
                embedded.ConvexRotation,
                offsets,
                embedded.MixedHalfThickness,
                out FixedContactAnchors relation);
        return TryBuildFinitePrismContact(
            collided,
            relation,
            out contact);
    }

    private static bool TryGetCylinderConvexPrismContact(
        LSCylinderCollider cylinder,
        LSCollider2D embedded,
        out MixedContact contact)
    {
        Span<Vector2d> boxOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = GetConvexPrismOffsets(
            embedded,
            boxOffsets);
        bool collided =
            FixedConvexPrismRelations.TryGetCenteredCylinderContact(
                cylinder.Center,
                cylinder.Rotation,
                Vector3d.Up,
                cylinder.Height,
                cylinder.ScaledRadius,
                GetEmbeddedCenter3D(embedded),
                embedded.ConvexRotation,
                offsets,
                embedded.MixedHalfThickness,
                out FixedContactAnchors relation);
        return TryBuildFinitePrismContact(
            collided,
            relation,
            out contact);
    }

    private static bool TryGetConeConvexPrismContact(
        LSConeCollider cone,
        LSCollider2D embedded,
        out MixedContact contact)
    {
        Span<Vector2d> boxOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = GetConvexPrismOffsets(
            embedded,
            boxOffsets);
        bool collided =
            FixedConvexPrismRelations.TryGetCenteredConeContact(
                cone.Center,
                cone.Rotation,
                Vector3d.Up,
                cone.Height,
                cone.ScaledRadius,
                GetEmbeddedCenter3D(embedded),
                embedded.ConvexRotation,
                offsets,
                embedded.MixedHalfThickness,
                out FixedContactAnchors relation);
        return TryBuildFinitePrismContact(
            collided,
            relation,
            out contact);
    }

    private static ReadOnlySpan<Vector2d> GetConvexPrismOffsets(
        LSCollider2D embedded,
        Span<Vector2d> boxOffsets)
    {
        if (embedded is LSPolygonCollider2D polygon)
            return polygon.ScaledLocalVertices;

        Vector2d halfExtents =
            ((LSAABBoxCollider2D)embedded).ScaledHalfExtents;
        boxOffsets[0] = new Vector2d(-halfExtents.X, -halfExtents.Y);
        boxOffsets[1] = new Vector2d(halfExtents.X, -halfExtents.Y);
        boxOffsets[2] = new Vector2d(halfExtents.X, halfExtents.Y);
        boxOffsets[3] = new Vector2d(-halfExtents.X, halfExtents.Y);
        return boxOffsets;
    }

    private static bool TryBuildFinitePrismContact(
        bool collided,
        in FixedContactAnchors relation,
        out MixedContact contact)
    {
        if (!collided)
            return NoContact(out contact);

        contact = new MixedContact(
            new ContactAnchor(relation.FirstAnchor),
            new ContactAnchor(relation.SecondAnchor),
            relation.Normal,
            relation.Depth,
            relation.DepthIsClamped);
        return true;
    }
}
