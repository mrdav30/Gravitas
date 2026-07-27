//=======================================================================
// CollisionDetectionMixed.Complex.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetectionMixed
{
    private static bool TryCompoundEmbedded2D(LSCompoundCollider compound, LSCollider2D embedded, out MixedContact contact)
    {
        bool found = false;
        MixedContact best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!part.Bounds.Intersects(embedded.MixedBounds3D)
                || !TryCollide(part, embedded, out MixedContact candidate))
            {
                continue;
            }

            candidate = candidate.WithFallbackMaterials(part.Material, embedded.Material);

            if (ContactSelectionPolicy.ShouldReplaceWithShallower(candidate, found, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
            return NoContact(out contact);

        contact = best;
        return true;
    }

    private static bool TryMeshEmbedded2D(LSMeshCollider mesh, LSCollider2D embedded, out MixedContact contact)
    {
        SwiftList<int> triangleBuffer = mesh.Context.CollisionScratch.MeshTriangleCandidatesA;
        FixedBoundVolume embeddedBounds = new(embedded.MixedBounds3D.Min, embedded.MixedBounds3D.Max);
        mesh.GetTrianglesInBounds(embeddedBounds, triangleBuffer);

        bool found = false;
        MixedContact best = default;
        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            int triangleIndex = triangleBuffer[i];
            MixedContact candidate;
            if (embedded is LSAABBoxCollider2D or LSPolygonCollider2D)
            {
                if (!TryGetTriangleConvexPrismContact(
                        mesh,
                        triangleIndex,
                        embedded,
                        out candidate))
                {
                    continue;
                }
            }
            else
            {
                if (!TryGetTriangleFiniteSurfaceContact(
                        mesh,
                        triangleIndex,
                        embedded,
                        out candidate))
                {
                    continue;
                }
            }

            if (ContactSelectionPolicy.ShouldReplaceWithShallower(candidate, found, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
            return NoContact(out contact);

        contact = best;
        return true;
    }

    private static bool TryGetTriangleFiniteSurfaceContact(
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
        var triangle = new FixedTriangle(first, second, third);
        Vector3d slabCenter = GetEmbeddedCenter3D(embedded);

        bool collided;
        FixedContactAnchors relation;
        if (embedded is LSCircleCollider2D circle)
        {
            collided = triangle.TryGetCircleSlabContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                slabCenter,
                circle.Rotation,
                circle.MixedHalfThickness,
                circle.ScaledRadius,
                out relation);
        }
        else if (embedded is LSCapsuleCollider2D capsule)
        {
            collided = triangle.TryGetCenteredCapsuleSlabContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                slabCenter,
                capsule.Rotation,
                Vector2d.Forward,
                capsule.AxisLength,
                capsule.ScaledRadius,
                capsule.MixedHalfThickness,
                out relation);
        }
        else
        {
            return NoContact(out contact);
        }

        return TryBuildFinitePrismContact(
            collided,
            relation,
            out contact);
    }
}
