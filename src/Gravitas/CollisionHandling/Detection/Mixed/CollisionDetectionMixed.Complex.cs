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

/// <content>Builds independent finite mesh/slab regions without treating authored seams as physical features.</content>
public static partial class CollisionDetectionMixed
{
    // Interval probes need exact admission; depth, anchors and closing rows are
    // materialized once at the retained witness using the ordinary manifold path.
    internal static bool IntersectsForContinuousCollision(LSCollider collider3D, LSCollider2D embedded,
        Fixed64 padding = default, int surfaceOwner = -1, MeshSlabFaceRegions? handledFaces = null)
    {
        if (collider3D is not LSMeshCollider mesh
            || embedded is not (LSCircleCollider2D or LSCapsuleCollider2D or LSAABBoxCollider2D or LSPolygonCollider2D)
            || mesh.Mode == MeshColliderMode.Convex && mesh.IsClosedSurface)
            // A boundary-only query cannot exclude a solid hull's interior.
            return padding > Fixed64.Zero || TryBuildManifold(collider3D, embedded, collider3D.Context.CollisionScratch.MixedManifold);
        Fixed64 length = embedded is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero;
        Fixed64 radius = embedded is LSCircleCollider2D circle ? circle.ScaledRadius
            : embedded is LSCapsuleCollider2D stadium ? stadium.ScaledRadius : Fixed64.Zero;
        Fixed64 halfThickness = embedded.MixedHalfThickness;
        Vector3d center = GetEmbeddedCenter3D(embedded);
        FixedBoundBox bounds = embedded.MixedBounds3D;
        if (padding > Fixed64.Zero)
        {
            // This is an interval enclosure, never physical admission. A world
            // motion ball fits inside a stadium enlarged radially and at both caps.
            if (embedded is not (LSCircleCollider2D or LSCapsuleCollider2D)
                || !Fixed64.TryAdd(radius, padding, out radius)
                || !Fixed64.TryAdd(halfThickness, padding, out halfThickness)
                || !Fixed64.TryAdd(embedded.CanonicalCenteredProxyRadius, padding, out Fixed64 reach)) return true;
            // The existing affine section kernel proves these widths. Wider
            // trusted search padding remains unresolved rather than being capped.
            if (!mesh.Mesh.GeometryFrame.IsRigid && (radius > ColliderGeometryPolicy.MaximumGeometryReach
                || halfThickness > ColliderGeometryPolicy.MaximumGeometryReach)) return true;
            Vector3d scope = new(reach, halfThickness, reach);
            if (!Vector3d.TrySubtract(center, scope, out Vector3d minimum)
                || !Vector3d.TryAdd(center, scope, out Vector3d maximum)) return true;
            bounds = FixedBoundBox.FromMinMax(minimum, maximum);
        }
        if (!mesh.Bounds.Intersects(bounds)) return false;
        Fixed64 yaw = embedded is LSAABBoxCollider2D ? Fixed64.Zero : embedded.Rotation;
        var section = new FiniteSlabSectionFrame(mesh.Mesh.GeometryFrame,
            center, FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw), length, radius, halfThickness);
        Span<Vector2d> box = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = GetConvexPrismOffsets(embedded, box);
        // A solved face still owns its true finite perimeter. Curved sections
        // admit exact retained subsegments; unknown prism cuts stay unresolved.
        if (handledFaces != null)
            return !offsets.IsEmpty || handledFaces.IntersectsContinuousBoundary(mesh, surfaceOwner, section);
        SwiftList<int> candidates = mesh.Context.CollisionScratch.MeshTriangleCandidatesA;
        mesh.GetTrianglesInBounds(new FixedBoundVolume(bounds.Min, bounds.Max), candidates);
        foreach (int triangle in candidates.AsReadOnlySpan())
        {
            if (surfaceOwner >= 0 && mesh.Mesh.GetManifoldSurfaceOwner(triangle) != surfaceOwner) continue;
            mesh.Mesh.GetGeometryTriangleVertices(triangle, out Vector3d first, out Vector3d second, out Vector3d third);
            if (section.IntersectsTriangle(new FixedTriangle(first, second, third), offsets)) return true;
        }
        return false;
    }

    private static void AppendMeshEmbedded2D(LSMeshCollider mesh, LSCollider2D embedded,
        ContactManifold manifold, int namespace3D, int namespace2D)
    {
        if (embedded is not (LSCircleCollider2D or LSCapsuleCollider2D or LSAABBoxCollider2D or LSPolygonCollider2D))
            return;
        SwiftList<int> triangleBuffer = mesh.Context.CollisionScratch.MeshTriangleCandidatesA;
        FixedBoundVolume embeddedBounds = new(embedded.MixedBounds3D.Min, embedded.MixedBounds3D.Max);
        mesh.GetTrianglesInBounds(embeddedBounds, triangleBuffer);
        Vector3d center = GetEmbeddedCenter3D(embedded);
        Fixed64 length = embedded is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero;
        Fixed64 radius = embedded is LSCircleCollider2D circle ? circle.ScaledRadius
            : embedded is LSCapsuleCollider2D stadium ? stadium.ScaledRadius : Fixed64.Zero;
        Fixed64 yaw = embedded is LSAABBoxCollider2D ? Fixed64.Zero : embedded.Rotation;
        Span<Vector2d> box = stackalloc Vector2d[4];
        AppendMeshSlabContacts(mesh, embedded, center, FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw),
            length, radius, embedded.MixedHalfThickness, GetConvexPrismOffsets(embedded, box),
            triangleBuffer.AsReadOnlySpan(), manifold, namespace3D, namespace2D);
    }

    internal static bool AppendMeshCircleSlabContacts(LSMeshCollider mesh, Vector3d center,
        Fixed64 radius, Fixed64 halfThickness, ReadOnlySpan<int> candidates, ContactManifold manifold)
    {
        AppendMeshSlabContacts(mesh, null, center, FixedQuaternion.Identity, Fixed64.Zero, radius,
            halfThickness, ReadOnlySpan<Vector2d>.Empty, candidates, manifold, 0, 0);
        return manifold.HasContact;
    }

    private static void AppendMeshSlabContacts(LSMeshCollider collider, LSCollider2D? embedded,
        Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
        ReadOnlySpan<Vector2d> prismOffsets, ReadOnlySpan<int> candidates, ContactManifold manifold,
        int namespace3D, int namespace2D)
    {
        PhysicsMesh mesh = collider.Mesh;
        CollisionSatScratch scratch = collider.Context.CollisionScratch;
        if (mesh.Mode == MeshColliderMode.Convex && mesh.IsClosedSurface && !prismOffsets.IsEmpty
            && TryAppendConvexMeshPrismContacts(collider, embedded!, center, rotation, halfThickness,
                prismOffsets, manifold, namespace3D, namespace2D)) return;
        if (mesh.Mode == MeshColliderMode.Convex && mesh.IsClosedSurface && prismOffsets.IsEmpty
            && ConvexHullCylinderContact.TryGetContactWithContainedCenter(mesh.GeometryFrame,
                mesh.GeometryVertices, mesh.Triangles, center, rotation, halfThickness * Fixed64.Two, radius,
                out FixedContactAnchors contact, Vector2d.Forward, length))
        {
            manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, collider.Material, embedded?.Material ?? collider.Material,
                contact.DepthIsClamped, namespace3D, namespace2D);
            return;
        }
        var section = new FiniteSlabSectionFrame(mesh.GeometryFrame, center, rotation, length, radius, halfThickness);
        scratch.MeshSurfaces.Clear();
        foreach (int triangle in candidates)
        {
            if (!scratch.MeshSurfaces.Add(mesh.GetManifoldSurfaceOwner(triangle))) continue;
            if (!MeshTriangleContactGenerator.GatherMeshSlabSurfaceTriangles(mesh, triangle, section,
                prismOffsets, candidates, scratch.MeshTriangleCandidatesB)) continue;
            mesh.GetGeometryTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            var frame = new FiniteSlabRayFrame(new FixedTriangle(a, b, c), mesh.GeometryFrame,
                center, rotation, length, radius, halfThickness);
            MeshTriangleContactGenerator.AppendMeshSlabSurfaceContacts(collider, embedded, triangle, frame,
                prismOffsets, scratch.MeshTriangleCandidatesB.AsReadOnlySpan(),
                MeshSlabFaceRegions.ContainsSection(mesh, triangle, frame, prismOffsets), manifold, namespace3D, namespace2D);
        }
    }

    private static bool TryAppendConvexMeshPrismContacts(LSMeshCollider collider, LSCollider2D embedded,
        Vector3d center, FixedQuaternion rotation, Fixed64 halfThickness, ReadOnlySpan<Vector2d> offsets,
        ContactManifold manifold, int namespace3D, int namespace2D)
    {
        CollisionSatScratch scratch = collider.Context.CollisionScratch;
        int count = offsets.Length, triangleCount = 12 * count - 12, edgeCount = 6 * count;
        if (scratch.PolyhedronVertices.Length < 2 * count) Array.Resize(ref scratch.PolyhedronVertices, 2 * count);
        if (scratch.PolyhedronTriangles.Length < triangleCount) Array.Resize(ref scratch.PolyhedronTriangles, triangleCount);
        if (scratch.PolyhedronEdges.Length < edgeCount) Array.Resize(ref scratch.PolyhedronEdges, edgeCount);
        Span<Vector3d> vertices = scratch.PolyhedronVertices.AsSpan(0, 2 * count);
        Span<int> triangles = scratch.PolyhedronTriangles.AsSpan(0, triangleCount), edges = scratch.PolyhedronEdges.AsSpan(0, edgeCount);
        FillPrismGeometry(offsets, halfThickness, vertices, triangles, edges);
        return MeshTriangleContactGenerator.TryAppendHullContacts(collider, ScaledRigidFrame3d.Rigid(center, rotation), center, vertices,
            triangles, edges, edges[..(2 * count + 2)], embedded.Material, manifold, false, namespace3D, namespace2D);
    }

    internal static void FillPrismGeometry(ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness,
        Span<Vector3d> vertices, Span<int> triangles, Span<int> edges)
    {
        int count = offsets.Length;
        int face = 0;
        for (int index = 0; index < count; index++)
        {
            int bottom = 2 * index, next = 2 * ((index + 1) % count);
            vertices[bottom] = new(offsets[index].X, -halfThickness, offsets[index].Y);
            vertices[bottom + 1] = new(offsets[index].X, halfThickness, offsets[index].Y);
            triangles[face++] = bottom; triangles[face++] = next; triangles[face++] = next + 1;
            triangles[face++] = bottom; triangles[face++] = next + 1; triangles[face++] = bottom + 1;
            int edge = 2 * index;
            edges[edge] = bottom; edges[edge + 1] = next;
            edges[2 * count + edge] = bottom; edges[2 * count + edge + 1] = bottom + 1;
            edges[4 * count + edge] = bottom + 1; edges[4 * count + edge + 1] = next + 1;
            if (index == 0 || index == count - 1) continue;
            triangles[face++] = 0; triangles[face++] = bottom; triangles[face++] = next;
            triangles[face++] = 1; triangles[face++] = next + 1; triangles[face++] = bottom + 1;
        }
    }
}
