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
    internal static bool IntersectsForContinuousCollision(LSCollider collider3D, LSCollider2D embedded)
    {
        if (collider3D is not LSMeshCollider mesh
            || embedded is not (LSCircleCollider2D or LSCapsuleCollider2D or LSAABBoxCollider2D or LSPolygonCollider2D))
            return TryBuildManifold(collider3D, embedded, collider3D.Context.CollisionScratch.MixedManifold);
        if (!mesh.Bounds.Intersects(embedded.MixedBounds3D)) return false;
        SwiftList<int> candidates = mesh.Context.CollisionScratch.MeshTriangleCandidatesA;
        mesh.GetTrianglesInBounds(new FixedBoundVolume(embedded.MixedBounds3D.Min, embedded.MixedBounds3D.Max), candidates);
        Fixed64 length = embedded is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero;
        Fixed64 radius = embedded is LSCircleCollider2D circle ? circle.ScaledRadius
            : embedded is LSCapsuleCollider2D stadium ? stadium.ScaledRadius : Fixed64.Zero;
        Fixed64 yaw = embedded is LSAABBoxCollider2D ? Fixed64.Zero : embedded.Rotation;
        var section = new FiniteSlabSectionFrame(mesh.Mesh.Origin, mesh.Mesh.Rotation,
            GetEmbeddedCenter3D(embedded), FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw),
            length, radius, embedded.MixedHalfThickness);
        Span<Vector2d> box = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = GetConvexPrismOffsets(embedded, box);
        foreach (int triangle in candidates.AsReadOnlySpan())
        {
            mesh.Mesh.GetLocalTriangleVertices(triangle, out Vector3d first, out Vector3d second, out Vector3d third);
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
        SwiftList<int> admitted = scratch.MeshTriangleCandidatesB;
        scratch.MeshSurfaces.Clear();
        var section = new FiniteSlabSectionFrame(mesh.Origin, mesh.Rotation, center, rotation,
            length, radius, halfThickness);
        Span<int> singleton = stackalloc int[1];
        foreach (int triangle in candidates)
        {
            int owner = mesh.GetManifoldSurfaceOwner(triangle);
            if (!scratch.MeshSurfaces.Add(owner)) continue;
            admitted.FastClear();
            singleton[0] = triangle;
            ReadOnlySpan<int> domain = mesh.GetCoplanarTriangleNeighbors(triangle).IsEmpty ? singleton : candidates;
            foreach (int candidate in domain)
            {
                if (mesh.GetManifoldSurfaceOwner(candidate) != owner) continue;
                mesh.GetLocalTriangleVertices(candidate, out Vector3d first, out Vector3d second, out Vector3d third);
                if (section.IntersectsTriangle(new FixedTriangle(first, second, third), prismOffsets)) admitted.Add(candidate);
            }
            if (admitted.Count == 0) continue;
            MeshSurfaceConnectivity connectivity = scratch.MeshConnectivity;
            connectivity.BuildSlab(mesh, triangle, section, embedded, admitted.AsReadOnlySpan());
            mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            var frame = new FiniteSlabRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
                center, rotation, length, radius, halfThickness);
            MeshSlabFaceRegions faces = scratch.MeshSlabFaces;
            faces.Prepare(frame, prismOffsets);
            int contactCount = manifold.Count;
            scratch.MeshSlabBoundary.BuildSurfaceContacts(collider, embedded, triangle,
                connectivity, faces, manifold, namespace3D, namespace2D);
            faces.Build(mesh, triangle, connectivity, prismOffsets, embedded);
            faces.AppendContacts(collider, embedded, triangle, manifold, namespace3D, namespace2D);
            if (manifold.Count == contactCount)
                scratch.MeshSlabBoundary.AppendSupportFallback(collider, embedded, triangle, faces,
                    manifold, namespace3D, namespace2D);
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
        return MeshTriangleContactGenerator.TryAppendHullContacts(collider, center, rotation, vertices,
            triangles, edges, edges[..(2 * count + 2)], embedded.Material, manifold, false, namespace3D, namespace2D);
    }
}
