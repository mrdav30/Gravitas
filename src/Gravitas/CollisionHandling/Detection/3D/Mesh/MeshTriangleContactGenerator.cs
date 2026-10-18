//=======================================================================
// MeshTriangleContactGenerator.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Builds finite mesh manifolds, retaining closed convex solids where available
/// and using triangle contacts for the remaining surface owners.
/// </summary>
internal static class MeshTriangleContactGenerator
{
    internal readonly struct HullSample : IPlaneContactSample
    {
        internal HullSample(FixedContactAnchors contact) => Contact = contact;
        internal readonly FixedContactAnchors Contact;
        public Vector3d Point => Contact.FirstAnchor.LocalPoint;
    }

    private static readonly IComparer<HullSample> HullSampleComparer = Comparer<HullSample>.Create(static (a, b) =>
    {
        int order = a.Contact.FirstAnchor.CompareLocalFeature(b.Contact.FirstAnchor);
        return order != 0 ? order : a.Contact.SecondAnchor.CompareLocalFeature(b.Contact.SecondAnchor);
    });

    // True means the convex pair was handled, including separation; false requests finite triangle fallback.
    internal static bool TryAppendHullContacts(LSMeshCollider hull, Vector3d origin, FixedQuaternion rotation,
        ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> triangles, ReadOnlySpan<int> edges,
        ReadOnlySpan<int> satEdges, PhysicsMaterial material, ContactManifold manifold, bool reverse,
        int namespaceA = 0, int namespaceB = 0, ContactGroupKey group = default)
    {
        PhysicsMesh mesh = hull.Mesh;
        // Select the solid exit once; finite witnesses must belong to that support.
        if (!WideOrientedBox.TryGetConvexHullPenetration(mesh.Origin, mesh.Rotation, mesh.ScaledLocalVertices,
            mesh.Triangles, mesh.ConvexSatEdgeVertexPairs, origin, rotation, vertices, triangles, satEdges,
            out WidePointSpanPenetration penetration, skipParallelDirections: true)) return true;
        SwiftList<HullSample> samples = hull.Context.CollisionScratch.HullSamples;
        samples.FastClear();
        WideOrientedBox.VisitConvexHullContacts(mesh.Origin, mesh.Rotation, mesh.ScaledLocalVertices,
            mesh.Triangles, mesh.ConvexSatEdgeVertexPairs, origin, rotation, vertices, triangles, edges,
            penetration, samples, static (pool, contact) => pool.Add(new HullSample(contact)));
        if (samples.Count == 0) return false;
        samples.SortInPlace(HullSampleComparer);
        Span<HullSample> pool = samples.AsSpan();
        int unique = 1;
        for (int index = 1; index < pool.Length; index++)
            if (pool[index].Contact.FirstAnchor.CompareLocalFeature(pool[unique - 1].Contact.FirstAnchor) != 0)
                pool[unique++] = pool[index];
        pool = pool[..unique];
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        int count = Math.Min(pool.Length, chosen.Length);
        for (int slot = 0; slot < count; slot++) chosen[slot] = slot;
        if (pool.Length > chosen.Length)
        {
            Vector3d normal = pool[0].Contact.Normal;
            var axis = new WideAxis3(Signed320.ExtendValue(Signed192.Raw(normal.X)),
                Signed320.ExtendValue(Signed192.Raw(normal.Y)), Signed320.ExtendValue(Signed192.Raw(normal.Z)));
            // Coverage ranks final representable samples; admission and witnesses stay exact.
            // A 33-bit solver normal times the normalized basis fits well below 192 bits.
            WideRigidProjection.GetLocalAxisProjections(axis, new WideRationalBasis3d(mesh.Rotation),
                out Signed576 x, out Signed576 y, out Signed576 z);
            var metrics = new WidePlaneMetrics(new WideAxis3(Signed320.NarrowValue(x),
                Signed320.NarrowValue(y), Signed320.NarrowValue(z)));
            for (int slot = 1; slot < count; slot++)
                chosen[slot] = MeshPlaneSampleReduction.ChooseNext<HullSample>(pool, chosen[..slot], metrics, HullSampleComparer);
            // A support line needs only its two extreme velocity levers.
            if (metrics.GetAreaSquared(pool[0].Point, pool[chosen[1]].Point, pool[chosen[2]].Point).IsZero) count = 2;
        }
        for (int slot = 0; slot < count; slot++)
        {
            FixedContactAnchors contact = pool[chosen[slot]].Contact;
            manifold.AddContact(new ContactAnchor(reverse ? contact.SecondAnchor : contact.FirstAnchor),
                new ContactAnchor(reverse ? contact.FirstAnchor : contact.SecondAnchor), contact.Depth,
                reverse ? -contact.Normal : contact.Normal, reverse ? material : hull.Material,
                reverse ? hull.Material : material, contact.DepthIsClamped, namespaceA, namespaceB, group);
        }
        return true;
    }

    public static bool TryBuildMeshSphereManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSSphereCollider sphere,
        SwiftList<int> triangleBuffer)
    {
        return TryBuildRoundedAxisManifold(pair, mesh, sphere.Center, sphere.Rotation,
            Fixed64.Zero, sphere.ScaledRadius, CreateBounds(sphere.BoundsMin, sphere.BoundsMax), triangleBuffer);
    }

    public static bool TryBuildMeshCapsuleManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCapsuleCollider capsule,
        SwiftList<int> triangleBuffer)
    {
        return TryBuildRoundedAxisManifold(pair, mesh, capsule.Center, capsule.Rotation,
            capsule.AxisLength, capsule.ScaledRadius, CreateBounds(capsule.BoundsMin, capsule.BoundsMax), triangleBuffer);
    }

    private static bool BuildJoinedRoundedManifold(CollisionWorkItem pair, LSMeshCollider collider,
        StaticSurface surface, Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius)
    {
        surface.BuildRounded(center, rotation, coreLength, radius);
        MeshCapsuleFaceRegions faces = surface.Faces;
        for (int region = 0; region < faces.RegionCount; region++)
            for (int sample = 0; sample < faces.GetSampleCount(region); sample++)
            {
                bool found = faces.TryGetJoinedSample(region, sample, out FixedContactAnchors contact, out LSMeshCollider? supplier);
                System.Diagnostics.Debug.Assert(found);
                // Selected rows belong to their supplier's pair; other overlapping members still raise events.
                if (supplier != collider) continue;
                pair.Manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                    contact.Depth, contact.Normal, collider.Material, pair.ColliderB.Material, contact.DepthIsClamped,
                    group: new ContactGroupKey(0, 0, 0, 0, region));
            }
        return faces.AppendJoinedBoundaries(collider, pair.ColliderB.Material, center, rotation, coreLength,
            radius, pair.Context.CollisionScratch.MeshCapsuleBoundary, pair.Manifold);
    }

    private static bool TryBuildRoundedAxisManifold(CollisionWorkItem pair, LSMeshCollider collider,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        FixedBoundVolume bounds, SwiftList<int> candidates)
    {
        PhysicsMesh mesh = collider.Mesh;
        // Zero-core capsules are spheres: body spin must not rotate their
        // geometry frame and introduce a rounded off-center impulse lever.
        if (coreLength == Fixed64.Zero) rotation = FixedQuaternion.Identity;
        if (collider.StaticSurface is { } staticSurface && !collider.IsTrigger && !pair.ColliderB.IsTrigger
            && staticSurface.Gather(pair.ColliderB, bounds))
            return BuildJoinedRoundedManifold(pair, collider, staticSurface, center, rotation, coreLength, radius);
        collider.GetTrianglesInBounds(bounds, candidates);
        CollisionSatScratch scratch = pair.Context.CollisionScratch;
        SwiftList<int> admitted = scratch.MeshTriangleCandidatesB;
        scratch.MeshSurfaces.Clear();
        var frame = new CapsuleFiniteSectionFrame(mesh.Origin, mesh.Rotation, center, rotation, coreLength, radius);
        Span<int> singleton = stackalloc int[1];
        foreach (int triangle in candidates.AsReadOnlySpan())
        {
            int owner = mesh.GetManifoldSurfaceOwner(triangle);
            if (!scratch.MeshSurfaces.Add(owner)) continue;
            admitted.FastClear();
            // A trusted owner with no shared-edge neighbor is one triangle.
            // Individually owned facets therefore retain linear gathering.
            singleton[0] = triangle;
            ReadOnlySpan<int> surfaceCandidates = mesh.GetCoplanarTriangleNeighbors(triangle).IsEmpty
                ? singleton : candidates.AsReadOnlySpan();
            foreach (int candidate in surfaceCandidates)
            {
                if (mesh.GetManifoldSurfaceOwner(candidate) != owner) continue;
                mesh.GetLocalTriangleVertices(candidate, out Vector3d a, out Vector3d b, out Vector3d c);
                if (CapsulePlaneRayEvents.IntersectsTriangle(frame, new FixedTriangle(a, b, c))) admitted.Add(candidate);
            }
            if (admitted.Count == 0) continue;
            MeshSurfaceConnectivity connectivity = scratch.MeshConnectivity;
            connectivity.BuildCapsule(mesh, triangle, frame, admitted.AsReadOnlySpan());
            MeshCapsuleFaceRegions faces = scratch.MeshCapsuleFaces;
            faces.Build(mesh, triangle, frame, center, rotation, connectivity);
            int surface = mesh.GetCanonicalSurfaceOrdinal(triangle);
            for (int region = 0; region < faces.RegionCount; region++)
            {
                var group = new ContactGroupKey(0, 0, surface, 0, region);
                int count = faces.GetSampleCount(region);
                for (int sample = 0; sample < count; sample++)
                {
                    bool found = faces.TryGetSample(region, sample, out FixedContactAnchors contact);
                    System.Diagnostics.Debug.Assert(found);
                    pair.Manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                        contact.Depth, contact.Normal, collider.Material, pair.ColliderB.Material,
                        contact.DepthIsClamped, group: group);
                }
            }
            // Boundary constraints retain their own feature direction. They
            // cannot replace or consume the region's finite face sample cap.
            if (faces.HasBoundaryIntersection)
                scratch.MeshCapsuleBoundary.BuildSurfaceContacts(collider, pair.ColliderB.Material,
                    center, rotation, coreLength, radius, triangle, connectivity, faces, pair.Manifold);
        }
        return pair.Manifold.HasContact;
    }

    public static bool TryBuildMeshCuboidManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCuboidCollider cuboid,
        SwiftList<int> triangleBuffer)
    {
        if (mesh.StaticSurface is { } surface && !mesh.IsTrigger && !cuboid.IsTrigger
            && surface.GatherSlab(cuboid))
        {
            Vector3d half = cuboid.OrientedBox.HalfExtents;
            surface.BuildSlab(cuboid, Fixed64.Zero, half.Y, new(half.X, half.Z));
            return surface.SlabFaces.AppendJoinedContacts(mesh, cuboid, pair.Context.CollisionScratch.MeshSlabBoundary, pair.Manifold);
        }
        mesh.GetTrianglesInBounds(CreateBounds(cuboid.BoundsMin, cuboid.BoundsMax), triangleBuffer);
        for (int i = 0; i < triangleBuffer.Count; i++)
            TryAddCuboidTriangleContact(pair, mesh, triangleBuffer[i], cuboid);

        return pair.Manifold.HasContact;
    }

    public static bool TryBuildMeshCylinderManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCylinderCollider cylinder,
        SwiftList<int> triangleBuffer)
    {
        if (mesh.StaticSurface is { } surface && !mesh.IsTrigger && !cylinder.IsTrigger
            && surface.GatherSlab(cylinder))
        {
            surface.BuildSlab(cylinder, cylinder.ScaledRadius, cylinder.Height, default);
            return surface.SlabFaces.AppendJoinedContacts(mesh, cylinder, pair.Context.CollisionScratch.MeshSlabBoundary, pair.Manifold);
        }
        mesh.GetTrianglesInBounds(CreateBounds(cylinder.BoundsMin, cylinder.BoundsMax), triangleBuffer);
        bool overlaps = false;
        for (int i = 0; i < triangleBuffer.Count; i++)
            overlaps |= TryAddCylinderTriangleContact(pair, mesh, triangleBuffer[i], cylinder);

        return overlaps;
    }

    public static bool TryBuildMeshMeshManifold(
        CollisionWorkItem pair,
        LSMeshCollider meshA,
        LSMeshCollider meshB,
        SwiftList<int> triangleBufferA,
        SwiftList<int> triangleBufferB)
    {
        CollisionSatScratch scratch = pair.Context.CollisionScratch;
        scratch.MeshSurfaces.Clear();
        LSMeshCollider? surface = meshA.Mode == MeshColliderMode.Convex && meshA.Mesh.IsClosedSurface
            && !meshB.Mesh.IsClosedSurface ? meshB
            : meshB.Mode == MeshColliderMode.Convex && meshB.Mesh.IsClosedSurface
                && !meshA.Mesh.IsClosedSurface ? meshA : null;
        if (surface != null)
        {
            LSMeshCollider hull = surface == meshA ? meshB : meshA;
            surface.GetTrianglesInBounds(CreateBounds(hull.BoundsMin, hull.BoundsMax), triangleBufferA);
            foreach (int triangle in triangleBufferA.AsReadOnlySpan())
            {
                int owner = surface.Mesh.GetManifoldSurfaceOwner(triangle);
                if (!scratch.MeshSurfaces.Add(owner)) continue;
                int ordinal = surface.Mesh.GetCanonicalSurfaceOrdinal(triangle);
                var group = new ContactGroupKey(0, 0, surface == meshA ? ordinal : 0, surface == meshB ? ordinal : 0);
                ReadOnlySpan<int> corners = surface.Mesh.GetConvexCoplanarPatchCornerVertexIndices(triangle);
                if (!corners.IsEmpty)
                {
                    int count = corners.Length;
                    if (scratch.PolyhedronVertices.Length < count) Array.Resize(ref scratch.PolyhedronVertices, count);
                    if (scratch.PolyhedronTriangles.Length < 3 * (count - 2)) Array.Resize(ref scratch.PolyhedronTriangles, 3 * (count - 2));
                    if (scratch.PolyhedronEdges.Length < 2 * count) Array.Resize(ref scratch.PolyhedronEdges, 2 * count);
                    Span<Vector3d> vertices = scratch.PolyhedronVertices.AsSpan(0, count);
                    Span<int> triangles = scratch.PolyhedronTriangles.AsSpan(0, 3 * (count - 2));
                    Span<int> edges = scratch.PolyhedronEdges.AsSpan(0, 2 * count);
                    for (int index = 0; index < count; index++)
                    {
                        vertices[index] = surface.Mesh.ScaledLocalVertices[corners[index]];
                        edges[2 * index] = index;
                        edges[2 * index + 1] = (index + 1) % count;
                        if (index < 2) continue;
                        int face = 3 * (index - 2);
                        triangles[face] = 0; triangles[face + 1] = index - 1; triangles[face + 2] = index;
                    }
                    // Use the trusted finite patch once, not its artificial triangle diagonals.
                    if (TryAppendHullContacts(hull, surface.Mesh.Origin, surface.Mesh.Rotation,
                        vertices, triangles, edges, edges, surface.Material, pair.Manifold, hull == meshB, group: group))
                        continue;
                }
                foreach (int candidate in triangleBufferA.AsReadOnlySpan())
                    if (surface.Mesh.GetManifoldSurfaceOwner(candidate) == owner)
                        AppendMeshTriangleContacts(pair, surface, candidate, hull, triangleBufferB,
                            surface == meshB, group);
            }
            return pair.Manifold.HasContact;
        }
        bool reverseContact = meshA.Id > meshB.Id;
        LSMeshCollider firstMesh = reverseContact ? meshB : meshA;
        LSMeshCollider secondMesh = reverseContact ? meshA : meshB;
        firstMesh.GetTrianglesInBounds(
            CreateBounds(secondMesh.BoundsMin, secondMesh.BoundsMax),
            triangleBufferA);
        for (int i = 0; i < triangleBufferA.Count; i++)
            AppendMeshTriangleContacts(pair, firstMesh, triangleBufferA[i], secondMesh, triangleBufferB, reverseContact);

        return pair.Manifold.HasContact;
    }

    private static void AppendMeshTriangleContacts(CollisionWorkItem pair, LSMeshCollider firstMesh,
        int triangle, LSMeshCollider secondMesh, SwiftList<int> candidates, bool reverse,
        ContactGroupKey? group = null)
    {
        firstMesh.Mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var firstTriangle = new FixedTriangle(a, b, c);
        GetTriangleBoundsInFrame(firstTriangle.Bounds, firstMesh.Mesh.Origin, firstMesh.Mesh.Rotation,
            secondMesh.Mesh.Origin, secondMesh.Mesh.Rotation, out FixedBoundVolume bounds);
        secondMesh.Mesh.GetTrianglesInLocalBounds(bounds, candidates);
        bool canonicalReverse = firstMesh.Id > secondMesh.Id;
        for (int j = 0; j < candidates.Count; j++)
        {
            secondMesh.Mesh.GetLocalTriangleVertices(candidates[j], out a, out b, out c);
            var secondTriangle = new FixedTriangle(a, b, c);
            FixedContactAnchors contact;
            bool intersects = canonicalReverse
                ? secondTriangle.TryGetContact(secondMesh.Mesh.Origin, secondMesh.Mesh.Rotation,
                    firstMesh.Mesh.Origin, firstMesh.Mesh.Rotation, firstTriangle, out contact)
                : firstTriangle.TryGetContact(firstMesh.Mesh.Origin, firstMesh.Mesh.Rotation,
                    secondMesh.Mesh.Origin, secondMesh.Mesh.Rotation, secondTriangle, out contact);
            if (!intersects) continue;

            ContactAnchor first = new(reverse != canonicalReverse ? contact.SecondAnchor : contact.FirstAnchor);
            ContactAnchor second = new(reverse != canonicalReverse ? contact.FirstAnchor : contact.SecondAnchor);
            Vector3d normal = reverse != canonicalReverse ? -contact.Normal : contact.Normal;
            if (group.HasValue)
            {
                pair.Manifold.AddContact(first, second, contact.Depth, normal,
                    pair.ColliderA.Material, pair.ColliderB.Material, contact.DepthIsClamped, group: group.Value);
            }
            else AddContact(pair, first, second, contact.Depth, normal, contact.DepthIsClamped);
        }
    }

    private static void TryAddCuboidTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCuboidCollider cuboid)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        Span<FixedContactLocalPoints> faceContacts =
            stackalloc FixedContactLocalPoints[4];
        if (!cuboid.OrientedBox.TryGetTriangleContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                triangle,
                faceContacts,
                out FixedContactAnchors contact,
                out int faceContactCount))
        {
            return;
        }

        if (faceContactCount > 0)
        {
            for (int index = 0; index < faceContactCount; index++)
            {
                AddReversedBoxTriangleContact(
                    pair,
                    contact,
                    faceContacts[index]);
            }
            return;
        }

        AddReversedBoxTriangleContact(pair, contact);
    }

    private static void AddReversedBoxTriangleContact(
        CollisionWorkItem pair,
        FixedContactAnchors contact)
    {
        AddContact(
            pair,
            new ContactAnchor(contact.SecondAnchor),
            new ContactAnchor(contact.FirstAnchor),
            contact.Depth,
            -contact.Normal,
            contact.DepthIsClamped);
    }

    private static void AddReversedBoxTriangleContact(
        CollisionWorkItem pair,
        FixedContactAnchors primary,
        FixedContactLocalPoints contact)
    {
        AddContact(
            pair,
            new ContactAnchor(
                primary.SecondAnchor.Origin,
                primary.SecondAnchor.Rotation,
                contact.SecondLocalPoint),
            new ContactAnchor(
                primary.FirstAnchor.Origin,
                primary.FirstAnchor.Rotation,
                contact.FirstLocalPoint),
            primary.Depth,
            -primary.Normal,
            primary.DepthIsClamped);
    }

    private static bool TryAddCylinderTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCylinderCollider cylinder)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        if (!triangle.TryGetCenteredFiniteCylinderContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                cylinder.Center,
                cylinder.Rotation,
                cylinder.Height,
                cylinder.ScaledRadius,
                out FixedContactAnchors contact,
                out bool isCapFaceContact))
        {
            return false;
        }

        // Enrichment belongs only to a certified cap/triangle-face winner.
        // Nearly parallel rim contacts must retain their selected witnesses.
        if (isCapFaceContact && TryAddCylinderCapTriangleContacts(
                pair, mesh, triangle, cylinder, contact))
        {
            return true;
        }

        AddContact(
            pair,
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
        return true;
    }

    private static bool TryAddCylinderCapTriangleContacts(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        FixedTriangle triangle,
        LSCylinderCollider cylinder,
        in FixedContactAnchors primary)
    {
        Vector3d normal = primary.Normal;
        CylinderContactGeometry.GetCapBasis(cylinder, out Vector3d tangentA, out Vector3d tangentB);
        // A valid support can be deduplicated or lose the four-contact reduction.
        // Geometric success must not depend on whether the manifold count grew.
        bool found = TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal + tangentA, primary);
        found |= TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal - tangentA, primary);
        found |= TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal + tangentB, primary);
        found |= TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal - tangentB, primary);
        return found || TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal, primary);
    }

    private static bool TryAddCylinderCapTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        FixedTriangle triangle,
        LSCylinderCollider cylinder,
        Vector3d supportDirection,
        in FixedContactAnchors primary)
    {
        if (!triangle.TryGetCenteredFiniteCylinderSupportContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                cylinder.Center,
                cylinder.Rotation,
                cylinder.Height,
                cylinder.ScaledRadius,
                supportDirection,
                primary.Normal,
                out FixedContactAnchors contact))
        {
            return false;
        }

        // Parallel cap/face samples share the selected exact plane gap.
        // The support query rounds into the triangle frame before measuring
        // depth; keep it for anchors only, avoiding a second depth rounding.
        AddContact(
            pair,
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            primary.Depth,
            primary.Normal,
            primary.DepthIsClamped);
        return true;
    }

    private static void GetTriangleBoundsInFrame(
        FixedBoundBox triangleBounds,
        Vector3d sourceOrigin,
        FixedQuaternion sourceRotation,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out FixedBoundVolume bounds)
    {
        FixedBoundBox reframedBounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                sourceOrigin,
                sourceRotation,
                triangleBounds.Min,
                triangleBounds.Max,
                frameOrigin,
                frameRotation);
        bounds = CreateBounds(reframedBounds.Min, reframedBounds.Max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedBoundVolume CreateBounds(Vector3d min, Vector3d max) =>
        new(min, max);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddContact(
        CollisionWorkItem pair,
        ContactAnchor anchorOnFirst,
        ContactAnchor anchorOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond,
        bool depthIsClamped)
    {
        pair.Manifold.AddContact(
            anchorOnFirst,
            anchorOnSecond,
            depth,
            normalFirstToSecond,
            depthIsClamped);
    }

}
