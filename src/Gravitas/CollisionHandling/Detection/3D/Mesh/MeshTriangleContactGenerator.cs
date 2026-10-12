//=======================================================================
// MeshTriangleContactGenerator.cs
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
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Builds deterministic triangle-level manifolds for mesh paths that cannot use
/// whole-shape convex assumptions.
/// </summary>
internal static class MeshTriangleContactGenerator
{
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

    private static bool TryBuildRoundedAxisManifold(CollisionWorkItem pair, LSMeshCollider collider,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        FixedBoundVolume bounds, SwiftList<int> candidates)
    {
        PhysicsMesh mesh = collider.Mesh;
        // Zero-core capsules are spheres: body spin must not rotate their
        // geometry frame and introduce a rounded off-center impulse lever.
        if (coreLength == Fixed64.Zero) rotation = FixedQuaternion.Identity;
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
        bool reverseContact = meshA.Id > meshB.Id;
        LSMeshCollider firstMesh = reverseContact ? meshB : meshA;
        LSMeshCollider secondMesh = reverseContact ? meshA : meshB;
        firstMesh.GetTrianglesInBounds(
            CreateBounds(secondMesh.BoundsMin, secondMesh.BoundsMax),
            triangleBufferA);
        for (int i = 0; i < triangleBufferA.Count; i++)
        {
            int triangleA = triangleBufferA[i];
            firstMesh.Mesh.GetLocalTriangleVertices(
                triangleA,
                out Vector3d firstVertex,
                out Vector3d secondVertex,
                out Vector3d thirdVertex);
            var firstTriangle =
                new FixedTriangle(firstVertex, secondVertex, thirdVertex);
            GetTriangleBoundsInFrame(
                firstTriangle.Bounds,
                firstMesh.Mesh.Origin,
                firstMesh.Mesh.Rotation,
                secondMesh.Mesh.Origin,
                secondMesh.Mesh.Rotation,
                out FixedBoundVolume firstInSecondFrameBounds);
            secondMesh.Mesh.GetTrianglesInLocalBounds(
                firstInSecondFrameBounds,
                triangleBufferB);

            for (int j = 0; j < triangleBufferB.Count; j++)
            {
                int triangleB = triangleBufferB[j];
                secondMesh.Mesh.GetLocalTriangleVertices(
                    triangleB,
                    out Vector3d secondFirstVertex,
                    out Vector3d secondSecondVertex,
                    out Vector3d secondThirdVertex);
                var secondTriangle = new FixedTriangle(
                    secondFirstVertex,
                    secondSecondVertex,
                    secondThirdVertex);
                if (!firstTriangle.TryGetContact(
                        firstMesh.Mesh.Origin,
                        firstMesh.Mesh.Rotation,
                        secondMesh.Mesh.Origin,
                        secondMesh.Mesh.Rotation,
                        secondTriangle,
                        out FixedContactAnchors contact))
                {
                    continue;
                }

                if (reverseContact)
                {
                    AddContact(
                        pair,
                        new ContactAnchor(contact.SecondAnchor),
                        new ContactAnchor(contact.FirstAnchor),
                        contact.Depth,
                        -contact.Normal,
                        contact.DepthIsClamped);
                }
                else
                {
                    AddContact(
                        pair,
                        new ContactAnchor(contact.FirstAnchor),
                        new ContactAnchor(contact.SecondAnchor),
                        contact.Depth,
                        contact.Normal,
                        contact.DepthIsClamped);
                }
            }
        }

        return pair.Manifold.HasContact;
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
