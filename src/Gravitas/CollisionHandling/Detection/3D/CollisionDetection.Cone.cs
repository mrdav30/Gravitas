//=======================================================================
// CollisionDetection.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections.Query;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    private static bool DoConeSphereCheck(CollisionWorkItem pair)
    {
        var cone = (LSConeCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        if (!FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                sphere.Center,
                cone.Center,
                cone.Rotation,
                Vector3d.Up,
                cone.Height,
                cone.ScaledRadius,
                Vector3d.Right,
                out FixedPointAnchor coneAnchor,
                out Vector3d outwardNormal,
                out Fixed64 signedDistance))
        {
            return false;
        }
        if (signedDistance > sphere.ScaledRadius)
            return false;

        // Containment increases penetration depth; the sphere still escapes
        // along the solid surface's outward normal, not toward the cone center.
        Vector3d normal = outwardNormal;
        ResolvePenetrationDepth(
            sphere.ScaledRadius,
            signedDistance,
            out Fixed64 penetrationDepth,
            out bool depthIsClamped);
        pair.Manifold.SetContact(
            new ContactAnchor(coneAnchor),
            new ContactAnchor(
                FixedSegment.GetCenteredCapsuleSupportAnchor(
                    sphere.Center,
                    sphere.Rotation,
                    Fixed64.Zero,
                    sphere.ScaledRadius,
                    -normal)),
            penetrationDepth,
            normal,
            depthIsClamped);

        return true;
    }

    private static bool DoConeConvexCheck(CollisionWorkItem pair)
    {
        GetConeConvexColliders(pair, out LSConeCollider cone, out LSCollider convex);

        if (!ConvexColliderSupport.Intersects(cone, convex))
            return false;

        Vector3d normalConeToConvex = ResolveNormal(convex.Center - cone.Center);
        normalConeToConvex = normalConeToConvex.Normalized;
        FixedPointAnchor pointOnCone = ConvexColliderSupport.GetSupportAnchor(
            cone,
            normalConeToConvex,
            Vector3d.Zero);
        FixedPointAnchor pointOnConvex = ConvexColliderSupport.GetSupportAnchor(
            convex,
            -normalConeToConvex,
            Vector3d.Zero);
        Fixed64 depth = pointOnCone.ProjectNonNegativeOffsetFrom(
            pointOnConvex,
            normalConeToConvex);

        SetContactInPairOrder(
            pair,
            cone,
            new ContactAnchor(pointOnCone),
            convex,
            new ContactAnchor(pointOnConvex),
            depth,
            normalConeToConvex,
            depthIsClamped: false);
        return true;
    }

    private static bool DoMeshConeCheck(CollisionWorkItem pair)
    {
        var mesh = (LSMeshCollider)pair.ColliderA;
        var cone = (LSConeCollider)pair.ColliderB;

        if (BuildMeshConeSurfaceContacts(mesh, cone, pair.Context.CollisionScratch, pair.Manifold))
            return true;

        // An open sheet has no solid interior, even when its bounds were
        // authored in convex mode. Only a closed convex volume owns containment.
        if (mesh.Mode == MeshColliderMode.Concave || !mesh.IsClosedSurface
            || !ConvexColliderSupport.Intersects(mesh, cone))
            return false;

        Vector3d normalMeshToCone = ResolveNormal(cone.Center - mesh.Center);
        FixedPointAnchor pointOnMesh = ConvexColliderSupport.GetSupportAnchor(
            mesh,
            normalMeshToCone,
            Vector3d.Zero);
        FixedPointAnchor pointOnCone = ConvexColliderSupport.GetSupportAnchor(
            cone,
            -normalMeshToCone,
            Vector3d.Zero);
        Fixed64 depth = pointOnMesh.ProjectNonNegativeOffsetFrom(
            pointOnCone,
            normalMeshToCone);

        pair.Manifold.SetContact(
            new ContactAnchor(pointOnMesh),
            new ContactAnchor(pointOnCone),
            depth,
            normalMeshToCone);
        return true;
    }

    private static bool BuildMeshConeSurfaceContacts(
        LSMeshCollider meshCollider, LSConeCollider cone, CollisionSatScratch scratch, ContactManifold manifold)
    {
        PhysicsMesh mesh = meshCollider.Mesh;
        var candidates = scratch.MeshTriangleCandidatesA;
        var admitted = scratch.MeshTriangleCandidatesB;
        meshCollider.GetTrianglesInBounds(new FixedBoundVolume(cone.BoundsMin, cone.BoundsMax), candidates);
        scratch.MeshConeSurfaces.Clear();
        ContactManifold staged = scratch.MeshConeManifold;
        staged.Reset();
        foreach (int triangleIndex in candidates.AsReadOnlySpan())
        {
            int owner = mesh.GetManifoldSurfaceOwner(triangleIndex);
            if (!scratch.MeshConeSurfaces.Add(owner)) continue;
            mesh.GetLocalTriangleVertices(triangleIndex, out Vector3d a, out Vector3d b, out Vector3d c);
            var frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
                cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius);
            admitted.FastClear();
            // Reuse one exact plane frame per surface rather than rebuilding
            // its reduced normal and rigid basis for each subdivided triangle.
            // ponytail: O(candidate count * surface count) owner comparisons;
            // bucket candidates if many distinct planes become a measured cost.
            foreach (int candidate in candidates.AsReadOnlySpan())
            {
                if (mesh.GetManifoldSurfaceOwner(candidate) != owner) continue;
                mesh.GetLocalTriangleVertices(candidate, out a, out b, out c);
                if (ConePlaneRayEvents.IntersectsTriangle(frame, new FixedTriangle(a, b, c))) admitted.Add(candidate);
            }
            if (admitted.Count == 0) continue;
            var connectivity = scratch.MeshConeConnectivity;
            connectivity.Build(mesh, triangleIndex, cone.Center, cone.Rotation,
                cone.Height, cone.ScaledRadius, admitted.AsReadOnlySpan());
            var faces = scratch.MeshConeFaces;
            faces.Build(mesh, triangleIndex, cone.Center, cone.Rotation,
                cone.Height, cone.ScaledRadius, connectivity, admitted.AsReadOnlySpan());
            int surface = mesh.GetCanonicalSurfaceOrdinal(triangleIndex);
            for (int region = 0; region < faces.RegionCount; region++)
            {
                var key = new ContactGroupKey(0, 0, surface, 0, region);
                Vector3d normal = faces.GetNormal(region);
                int count = faces.GetSampleCount(region);
                for (int sample = 0; sample < count; sample++)
                {
                    bool found = faces.TryGetSample(region, sample, out Vector3d p, out Vector3d q, out Fixed64 depth);
                    System.Diagnostics.Debug.Assert(found);
                    staged.AddContact(ContactAnchor.FromWorldPoint(p), ContactAnchor.FromWorldPoint(q),
                        depth, normal, meshCollider.Material, cone.Material, group: key,
                        contactIdentity: faces.GetSampleIdentity(region, sample));
                }
            }
            // Finite exposed-feature witnesses lie in the cone. Reuse the
            // face owner's exact boundary admission rather than constructing
            // support candidates for an entirely disjoint perimeter.
            if (faces.HasBoundaryIntersection)
                scratch.MeshConeBoundary.BuildSurfaceContacts(meshCollider, cone, triangleIndex, connectivity, faces, staged);
        }
        // Geometry/range failures leave the pair unpublished. Reserve the
        // actual grouped result before copying; both owners retain high water.
        manifold.ReserveGroups(staged.GroupCount);
        for (int group = 0; group < staged.GroupCount; group++)
        {
            ref ContactGroup source = ref staged.GetGroup(group);
            for (int sample = 0; sample < source.Count; sample++)
                manifold.AddContact(source.Key, source[sample]);
        }
        return staged.HasContact;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetConeConvexColliders(
        CollisionWorkItem pair,
        out LSConeCollider cone,
        out LSCollider convex)
    {
        if (pair.ColliderA is LSConeCollider coneA && ConvexColliderSupport.IsSupported(pair.ColliderB))
        {
            cone = coneA;
            convex = pair.ColliderB;
            return;
        }

        cone = (LSConeCollider)pair.ColliderB;
        convex = pair.ColliderA;
    }

}
