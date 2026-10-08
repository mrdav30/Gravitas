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

        Vector3d normal = signedDistance < Fixed64.Zero
            ? -outwardNormal
            : outwardNormal;
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

        if (TryFindMeshConeTriangleContact(
                mesh,
                cone,
                pair.Context.CollisionScratch,
                out ContactAnchor meshAnchor,
                out ContactAnchor coneAnchor,
                out Vector3d normalMeshToCone,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            pair.Manifold.SetContact(
                meshAnchor,
                coneAnchor,
                depth,
                normalMeshToCone,
                depthIsClamped);
            return true;
        }

        if (mesh.Mode == MeshColliderMode.Concave || !ConvexColliderSupport.Intersects(mesh, cone))
            return false;

        normalMeshToCone = ResolveNormal(cone.Center - mesh.Center);
        FixedPointAnchor pointOnMesh = ConvexColliderSupport.GetSupportAnchor(
            mesh,
            normalMeshToCone,
            Vector3d.Zero);
        FixedPointAnchor pointOnCone = ConvexColliderSupport.GetSupportAnchor(
            cone,
            -normalMeshToCone,
            Vector3d.Zero);
        depth = pointOnMesh.ProjectNonNegativeOffsetFrom(
            pointOnCone,
            normalMeshToCone);

        pair.Manifold.SetContact(
            new ContactAnchor(pointOnMesh),
            new ContactAnchor(pointOnCone),
            depth,
            normalMeshToCone);
        return true;
    }

    private static bool TryFindMeshConeTriangleContact(
        LSMeshCollider mesh,
        LSConeCollider cone,
        CollisionSatScratch scratch,
        out ContactAnchor meshAnchor,
        out ContactAnchor coneAnchor,
        out Vector3d normalMeshToCone,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        meshAnchor = default;
        coneAnchor = default;
        normalMeshToCone = Vector3d.Zero;
        depth = Fixed64.Zero;
        depthIsClamped = false;

        var triangleBuffer = scratch.MeshTriangleCandidatesA;
        mesh.GetTrianglesInBounds(new FixedBoundVolume(cone.BoundsMin, cone.BoundsMax), triangleBuffer);
        PrepareMeshConePatchContacts(mesh, cone, scratch);
        bool found = false;
        Fixed64 bestDepth = Fixed64.MaxValue;

        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            int triangleIndex = triangleBuffer[i];
            mesh.Mesh.GetLocalTriangleVertices(
                triangleIndex,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            var triangle = new FixedTriangle(first, second, third);
            // Admission and the complete contact must belong to the same
            // feature; a nearest-center sample can miss a side intersection.
            if (!scratch.MeshConePatchContacts.TryGetValue(
                    mesh.Mesh.GetCoplanarPatchId(triangleIndex), out FixedContactAnchors contact)
                && !triangle.TryGetCenteredFiniteConeContact(
                    mesh.Mesh.Origin, mesh.Mesh.Rotation,
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius,
                    out contact))
            {
                continue;
            }

            if (found && contact.Depth >= bestDepth)
                continue;

            found = true;
            bestDepth = contact.Depth;
            meshAnchor = new ContactAnchor(contact.FirstAnchor);
            coneAnchor = new ContactAnchor(contact.SecondAnchor);
            normalMeshToCone = contact.Normal;
            depth = contact.Depth;
            depthIsClamped = contact.DepthIsClamped;
        }

        return found;
    }

    private static void PrepareMeshConePatchContacts(
        LSMeshCollider mesh, LSConeCollider cone, CollisionSatScratch scratch)
    {
        var contacts = scratch.MeshConePatchContacts;
        contacts.Clear();
        var candidates = scratch.MeshTriangleCandidatesA;
        Span<Vector3d> witnessBounds = stackalloc Vector3d[2]
        {
            mesh.Mesh.ScaledLocalBounds.Min, mesh.Mesh.ScaledLocalBounds.Max
        };
        for (int i = 0; i < candidates.Count; i++)
        {
            int triangleIndex = candidates[i];
            int patch = mesh.Mesh.GetCoplanarPatchId(triangleIndex);
            if (patch < 0 || contacts.ContainsKey(patch))
                continue;
            mesh.Mesh.GetLocalTriangleVertices(triangleIndex,
                out Vector3d first, out Vector3d second, out Vector3d third);
            var triangle = new FixedTriangle(first, second, third);
            // A later BVH triangle may contain the certified face witness.
            // Certify first, then reduce in the original BVH order so an earlier
            // internal-edge exit cannot defeat the whole patch's face exit.
            if (TriangleConeContact.TryGetPatchFaceContact(
                    triangle, mesh.Mesh.Origin, mesh.Mesh.Rotation,
                    mesh.Mesh.ScaledLocalVertices, mesh.Mesh.GetCoplanarPatchBoundaryVertexPairs(triangleIndex),
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius,
                    out FixedContactAnchors contact, witnessBounds))
            {
                contacts.Add(patch, contact);
                continue;
            }

            ReadOnlySpan<int> corners = mesh.Mesh.GetConvexCoplanarPatchCornerVertexIndices(triangleIndex);
            // A convex patch's normal fan consists of its real perimeter
            // edges and corners. An intersecting authored seed supplies a
            // valid base-pole witness when no minimum-face proof applies.
            if (!corners.IsEmpty
                && triangle.TryGetCenteredFiniteConeContact(mesh.Mesh.Origin, mesh.Mesh.Rotation,
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, out _))
            {
                bool found = TriangleConeContact.TryGetConvexPatchContact(triangle, mesh.Mesh.Origin, mesh.Mesh.Rotation,
                    mesh.Mesh.ScaledLocalVertices, corners,
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius,
                    out contact, witnessBounds);
                // The admitted triangle is a subset of this exact filled
                // convex patch, so the complete polygon cannot be separated.
                System.Diagnostics.Debug.Assert(found);
                contacts.Add(patch, contact);
            }
        }
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
