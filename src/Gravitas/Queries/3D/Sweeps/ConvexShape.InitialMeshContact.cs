//=======================================================================
// ConvexShape.InitialMeshContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;
using System.Diagnostics;

namespace Gravitas.Queries;

/// <content>Resolves initial mesh contact witnesses from the selected convex source leaf.</content>
internal readonly partial struct ConvexShape
{
    // Both views describe their initial committed poses. Compound reduction
    // must call this while the actual source and target leaves are retained.
    internal bool TryGetInitialMeshContactNormal(
        in ConvexShape source,
        bool allowHullFaceExit,
        out FixedPointAnchor targetAnchor,
        out Vector3d normal)
    {
        Debug.Assert(_offset == Vector3d.Zero && source._offset == Vector3d.Zero);
        FixedContactAnchors contact;
        if (_kind == ConvexShapeKind.Triangle)
        {
            if (TryGetInitialMeshFaceContactNormal(source, out targetAnchor, out normal))
                return true;
            bool found = TryGetInitialTriangleContact(source,
                new FixedTriangle(_triangleA, _triangleB, _triangleC), out contact);
            targetAnchor = contact.FirstAnchor;
            normal = contact.Normal;
            return found;
        }

        var mesh = (LSMeshCollider)_collider!;
        PhysicsMesh hull = mesh.Mesh;
        bool hasContact;
        switch (source._collider)
        {
            case LSCuboidCollider box:
                hasContact = WideOrientedBox.TryGetConvexHullContact(
                    box.Center, box.Rotation, box.OrientedBox.HalfExtents,
                    hull.Origin, hull.Rotation, hull.ScaledLocalVertices,
                    hull.Triangles, hull.ConvexSatEdgeVertexPairs, out contact);
                targetAnchor = contact.SecondAnchor;
                normal = -contact.Normal;
                return hasContact;
            case LSCapsuleCollider capsule:
                hasContact = TryGetHullCapsuleContact(hull, capsule.Center,
                    capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius, out contact);
                break;
            case LSSphereCollider sphere:
                hasContact = TryGetHullCapsuleContact(hull, sphere.Center,
                    sphere.Rotation, Fixed64.Zero, sphere.ScaledRadius, out contact);
                break;
            case LSMeshCollider sourceMesh:
                PhysicsMesh sourceHull = sourceMesh.Mesh;
                hasContact = FixedConvexHullRelations.TryGetContact(
                    hull.Origin, hull.Rotation, hull.ScaledLocalVertices,
                    hull.Triangles, hull.ConvexSatEdgeVertexPairs,
                    sourceHull.Origin, sourceHull.Rotation, sourceHull.ScaledLocalVertices,
                    sourceHull.Triangles, sourceHull.ConvexSatEdgeVertexPairs, out contact);
                break;
            default:
                if (source._kind == ConvexShapeKind.Sphere)
                {
                    hasContact = TryGetHullCapsuleContact(hull, source._center,
                        FixedQuaternion.Identity, Fixed64.Zero, source._radius, out contact);
                    break;
                }
                // A hull face exit alone cannot certify a tolerant, disjoint contact.
                if (!allowHullFaceExit)
                {
                    targetAnchor = default;
                    normal = default;
                    return false;
                }
                GetInitialHullFaceExit(source, hull, out targetAnchor, out normal);
                return true;
        }
        targetAnchor = contact.FirstAnchor;
        normal = contact.Normal;
        return hasContact;
    }

    /// <summary>Certifies a positive finite face contact before reducing covered triangle witnesses.</summary>
    internal bool TryGetInitialMeshFaceContactNormal(in ConvexShape source,
        out FixedPointAnchor targetAnchor, out Vector3d normal)
    {
        targetAnchor = default;
        normal = default;
        Debug.Assert(_kind == ConvexShapeKind.Triangle);
        PhysicsMesh mesh = _triangleOwner!.Mesh;
        Vector3d planeNormal = GetInitialExitNormal(source, mesh.GetFaceNormalWorld(_triangleIndex));
        FixedPointAnchor support = source.GetSupportAnchor(-planeNormal);
        var triangle = new FixedTriangle(_triangleA, _triangleB, _triangleC);
        if (!TryGetInitialSurfaceProjectionTriangle(mesh, support, ref triangle)
            || !TryGetInitialTriangleContact(source, triangle, out FixedContactAnchors contact))
            return false;

        // A finite neighboring face must actually contact the source; projection
        // alone is insufficient for tilted shapes, and a perimeter touch stays radial.
        var matched = new ConvexShape(_triangleOwner!, _triangleIndex, triangle.A, triangle.B, triangle.C);
        FixedPointAnchor targetSupport = matched.GetSupportAnchor(contact.Normal);
        FixedPointAnchor sourceSupport = source.GetSupportAnchor(-contact.Normal);
        if (WidePointAnchor3d.CompareProjectedOffsets(targetSupport, sourceSupport,
                targetSupport, targetSupport, contact.Normal) <= 0)
            return false;
        targetAnchor = triangle.GetClosestPointAnchor(mesh.Origin, mesh.Rotation, support);
        normal = planeNormal;
        return true;
    }

    private bool TryGetInitialSurfaceProjectionTriangle(
        PhysicsMesh mesh, in FixedPointAnchor support, ref FixedTriangle triangle)
    {
        if (WideTriangleRelations.ContainsProjection(triangle, mesh.Origin, mesh.Rotation, support))
            return true;
        if (mesh.GetCoplanarTriangleNeighbors(_triangleIndex).IsEmpty)
            return false;

        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(_triangleIndex);
        if (!corners.IsEmpty)
        {
            // The prepared convex fill removes subdivision cost without filling holes or notches.
            ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
            for (int corner = 1; corner < corners.Length - 1; corner++)
            {
                triangle = new FixedTriangle(vertices[corners[0]], vertices[corners[corner]],
                    vertices[corners[corner + 1]]);
                if (WideTriangleRelations.ContainsProjection(triangle, mesh.Origin, mesh.Rotation, support))
                    return true;
            }
            return false;
        }

        int owner = mesh.GetManifoldSurfaceOwner(_triangleIndex);
        for (int index = 0; index < mesh.TriangleCount; index++)
        {
            if (index == _triangleIndex || mesh.GetManifoldSurfaceOwner(index) != owner)
                continue;
            mesh.GetLocalTriangleVertices(index, out Vector3d a, out Vector3d b, out Vector3d c);
            triangle = new FixedTriangle(a, b, c);
            if (WideTriangleRelations.ContainsProjection(triangle, mesh.Origin, mesh.Rotation, support))
                return true;
        }
        return false;
    }

    private bool TryGetInitialTriangleContact(in ConvexShape source, FixedTriangle triangle,
        out FixedContactAnchors contact)
    {
        PhysicsMesh targetMesh = _triangleOwner!.Mesh;
        if (source._kind == ConvexShapeKind.Sphere)
            return triangle.TryGetSphereContact(targetMesh.Origin, targetMesh.Rotation,
                source._center, FixedQuaternion.Identity, source._radius, out contact);
        if (source._kind == ConvexShapeKind.CircleSlab)
            return triangle.TryGetCircleSlabContact(targetMesh.Origin, targetMesh.Rotation,
                source._center, Fixed64.Zero, source._halfHeight, source._radius, out contact);

        switch (source._collider)
        {
            case LSSphereCollider sphere:
                return triangle.TryGetSphereContact(targetMesh.Origin, targetMesh.Rotation,
                    sphere.Center, sphere.Rotation, sphere.ScaledRadius, out contact);
            case LSCapsuleCollider capsule:
                Vector3d fallback = GetInitialExitNormal(source,
                    targetMesh.GetFaceNormalWorld(_triangleIndex));
                return triangle.TryGetCenteredCapsuleContact(targetMesh.Origin, targetMesh.Rotation,
                    capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                    fallback, out contact);
            case LSCuboidCollider box:
                bool found = box.OrientedBox.TryGetTriangleContact(targetMesh.Origin,
                    targetMesh.Rotation, triangle, out FixedContactAnchors reversed);
                contact = new FixedContactAnchors(reversed.SecondAnchor, reversed.FirstAnchor,
                    -reversed.Normal, reversed.Depth, reversed.DepthIsClamped);
                return found;
            case LSCylinderCollider cylinder:
                return triangle.TryGetCenteredFiniteCylinderContact(targetMesh.Origin, targetMesh.Rotation,
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius, out contact);
            case LSConeCollider cone:
                return triangle.TryGetCenteredFiniteConeContact(targetMesh.Origin, targetMesh.Rotation,
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, out contact);
            default:
                var mesh = (LSMeshCollider)source._collider!;
                PhysicsMesh sourceHull = mesh.Mesh;
                ReadOnlySpan<Vector3d> vertices = stackalloc Vector3d[3]
                    { triangle.A, triangle.B, triangle.C };
                ReadOnlySpan<int> triangles = stackalloc int[3] { 0, 1, 2 };
                ReadOnlySpan<int> edges = stackalloc int[6] { 0, 1, 1, 2, 2, 0 };
                return FixedConvexHullRelations.TryGetContact(
                    targetMesh.Origin, targetMesh.Rotation, vertices, triangles, edges,
                    sourceHull.Origin, sourceHull.Rotation, sourceHull.ScaledLocalVertices,
                    sourceHull.Triangles, sourceHull.ConvexSatEdgeVertexPairs, out contact);
        }
    }

    internal Vector3d GetInitialExitNormal(in ConvexShape source, Vector3d axis)
    {
        FixedPointAnchor targetMax = GetSupportAnchor(axis);
        FixedPointAnchor sourceMin = source.GetSupportAnchor(-axis);
        FixedPointAnchor sourceMax = source.GetSupportAnchor(axis);
        FixedPointAnchor targetMin = GetSupportAnchor(-axis);
        return WidePointAnchor3d.CompareProjectedOffsets(
            targetMax, sourceMin, sourceMax, targetMin, axis) <= 0 ? axis : -axis;
    }

    private void GetInitialHullFaceExit(in ConvexShape source, PhysicsMesh hull,
        out FixedPointAnchor targetAnchor, out Vector3d normal)
    {
        // A two-sided triangle MTV does not describe exit from a solid hull.
        // Rank full-shape exit projections on the cached unit face axes. This
        // preserves hull containment and authored ties without claiming a
        // complete curved-shape/hull MTV (edge axes can provide shorter exits).
        // Committed mesh topology has at least one nondegenerate triangle;
        // scale validation preserves nonzero cached face normals.
        normal = GetInitialExitNormal(source, hull.GetFaceNormalWorld(0));
        targetAnchor = GetSupportAnchor(normal);
        FixedPointAnchor selectedSource = source.GetSupportAnchor(-normal);
        for (int index = 1; index < hull.TriangleCount; index++)
        {
            Vector3d axis = hull.GetFaceNormalWorld(index);
            Debug.Assert(axis != Vector3d.Zero);
            axis = GetInitialExitNormal(source, axis);
            FixedPointAnchor targetSupport = GetSupportAnchor(axis);
            FixedPointAnchor sourceSupport = source.GetSupportAnchor(-axis);
            if (WidePointAnchor3d.CompareProjectedOffsets(
                    targetSupport, sourceSupport, targetAnchor, selectedSource,
                    axis, normal) >= 0)
                continue;
            normal = axis;
            targetAnchor = targetSupport;
            selectedSource = sourceSupport;
        }
    }

    private static bool TryGetHullCapsuleContact(PhysicsMesh hull, Vector3d center,
        FixedQuaternion rotation, Fixed64 axisLength, Fixed64 radius, out FixedContactAnchors contact) =>
        FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hull.Origin, hull.Rotation, hull.ScaledLocalVertices,
            hull.Triangles, hull.ConvexSatEdgeVertexPairs,
            center, rotation, Vector3d.Up, axisLength, radius, out contact);
}
